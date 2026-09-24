#!/usr/bin/env python3
"""Tier-1 tests for ha_history: synthetic exports whose ground truth is known by construction.

Run: python3 -m unittest discover -s tools -p "test_*.py"

These are the only tests that can prove the de-duplication reconstruction is EXACT, because only
here is it known which rows were dropped. Against a real export the best available check is
`sampled + held + missing == len(anchors)`, which is necessary but not sufficient. They are also
the only coverage the headerless single-entity export path gets: all 18 exports in this repo carry
an entity_id column.
"""
import csv
import os
import random
import tempfile
import unittest
from datetime import datetime, timedelta, timezone

import battery_power_from_history as bp
import ha_history as hh

PREFIX = "sensor.test_device_"
EPOCH = datetime(2026, 9, 1, tzinfo=timezone.utc)


def write_rows(path, rows, with_entity_column=True):
    """rows: [(stamp, short_name, state_string)]."""
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        if with_entity_column:
            writer.writerow(["entity_id", "state", "last_changed"])
        else:
            writer.writerow(["state", "last_changed"])
        for stamp, name, state in rows:
            stamp_text = stamp.strftime("%Y-%m-%dT%H:%M:%S.") + f"{stamp.microsecond // 1000:03d}Z"
            if with_entity_column:
                writer.writerow([PREFIX + name, state, stamp_text])
            else:
                writer.writerow([state, stamp_text])


class SyntheticExport:
    """A planted export plus the ground truth about it.

    Publishes every `cadence_s`, with each entity's row offset a few milliseconds inside the
    cluster and in a DIFFERENT order each cycle -- which is what makes the as-of-vs-cluster
    distinction observable at all.
    """

    def __init__(self, cycles=400, cadence_s=300.0, seed=1234, names=None):
        self.names = names or ("hp_awake_time", "radio_tx_time", "radio_rx_time", "cca_failures")
        self.rng = random.Random(seed)
        self.cadence_s = cadence_s
        self.anchors = []
        self.planted = {name: [] for name in self.names}
        for index in range(cycles):
            anchor = EPOCH + timedelta(seconds=index * cadence_s)
            self.anchors.append(anchor)
            for name in self.names:
                if name == "cca_failures":
                    # A counter: mostly zero, so most of its rows get de-duplicated away.
                    value = float(self.rng.choice([0, 0, 0, 0, 0, 0, 1, 2, 7]))
                elif name == "hp_awake_time":
                    value = round(self.rng.uniform(800.0, 1100.0), 3)
                else:
                    value = round(self.rng.uniform(80.0, 400.0), 3)
                self.planted[name].append(value)

    def rows(self):
        """Every row that WOULD be written with no de-duplication, in publish order."""
        out = []
        for index, anchor in enumerate(self.anchors):
            order = list(self.names)
            self.rng.shuffle(order)
            for slot, name in enumerate(order):
                stamp = anchor + timedelta(milliseconds=slot * 3)
                # str(), not %g: %g truncates to 6 significant digits, which would silently make
                # the "reconstruction is exact" assertion below unable to fail.
                out.append((stamp, name, str(self.planted[name][index])))
        return out

    def deduplicated(self):
        """(rows_after_dedup, dropped_count_per_name) -- HA writes a row only on a CHANGE."""
        last = {}
        kept = []
        dropped = {name: 0 for name in self.names}
        for stamp, name, state in self.rows():
            if last.get(name) == state:
                dropped[name] += 1
                continue
            last[name] = state
            kept.append((stamp, name, state))
        return kept, dropped


class CycleReconstructionTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.synthetic = SyntheticExport()
        self.kept, self.dropped = self.synthetic.deduplicated()
        self.path = os.path.join(self.dir.name, "planted.csv")
        write_rows(self.path, self.kept)
        self.export = hh.load_export(self.path)

    def test_anchors_recover_every_planted_cycle(self):
        anchors = hh.cycle_anchors(self.export, self.synthetic.names)
        self.assertEqual(len(anchors), len(self.synthetic.anchors))
        # An anchor is the first SURVIVING row of its cluster, so when the entity that happened to
        # publish first had an unchanged value HA dropped that row and the anchor lands a few ms
        # later. Same cycle, later stamp -- which is exactly why cadence is quoted with an
        # anchor-definition tolerance rather than as an exact figure.
        for found, planted in zip(anchors, self.synthetic.anchors):
            self.assertGreaterEqual(found, planted)
            self.assertLess((found - planted).total_seconds(), hh.CYCLE_EPSILON_S)

    def test_fill_restores_precisely_the_dropped_rows(self):
        anchors = hh.cycle_anchors(self.export, self.synthetic.names)
        table = hh.cycle_table(self.export, anchors, self.synthetic.names)
        for name in self.synthetic.names:
            column = table.columns[name]
            self.assertEqual(column.held, self.dropped[name],
                             f"{name}: held {column.held} != {self.dropped[name]} dropped")
            self.assertEqual(column.missing, 0, name)
            self.assertEqual(column.values, self.synthetic.planted[name],
                             f"{name}: reconstruction is not exact")

    def test_column_counts_partition_the_anchors(self):
        anchors = hh.cycle_anchors(self.export, self.synthetic.names)
        table = hh.cycle_table(self.export, anchors, self.synthetic.names)
        for column in table.columns.values():
            self.assertEqual(column.sampled + column.held + column.missing, len(anchors))

    def test_counter_dedup_is_severe_and_timings_are_not(self):
        """The bias that motivates forward-filling ALWAYS, rather than per-metric judgement."""
        anchors = hh.cycle_anchors(self.export, self.synthetic.names)
        table = hh.cycle_table(self.export, anchors, self.synthetic.names)
        counter = table.columns["cca_failures"]
        observed = hh.series_for(self.export, "cca_failures").samples
        mean_observed = sum(value for _, value in observed) / len(observed)
        self.assertGreater(mean_observed, hh.column_mean(counter) * 1.2)
        timing = table.columns["hp_awake_time"]
        self.assertEqual(timing.held, 0)

    def test_as_of_shifts_where_cycle_table_does_not(self):
        """The canary from docs/plan_ha_log_metrics.md section 5.2, on planted data.

        Entities publish in a different order each cycle, so any entity landing after the anchor
        is pulled back a whole cycle by an as-of join. cycle_table must be immune.
        """
        anchors = hh.cycle_anchors(self.export, self.synthetic.names)
        table = hh.cycle_table(self.export, anchors, self.synthetic.names)
        self.assertEqual(table.columns["radio_rx_time"].values,
                         self.synthetic.planted["radio_rx_time"])
        shifted = hh.as_of_join(anchors, hh.series_for(self.export, "radio_rx_time"))
        self.assertNotEqual(shifted, self.synthetic.planted["radio_rx_time"])

    def test_epsilon_larger_than_the_gap_is_refused(self):
        with self.assertRaises(ValueError):
            hh.cycle_anchors(self.export, self.synthetic.names, epsilon_s=1000.0)


class BlackoutTest(unittest.TestCase):
    """`unavailable` runs, and the difference between one and a real outage."""

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)

    def _export(self, rows):
        path = os.path.join(self.dir.name, "b.csv")
        write_rows(path, rows)
        return hh.load_export(path)

    def test_blackout_run_is_captured_with_its_duration(self):
        rows = [
            (EPOCH, "voltage", "3.9"),
            (EPOCH + timedelta(seconds=300), "voltage", "unavailable"),
            (EPOCH + timedelta(seconds=380), "voltage", "3.9"),
        ]
        series = hh.series_for(self._export(rows), "voltage")
        self.assertEqual(len(series.blackouts), 1)
        self.assertAlmostEqual(series.blackouts[0].seconds, 80.0)
        self.assertEqual(series.blackouts[0].marker, "unavailable")
        self.assertEqual([value for _, value in series.samples], [3.9, 3.9])

    def test_monotone_increments_ignores_a_blackout_repeat(self):
        """heater_run_count reads 9 -> unavailable -> 9 across an HA restart.

        Counting transitions would score that as an increment and corrupt every LP period
        derived from it.
        """
        rows = [
            (EPOCH, "heater_run_count", "9"),
            (EPOCH + timedelta(seconds=300), "heater_run_count", "unavailable"),
            (EPOCH + timedelta(seconds=600), "heater_run_count", "9"),
            (EPOCH + timedelta(seconds=900), "heater_run_count", "10"),
        ]
        increments = hh.monotone_increments(hh.series_for(self._export(rows), "heater_run_count"))
        self.assertEqual(len(increments), 1)
        self.assertEqual(increments[0][1], 10.0)

    def test_constant_value_reads_through_a_blackout(self):
        rows = [
            (EPOCH, "boot_count", "272"),
            (EPOCH + timedelta(seconds=300), "boot_count", "unavailable"),
            (EPOCH + timedelta(seconds=600), "boot_count", "272"),
        ]
        value, complaint = hh.constant_value(hh.series_for(self._export(rows), "boot_count"))
        self.assertEqual(value, 272.0)
        self.assertIsNone(complaint)

    def test_constant_value_complains_when_it_moves(self):
        rows = [
            (EPOCH, "boot_count", "272"),
            (EPOCH + timedelta(seconds=300), "boot_count", "273"),
        ]
        value, complaint = hh.constant_value(hh.series_for(self._export(rows), "boot_count"))
        self.assertIsNone(value)
        self.assertIn("not constant", complaint)

    def test_hold_stops_at_the_dwell_cap(self):
        """A real outage is a plain silence, not a blackout marker -- and must not be filled."""
        rows = [
            (EPOCH, "hp_awake_time", "900"),
            (EPOCH + timedelta(seconds=300), "hp_awake_time", "910"),
            (EPOCH + timedelta(seconds=300 + 20000), "hp_awake_time", "79500"),
        ]
        export = self._export(rows)
        self.assertEqual(hh.series_for(export, "hp_awake_time").blackouts, [])
        anchors = [EPOCH, EPOCH + timedelta(seconds=300),
                   EPOCH + timedelta(seconds=300 + hh.MAX_HOLD_S + 60),
                   EPOCH + timedelta(seconds=300 + 20000)]
        table = hh.cycle_table(export, anchors, ["hp_awake_time"])
        self.assertEqual(table.columns["hp_awake_time"].values, [900.0, 910.0, None, 79500.0])
        self.assertEqual(table.columns["hp_awake_time"].missing, 1)

    def test_non_numeric_states_are_counted_not_crashed_on(self):
        rows = [
            (EPOCH, "heater_problem", "off"),
            (EPOCH + timedelta(seconds=300), "heater_problem", "on"),
        ]
        series = hh.series_for(self._export(rows), "heater_problem")
        self.assertEqual(series.samples, [])
        self.assertEqual(series.non_numeric, 2)


class NamingTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)

    def test_device_prefix_is_trimmed_to_the_last_underscore(self):
        self.assertEqual(
            hh.device_prefix_of(["sensor.esp32_ot_mqtt_outdoor_voltage",
                                 "sensor.esp32_ot_mqtt_outdoor_signal_strength",
                                 "number.esp32_ot_mqtt_outdoor_sensor_samples"]),
            "esp32_ot_mqtt_outdoor_")

    def test_single_entity_has_no_meaningful_prefix(self):
        self.assertEqual(hh.device_prefix_of(["sensor.only_one"]), "")

    def test_series_for_is_exact_not_endswith(self):
        """`endswith("_signal_strength")` would match uplink_signal_strength too."""
        path = os.path.join(self.dir.name, "n.csv")
        write_rows(path, [(EPOCH, "signal_strength", "-61"),
                          (EPOCH, "uplink_signal_strength", "-47")])
        export = hh.load_export(path)
        self.assertEqual(hh.series_for(export, "signal_strength").samples[0][1], -61.0)
        self.assertEqual(hh.series_for(export, "uplink_signal_strength").samples[0][1], -47.0)
        with self.assertRaises(KeyError):
            hh.series_for(export, "strength")

    def test_headerless_single_entity_export(self):
        """No entity_id column at all: every row is the one series, and any name resolves to it.

        No real export in this repo exercises this path, which is why it is planted here.
        """
        path = os.path.join(self.dir.name, "h.csv")
        write_rows(path, [(EPOCH, "voltage", "4.104"),
                          (EPOCH + timedelta(seconds=300), "voltage", "4.100")],
                   with_entity_column=False)
        export = hh.load_export(path)
        self.assertEqual(export.device_prefix, "")
        self.assertEqual(hh.short_names(export), [""])
        self.assertEqual([value for _, value in hh.series_for(export, "voltage").samples],
                         [4.104, 4.100])

    def test_boundary_rows_are_reported_not_removed(self):
        """A constant's export-start snapshot is counted, so anchoring on it is a visible choice."""
        path = os.path.join(self.dir.name, "bd.csv")
        write_rows(path, [(EPOCH, "boot_count", "272"),
                          (EPOCH + timedelta(seconds=300), "voltage", "3.9"),
                          (EPOCH + timedelta(seconds=600), "voltage", "3.8")])
        export = hh.load_export(path)
        self.assertEqual(hh.series_for(export, "boot_count").boundary_stamps, 1)
        self.assertEqual(len(hh.series_for(export, "boot_count").samples), 1)


class StatisticsTest(unittest.TestCase):
    def test_dwell_weighted_mean_weights_by_time_not_row_count(self):
        """Ten fast rows at 30 C then one that stands for hours must not outvote the long one."""
        samples = [(EPOCH + timedelta(seconds=i), 30.0) for i in range(10)]
        samples.append((EPOCH + timedelta(seconds=10), 10.0))
        samples.append((EPOCH + timedelta(seconds=3610), 10.0))
        self.assertLess(hh.dwell_weighted_mean(samples), 11.0)

    def test_equal_weights_reproduce_the_unweighted_fit(self):
        """The identity that makes the weighted fit a generalisation rather than a new estimator."""
        rng = random.Random(3)
        hours = [i * 0.25 for i in range(400)]
        values = [80.0 - 0.01 * h + rng.gauss(0.0, 0.4) for h in hours]
        plain = hh.ordinary_least_squares_slope(hours, values)
        weighted = hh.weighted_least_squares_slope(hours, values, [1.0] * len(hours))
        self.assertAlmostEqual(plain.slope, weighted.slope, places=10)
        self.assertAlmostEqual(plain.intercept, weighted.intercept, places=10)
        # ... and any uniform scaling of the weights is the same fit again.
        scaled = hh.weighted_least_squares_slope(hours, values, [7.5] * len(hours))
        self.assertAlmostEqual(plain.slope, scaled.slope, places=10)

    def test_weighting_recovers_a_slope_that_drifting_density_distorts(self):
        """Dense sampling that MOVES through the diurnal phase is what actually biases a slope.

        Note the "drifting" carefully: a density pattern identical every day biases the
        INTERCEPT, not the slope, and both estimators recover the trend from it about equally.
        The bias needs the dense hours to sit at the wobble's peak early in the window and at
        its trough later, which is the real failure mode -- HA emits a row when the voltage
        moves, and what makes it move drifts with the weather across a multi-day window.
        """
        import math as _math
        truth = -0.02
        hours = []
        for day in range(10):
            centre = 12.0 if day < 5 else 0.0        # peak early, trough late
            hours.extend(day * 24.0 + centre + i * 0.05 for i in range(60))
            hours.extend(day * 24.0 + h for h in (2.0, 6.0, 10.0, 14.0, 18.0, 22.0))
        hours = sorted(h for h in hours if h >= 0.0)
        values = [50.0 + truth * h + 2.0 * _math.sin(h / 24.0 * 2 * _math.pi) for h in hours]
        plain = hh.ordinary_least_squares_slope(hours, values).slope
        weighted = hh.weighted_least_squares_slope(hours, values, bp.dwell_weights(hours)).slope
        self.assertLess(abs(weighted - truth), abs(plain - truth) / 3.0)

    def test_bootstrap_weights_change_the_interval(self):
        """A weighted point estimate must carry a weighted interval, not the unweighted one."""
        rng = random.Random(5)
        hours = [i * 0.1 for i in range(1200)]
        values = [90.0 - 0.03 * h + rng.gauss(0.0, 0.6) for h in hours]
        weights = [1.0 if i % 2 else 5.0 for i in range(len(hours))]
        self.assertNotEqual(hh.slope_block_bootstrap(hours, values).slope,
                            hh.slope_block_bootstrap(hours, values, weights).slope)

    def test_bootstrap_is_deterministic_and_brackets_the_slope(self):
        rng = random.Random(7)
        hours = [i * 0.1 for i in range(2000)]
        values = [100.0 - 0.02 * h + rng.gauss(0.0, 0.5) for h in hours]
        first = hh.slope_block_bootstrap(hours, values)
        second = hh.slope_block_bootstrap(hours, values)
        self.assertEqual(first, second)
        self.assertLess(first.low, first.slope)
        self.assertGreater(first.high, first.slope)
        self.assertEqual(first.confidence, 0.95)
        self.assertGreater(first.blocks, 1)

    def test_bootstrap_widens_with_autocorrelated_residuals(self):
        """The whole reason for whole-block resampling: a daily ripple is not i.i.d. noise."""
        import math as _math
        hours = [i * 0.1 for i in range(2000)]
        clean = [100.0 - 0.02 * h for h in hours]
        rippled = [clean[i] + 3.0 * _math.sin(hours[i] / 24.0 * 2 * _math.pi) for i in range(len(hours))]
        rng = random.Random(11)
        noisy = [clean[i] + rng.gauss(0.0, 3.0) for i in range(len(hours))]
        self.assertGreater(hh.slope_block_bootstrap(hours, rippled).half_width,
                           hh.slope_block_bootstrap(hours, noisy).half_width)


if __name__ == "__main__":
    unittest.main()
