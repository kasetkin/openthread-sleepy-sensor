#!/usr/bin/env python3
"""Tier-2 and tier-3 regression tests: the two real measurement windows.

Run: python3 -m unittest discover -s tools -p "test_*.py"

Tier 2 reads the committed tools/testdata/*.csv.gz fixtures, so these run on every machine. Tier 3
reads the untracked raw exports when they happen to be present, for the few facts the 11-entity
subset cannot carry.

Every expectation below is one line with its own tolerance and a note naming where the figure
comes from, because a bare pair of numbers in a failure message is not enough to tell a real
regression from a re-blessing that never happened. Quantities section 6 marks EXACT use
assertEqual -- a tolerance there would hide exactly the off-by-one-cycle bug that the
cluster-scoped fill exists to prevent.
"""
import os
import unittest
from collections import namedtuple

import ha_history as hh
import ha_log_metrics as hlm
import power_model as pm

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
FIXTURE_A = os.path.join(HERE, "testdata", "window_a_ss4.csv.gz")
FIXTURE_B = os.path.join(HERE, "testdata", "window_b_ss16.csv.gz")
RAW_A = os.path.join(REPO, "metrics_history_2026-08-29__to__2026-09-07.csv")

# kind: "exact" (assertEqual), "abs" (+/- tolerance in the value's own unit), "rel" (fraction).
Expect = namedtuple("Expect", "name value tolerance kind note")

WINDOW_A = (
    Expect("n_cycles", 3390, 0, "exact", "8-entity measurement anchor; all 26 give 3395"),
    Expect("seconds", 794182, 1.0, "abs", "last anchor - first anchor"),
    Expect("cadence_s", 234.34, 0.5, "abs", "T/(n-1); anchor-definition sensitive"),
    Expect("mode_s", 304.1, 1.0, "abs", "(max_skip+1) x lp_poll_period = 15 polls"),
    Expect("mode_share", 0.56, 0.02, "abs", "the rest are dT/dRH-triggered"),
    Expect("tx_s", 335.6, 0.001, "rel", "sum of forward-filled radio_tx_time"),
    Expect("rx_s", 1234.8, 0.001, "rel", "sum of forward-filled radio_rx_time"),
    Expect("cpu_s", 1689.1, 0.001, "rel", "per-cycle awake-tx-rx, clamped, summed"),
    Expect("tx_ms", 99.01, 0.001, "rel", "power_model PHASE_TIMES_WINDOW_A_S[0]"),
    Expect("rx_ms", 364.25, 0.001, "rel", "power_model PHASE_TIMES_WINDOW_A_S[1]"),
    Expect("cpu_ms", 498.26, 0.001, "rel", "power_model PHASE_TIMES_WINDOW_A_S[2]"),
    Expect("negatives", 0, 0, "exact", "a nonzero count means the anchoring is wrong"),
    Expect("lp_raw_s", 20.3442, 0.00033, "rel", "heater span / (4320 x 8 intervals)"),
    Expect("lp_cross_s", 20.3559, 0.001, "rel", "safeguard pool p90 / k=15"),
    Expect("lp_interval_raw_s", 20.0002, 0.01, "abs", "device_config.yaml lp_poll_interval_sec 20"),
    Expect("stall_s", 96.0, 3.0, "abs", "kRhSettleMs 90 s + ~3 x kCooldownPollMs 2 s"),
    Expect("outlier_count", 15, 0, "exact", "hp_awake_time > 5 s"),
    Expect("outlier_s", 297.8, 1.0, "abs", "9.1 % of all HP-awake time"),
    Expect("multiplier", 0.9195, 0.001, "abs", "arrhenius_multiplier docstring's 0.920"),
    Expect("measured_ua", 600.0, 1.0, "abs", "charge-correct; battery%-direct fit gives 600.2"),
    Expect("sensor_samples", 4, 0, "exact", "number.*_sensor_samples, flat"),
    Expect("tx_power_dbm", 20.0, 0, "exact", "tx_power_active, flat"),
    Expect("model_mid_ua", 494.5, 0.05, "abs", "power_model REGRESSION_TABLE window A row"),
    Expect("model_device_ua", 458.7, 0.05, "abs", "power_model REGRESSION_TABLE window A row"),
)

WINDOW_B = (
    Expect("n_cycles", 3015, 0, "exact", "not outage-free: contains the 2 h 16 m blackout"),
    Expect("seconds", 720791, 1.0, "abs", "last anchor - first anchor"),
    Expect("cadence_s", 239.15, 0.5, "abs", "T/(n-1)"),
    Expect("mode_s", 325.5, 1.0, "abs", "15 polls at the longer ss=16 period"),
    Expect("mode_share", 0.54, 0.02, "abs", "mode share is weather, not a device setting"),
    Expect("tx_s", 297.1, 0.001, "rel", "sum of forward-filled radio_tx_time"),
    Expect("rx_s", 954.7, 0.001, "rel", "sum of forward-filled radio_rx_time"),
    Expect("cpu_s", 1514.3, 0.001, "rel", "per-cycle awake-tx-rx, clamped, summed"),
    Expect("tx_ms", 98.57, 0.001, "rel", "power_model PHASE_TIMES_WINDOW_B_S[0]"),
    Expect("rx_ms", 316.76, 0.001, "rel", "power_model PHASE_TIMES_WINDOW_B_S[1]"),
    Expect("cpu_ms", 502.41, 0.001, "rel", "power_model PHASE_TIMES_WINDOW_B_S[2]"),
    Expect("negatives", 0, 0, "exact", "survives the outage recovery cycle"),
    Expect("lp_raw_s", 21.7868, 0.00051, "rel", "heater span / (4320 x 5 intervals)"),
    Expect("lp_cross_s", 21.7928, 0.001, "rel", "safeguard pool p90 / k=15"),
    Expect("lp_interval_raw_s", 20.1108, 0.01, "abs",
           "the +110.8 ms ss=16 anomaly, from an independent code path"),
    Expect("stall_s", 97.0, 3.0, "abs", "agrees with window A to 1 s at a different LP period"),
    Expect("outlier_count", 3, 0, "exact", "one is the 79.5 s outage-recovery cycle"),
    Expect("outlier_s", 91.4, 1.0, "abs", "3.3 % of all HP-awake time"),
    Expect("multiplier", 1.0327, 0.001, "abs", "arrhenius_multiplier docstring's 1.033"),
    Expect("measured_ua", 1253.4, 1.0, "abs", "charge-correct; printed 1168.6 before the fix"),
    Expect("sensor_samples", 16, 0, "exact", "number.*_sensor_samples, flat"),
    Expect("tx_power_dbm", 20.0, 0, "exact", "tx_power_active, flat"),
    Expect("model_mid_ua", 1011.6, 0.05, "abs", "power_model REGRESSION_TABLE window B row"),
    Expect("model_device_ua", 920.5, 0.05, "abs", "power_model REGRESSION_TABLE window B row"),
)


def measure(path):
    """Flatten a derive() run into the flat name -> value mapping the tables above pin."""
    findings = hlm.derive(hh.load_export(path))
    tx_s, rx_s, cpu_s = hlm.model_phase_times(findings)
    budget = pm.power_budget(findings.window.cadence_s, findings.sensor_samples,
                             findings.lp.raw_s, findings.multiplier, None,
                             findings.measured.ua, findings.tx_power_dbm,
                             (tx_s, rx_s, cpu_s))
    return findings, {
        "n_cycles": findings.window.n_cycles,
        "seconds": findings.window.seconds,
        "cadence_s": findings.window.cadence_s,
        "mode_s": findings.safeguard.mode_s,
        "mode_share": findings.safeguard.share,
        "tx_s": findings.phases.tx_s,
        "rx_s": findings.phases.rx_s,
        "cpu_s": findings.phases.cpu_s,
        "tx_ms": tx_s * 1000.0,
        "rx_ms": rx_s * 1000.0,
        "cpu_ms": cpu_s * 1000.0,
        "negatives": findings.phases.negatives,
        "lp_raw_s": findings.lp.raw_s,
        "lp_cross_s": findings.lp.cross_check_s,
        "lp_interval_raw_s": findings.lp.interval_raw_s,
        "stall_s": findings.lp.stall_s,
        "outlier_count": findings.phases.outlier_count,
        "outlier_s": findings.phases.outlier_s,
        "multiplier": findings.multiplier,
        "measured_ua": findings.measured.ua,
        "sensor_samples": findings.sensor_samples,
        "tx_power_dbm": findings.tx_power_dbm,
        "model_mid_ua": budget.mid_ua,
        "model_device_ua": budget.device_ua,
    }


class FixtureMixin:
    def assert_table(self, table, measured):
        for expect in table:
            got = measured[expect.name]
            detail = f"{expect.name}: expected {expect.value} got {got} -- {expect.note}"
            if expect.kind == "exact":
                self.assertEqual(got, expect.value, detail)
            elif expect.kind == "abs":
                self.assertLessEqual(abs(got - expect.value), expect.tolerance, detail)
            else:
                self.assertLessEqual(abs(got / expect.value - 1.0), expect.tolerance, detail)


class WindowATest(unittest.TestCase, FixtureMixin):
    @classmethod
    def setUpClass(cls):
        cls.findings, cls.measured = measure(FIXTURE_A)

    def test_section_6_table(self):
        self.assert_table(WINDOW_A, self.measured)

    def test_every_guard_rail_passes(self):
        failed = [guard.name for guard in self.findings.guards if not guard.ok]
        self.assertEqual(failed, [], f"expected a clean window, got {failed}")

    def test_phase_times_match_the_models_pinned_pair(self):
        """The extractor and power_model must not drift apart on the same window."""
        self.assertEqual(
            tuple(round(value, 5) for value in hlm.model_phase_times(self.findings)),
            pm.PHASE_TIMES_WINDOW_A_S)

    def test_fill_is_auditable_on_real_data(self):
        for column in self.findings.window.table.columns.values():
            self.assertEqual(column.sampled + column.held + column.missing,
                             self.findings.window.n_cycles, column.name)
        self.assertEqual(self.findings.window.table.columns["radio_tx_time"].held, 126)

    def test_command_line_is_reproducible(self):
        line = hlm.model_command_line(self.findings)
        self.assertIn("--lp-poll-period 20.3442", line)
        self.assertIn("--sensor-samples 4", line)
        self.assertIn("--tx-power 20", line)
        self.assertNotIn("--lp-poll-interval", line)

    def test_corrected_lp_period_sits_below_the_raw_one(self):
        """The heater interval is 4320*P + stall, so the raw quotient overshoots."""
        lp = self.findings.lp
        self.assertLess(lp.corrected_s, lp.raw_s)
        self.assertAlmostEqual(lp.raw_s - lp.corrected_s,
                               lp.stall_s / hlm.HEATER_TICKS_PER_STEP, places=6)
        self.assertGreater(lp.raw_s - lp.corrected_s, lp.uncertainty_frac * lp.raw_s,
                           "the stall bias must exceed the quantization uncertainty, which is "
                           "the whole reason both figures are reported")


class WindowBTest(unittest.TestCase, FixtureMixin):
    @classmethod
    def setUpClass(cls):
        cls.findings, cls.measured = measure(FIXTURE_B)

    def test_section_6_table(self):
        self.assert_table(WINDOW_B, self.measured)

    def test_only_the_edge_blackout_warns(self):
        failed = [guard.name for guard in self.findings.guards if not guard.ok]
        self.assertEqual(failed, ["edge blackouts"])
        self.assertTrue(all(guard.ok or not guard.fatal for guard in self.findings.guards))

    def test_phase_times_match_the_models_pinned_pair(self):
        self.assertEqual(
            tuple(round(value, 5) for value in hlm.model_phase_times(self.findings)),
            pm.PHASE_TIMES_WINDOW_B_S)

    def test_the_outage_is_a_silence_not_a_blackout(self):
        """A real outage leaves no `unavailable` marker at all -- only a gap between anchors.

        This is why section 2.4 refuses to classify: the CSV cannot tell an HA restart from a
        device outage, so the extractor takes a dwell cap as a parameter and reports gaps.
        """
        gaps = hh.anchor_gaps(self.findings.window.anchors)
        self.assertGreater(max(gaps), 8000.0)
        self.assertLess(max(gaps), 8300.0)


@unittest.skipUnless(os.path.exists(RAW_A), "raw export not present (untracked working file)")
class RawExportTest(unittest.TestCase):
    """Tier 3: facts the 11-entity fixture structurally cannot carry."""

    def test_anchoring_on_every_entity_invents_cycles(self):
        """The correction that section 2.1 exists for, demonstrated rather than asserted.

        Constants carry HA's export-boundary snapshot rows. Anchoring on them adds five cycles
        that never happened and moves the cadence by 0.3 %.
        """
        export = hh.load_export(RAW_A)
        measurement_anchors = hh.cycle_anchors(export, hlm.PER_CYCLE_ENTITIES)
        every_anchor = hh.cycle_anchors(export, tuple(hh.short_names(export)))
        self.assertEqual(len(measurement_anchors), 3390)
        self.assertEqual(len(every_anchor), 3395)

    def test_mqtt_client_start_is_not_a_safe_cycle_clock(self):
        """It is de-duplicated like everything else, and is not even the densest series."""
        export = hh.load_export(RAW_A)
        self.assertEqual(len(hh.series_for(export, "mqtt_client_start").samples), 3388)
        self.assertEqual(len(hh.series_for(export, "hp_awake_time").samples), 3390)


if __name__ == "__main__":
    unittest.main()
