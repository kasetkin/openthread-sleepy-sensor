#!/usr/bin/env python3
"""Shared reader for Home Assistant history CSV exports.

Every analysis tool in this directory starts by turning an HA export into per-entity time series,
and they were each about to grow their own copy of that code. This module is the one copy.

Three properties of HA exports drive the whole design and are worth stating once.

**HA de-duplicates.** A row is written only when the value CHANGES, so the surviving rows are a
biased sample of cycles -- mildly for timings, which change every cycle, and severely for
counters, where repeated zeros vanish and the survivors are all the nonzero ones. On the
2026-08-29 -> 09-07 window `cca_failures` reads 3.88 over the surviving rows against 2.54 over
reconstructed cycles, a +53 % bias, while `radio_rx_time` moves by 0.0 %.

Reconstructing a dropped row is therefore not interpolation, it is exact: HA drops a row ONLY
when the value is unchanged, so the reconstructed value is what the device published. Verified on
that window -- `radio_tx_time` has 3264 rows, 428 distinct values, and zero surviving
consecutive-equal pairs, so the 126 cells this module fills are precisely the 126 repeats HA
removed. That is why there is no interpolation code here and no `method=` parameter for anyone to
add one to: the alternative has to be unimplemented, not merely discouraged.

**`unavailable` and `unknown` are not values.** They appear in every export, usually because Home
Assistant restarted rather than because the device went away, and the two are NOT distinguishable
from the CSV. They are split out of the numeric series rather than parsed, dropped, or silently
held across.

**Entities that never change carry boundary rows.** HA stamps a state snapshot at the export's
start instant, which for a constant like `boot_count` or `tx_power_active` may be most of the rows
it has. Anchoring cycles on such an entity invents cycles that never happened -- five of them on
that same window. `boundary_stamps` below reports them; choosing which entities to anchor on is
the caller's job, and it should choose per-cycle measurements only.

See docs/plan_ha_log_metrics.md for the derivations behind all three.
"""
import csv
import gzip
import math
import random
import statistics
from bisect import bisect_left, bisect_right
from collections import namedtuple
from datetime import datetime, timedelta

# HA's two non-value states. Neither is parseable as a float and neither means "no change".
NOT_A_NUMBER = ("unavailable", "unknown")

# How long a held value stays credible. A chosen threshold, not a derived one: the safeguard
# publish gap on this device is 304-326 s, so ~20x that is certainly a real gap rather than a
# de-duplicated repeat. Do NOT read it as expire_after: that is 680 s and unrelated.
MAX_HOLD_S = 7200.0

# Cluster width for grouping the entities of one MQTT publish into one cycle. Measured, the whole
# cluster spans at most 16 ms. The smallest real inter-publish gap was 16.1 s on the 2026-09
# reference windows, but 14.3 s on 2026-09-23, where a failed publish is retried one 20 s LP poll
# later and one of the two lands late -- and cycle_anchors() refuses anything under 1.5x epsilon,
# so the old 10 s turned a routine retry into a refusal. 5 s still sits ~300x above the cluster
# spread and is refused only below 7.5 s. A parameter and not a law.
CYCLE_EPSILON_S = 5.0


class AnchorError(ValueError):
    """The export cannot be split into publish cycles as asked.

    No samples at all, or an epsilon too close to the smallest gap. Both are properties of the
    data rather than bugs, so a caller refuses the window instead of crashing on it.
    """


# Bootstrap recipe, pinned so the interval a run prints is the interval another run prints. The
# figure this replaces (+/-0.077 mA, from the 2026-08-29 power-budget report) could not be
# reproduced from its description, which is the whole argument for fixing the recipe in code
# rather than carrying a constant. Seed included deliberately: an unreproducible confidence
# interval is barely better than none.
BOOTSTRAP_BLOCK_HOURS = 24.0
BOOTSTRAP_REPLICATIONS = 2000
BOOTSTRAP_CONFIDENCE = 0.95
BOOTSTRAP_SEED = 20260909

# Home Assistant multi-entity exports carry every sensor in one file, so rows have to be selected
# by entity_id or humidity/RSSI/counter rows get parsed as "voltages". Single-entity exports have
# no entity_id column at all -- then every row belongs to the one series.
VOLTAGE_SUFFIX = "_voltage"
TEMPERATURE_SUFFIX = "_temperature"

# `edge` marks a blackout anchored to an export boundary: one that is already in progress at the
# first row, or that never recovers before the last. Neither says anything about the device --
# the leading kind is Home Assistant not yet knowing a slow-changing entity's value, and on the
# 2026-08-20 -> 08-29 window that makes heater_run_count read `unavailable` for its first 26
# hours purely because it only changes once a day. Interior blackouts are the informative ones.
Blackout = namedtuple("Blackout", "start end seconds marker edge")
Series = namedtuple("Series", "name entity_id samples blackouts boundary_stamps non_numeric")
Export = namedtuple("Export", "path device_prefix series start end")
Column = namedtuple("Column", "name values sampled held missing")
CycleTable = namedtuple("CycleTable", "anchors columns epsilon_s max_hold_s")
LinearFit = namedtuple("LinearFit", "slope intercept")
Bootstrap = namedtuple("Bootstrap", "slope sigma half_width low high confidence block_hours "
                                    "replications blocks seed")


def parse_timestamp(text):
    """HA's ISO-8601-with-Z stamp -> timezone-aware datetime."""
    return datetime.fromisoformat(text.replace("Z", "+00:00"))


def open_export(path):
    """Open a .csv or .csv.gz export for reading as text.

    The gzip branch is two lines and it is what lets the committed regression fixtures be a fifth
    the size of the raw exports, so they can be tracked at all; it also lets a user keep their own
    exports compressed.
    """
    if str(path).endswith(".gz"):
        return gzip.open(path, "rt", newline="")
    return open(path, newline="")


def device_prefix_of(entity_ids):
    """Common prefix of the domain-stripped entity ids, trimmed back to the last underscore.

    Returns "" when fewer than two ids make a prefix meaningful. On this project's exports it
    yields "esp32_ot_mqtt_outdoor_" across all 26 entities, spanning sensor./number./binary_sensor.
    """
    stripped = [eid.split(".", 1)[-1] for eid in entity_ids]
    if len(stripped) < 2:
        return ""
    prefix = stripped[0]
    for name in stripped[1:]:
        while not name.startswith(prefix):
            prefix = prefix[:-1]
            if not prefix:
                return ""
    return prefix[:prefix.rfind("_") + 1] if "_" in prefix else ""


def load_export(path, device_prefix=None):
    """Read an HA export into an Export of short-name-keyed Series.

    Numeric rows go to `samples`; `unavailable`/`unknown` runs are collapsed into `blackouts`
    (each ending at the next numeric row, or at the export end if it never recovers). Rows whose
    timestamp sits exactly on the export's start or end instant are counted in `boundary_stamps`
    but are NOT removed -- they are usually a real last-known value with a synthetic timestamp,
    and dropping them would silently change every cycle count derived from this file.
    """
    rows = []
    entity_ids = set()
    with open_export(path) as handle:
        reader = csv.DictReader(handle)
        has_entity_column = reader.fieldnames is not None and "entity_id" in reader.fieldnames
        for row in reader:
            entity = row["entity_id"] if has_entity_column else ""
            rows.append((parse_timestamp(row["last_changed"]), entity, row["state"]))
            if entity:
                entity_ids.add(entity)
    if not rows:
        raise ValueError(f"{path}: no rows")
    rows.sort(key=lambda r: (r[0], r[1]))
    start = rows[0][0]
    end = rows[-1][0]

    if device_prefix is None:
        device_prefix = device_prefix_of(sorted(entity_ids)) if entity_ids else ""

    by_entity = {}
    for stamp, entity, state in rows:
        by_entity.setdefault(entity, []).append((stamp, state))

    series = {}
    for entity, entity_rows in by_entity.items():
        short = entity.split(".", 1)[-1]
        if device_prefix and short.startswith(device_prefix):
            short = short[len(device_prefix):]
        samples = []
        blackouts = []
        boundary = 0
        non_numeric = 0
        pending_start = None
        pending_marker = None
        for stamp, state in entity_rows:
            if state in NOT_A_NUMBER:
                if pending_start is None:
                    pending_start = stamp
                    pending_marker = state
                continue
            try:
                value = float(state)
            except ValueError:
                # A genuinely non-numeric entity: binary_sensor's on/off, csl_status's text. Not a
                # blackout and not a value -- counted so an empty series is explicable, then
                # skipped, because everything downstream of here does arithmetic.
                non_numeric += 1
                continue
            if pending_start is not None:
                blackouts.append(Blackout(start=pending_start, end=stamp,
                                          seconds=(stamp - pending_start).total_seconds(),
                                          marker=pending_marker, edge=pending_start == start))
                pending_start = None
            samples.append((stamp, value))
            if stamp == start or stamp == end:
                boundary += 1
        if pending_start is not None:
            blackouts.append(Blackout(start=pending_start, end=end,
                                      seconds=(end - pending_start).total_seconds(),
                                      marker=pending_marker, edge=True))
        series[short] = Series(name=short, entity_id=entity, samples=samples,
                               blackouts=blackouts, boundary_stamps=boundary,
                               non_numeric=non_numeric)
    return Export(path=str(path), device_prefix=device_prefix, series=series, start=start, end=end)


def short_names(export):
    """Sorted short names present in the export, for --list-entities."""
    return sorted(export.series)


def series_for(export, name, required=True):
    """One Series by EXACT short name.

    Exact, not `endswith`: `_signal_strength` matches both `signal_strength` and
    `uplink_signal_strength`, and `_mqtt_state_publish` matches `mqtt_state_publish_raw`. All four
    are present in every metrics export from this device, so the ambiguity is one entity name away
    at all times.

    An export carrying exactly ONE series resolves any requested name to it, since such an export
    cannot be ambiguous. That covers both shapes it comes in: a headerless export with no
    entity_id column (its series is keyed by the empty name), and a single-entity export that DOES
    have the column -- the repo's own voltage_history.csv is the latter, and because a lone entity
    id yields no meaningful device prefix its series is keyed by the full "esp32_ot_mqtt_outdoor_
    voltage" rather than "voltage".
    """
    if name in export.series:
        return export.series[name]
    if len(export.series) == 1:
        return next(iter(export.series.values()))
    if not required:
        return None
    raise KeyError(f"{export.path}: no series {name!r}; available: {', '.join(short_names(export))}")


def cycle_anchors(export, names, epsilon_s=CYCLE_EPSILON_S):
    """Cluster the union of the named series' timestamps into one anchor per publish cycle.

    `names` must be the PER-CYCLE MEASUREMENT entities. Including an entity whose value rarely
    changes adds phantom anchors from its boundary rows: on the 2026-08-29 -> 09-07 window the
    eight measurement entities give 3390 cycles at 234.341 s, while all 26 give 3395 at 233.996 s
    and no single entity is dense enough to anchor on alone.

    Raises AnchorError when epsilon is too large to separate the cycles it found, rather than
    silently merging two publishes into one.
    """
    stamps = sorted(stamp
                    for name in names if name in export.series
                    for stamp, _ in export.series[name].samples)
    if not stamps:
        raise AnchorError(f"{export.path}: no samples in {names}")
    window = timedelta(seconds=epsilon_s)
    anchors = []
    for stamp in stamps:
        if not anchors or stamp - anchors[-1] >= window:
            anchors.append(stamp)
    gaps = anchor_gaps(anchors)
    if gaps and min(gaps) < epsilon_s * 1.5:
        raise AnchorError(
            f"{export.path}: cycle epsilon {epsilon_s} s is too close to the smallest gap "
            f"{min(gaps):.1f} s -- real cycles are at risk of being merged")
    return anchors


def cycle_table(export, anchors, names, epsilon_s=CYCLE_EPSILON_S, max_hold_s=MAX_HOLD_S):
    """Cluster-scoped assign-then-hold: one row per anchor, one column per name.

    Use this for every series CO-PUBLISHED with the anchor, which on this device is all ~18
    per-cycle entities -- they ride one MQTT message and land within 16 ms of each other, but not
    in a fixed order. Assigning by "most recent at or before the anchor" (as_of) instead shifts
    any entity that published a few milliseconds AFTER the anchor back by a whole cycle: on the
    2026-08-29 -> 09-07 window that fabricates 12 negative CPU residuals, where this function
    produces none.

    A column cell is `sampled` when the entity published inside [anchor, anchor + epsilon_s),
    `held` when it did not and the previous value is still within max_hold_s, `missing`
    otherwise. `sampled + held + missing == len(anchors)` always, so every filled cell is
    auditable -- `held` is exactly the count of HA-dropped repeats put back.
    """
    window = timedelta(seconds=epsilon_s)
    columns = {}
    for name in names:
        series = series_for(export, name)
        stamps = [stamp for stamp, _ in series.samples]
        values = [value for _, value in series.samples]
        cells = []
        sampled = 0
        held = 0
        missing = 0
        for anchor in anchors:
            inside = bisect_left(stamps, anchor + window) - 1
            if inside >= 0 and stamps[inside] >= anchor:
                cells.append(values[inside])
                sampled += 1
                continue
            before = bisect_left(stamps, anchor) - 1
            if before >= 0 and (anchor - stamps[before]).total_seconds() <= max_hold_s:
                cells.append(values[before])
                held += 1
            else:
                cells.append(None)
                missing += 1
        assert sampled + held + missing == len(anchors)
        columns[name] = Column(name=name, values=cells, sampled=sampled, held=held,
                               missing=missing)
    return CycleTable(anchors=list(anchors), columns=columns, epsilon_s=epsilon_s,
                      max_hold_s=max_hold_s)


def as_of(series, stamp, max_hold_s=MAX_HOLD_S, stamps=None):
    """Most recent sample at or before `stamp`, or None past max_hold_s.

    NOT for per-cycle metrics -- use cycle_table() for anything co-published with the cycle
    anchor, and see its docstring for what this costs (a whole-cycle shift and 12 fabricated
    negative residuals on the reference window). This is for series INDEPENDENT of the anchor,
    such as joining ambient temperature onto battery voltage.

    Pass `stamps` to reuse a precomputed timestamp list across many calls.
    """
    if stamps is None:
        stamps = [sample_stamp for sample_stamp, _ in series.samples]
    index = bisect_right(stamps, stamp) - 1
    if index < 0 or (stamp - stamps[index]).total_seconds() > max_hold_s:
        return None
    return series.samples[index][1]


def as_of_join(stamps, series, max_hold_s=MAX_HOLD_S):
    """Vectorised as_of over many stamps. Same warning as as_of: independent series only."""
    series_stamps = [sample_stamp for sample_stamp, _ in series.samples]
    return [as_of(series, stamp, max_hold_s, series_stamps) for stamp in stamps]


def dwell_weighted_mean(samples, max_dwell_s=MAX_HOLD_S):
    """Mean of a de-duplicated series weighted by how long each value STOOD, not by row count.

    An unweighted mean over-represents whatever changes fastest -- hot afternoons generate far
    more rows than quiet nights.
    """
    weighted = 0.0
    total_s = 0.0
    for (stamp, value), (next_stamp, _) in zip(samples, samples[1:]):
        dwell_s = (next_stamp - stamp).total_seconds()
        if not 0.0 < dwell_s < max_dwell_s:
            continue
        weighted += value * dwell_s
        total_s += dwell_s
    return weighted / total_s if total_s else None


def anchor_gaps(anchors, min_seconds=None):
    """Inter-anchor gaps in seconds, optionally only those at or above min_seconds."""
    gaps = [(anchors[i + 1] - anchors[i]).total_seconds() for i in range(len(anchors) - 1)]
    if min_seconds is None:
        return gaps
    return [gap for gap in gaps if gap >= min_seconds]


def gap_histogram(anchors, bin_s=5.0):
    """{bin_index: count} over the inter-anchor gaps; bin_index * bin_s is the bin's low edge."""
    histogram = {}
    for gap in anchor_gaps(anchors):
        index = int(gap // bin_s)
        histogram[index] = histogram.get(index, 0) + 1
    return histogram


def percentile(sorted_values, fraction):
    """Linear-interpolated percentile of an already-sorted list."""
    if not sorted_values:
        return None
    position = (len(sorted_values) - 1) * fraction
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return sorted_values[int(position)]
    return sorted_values[low] + (sorted_values[high] - sorted_values[low]) * (position - low)


def constant_value(series, allow_blackouts=True):
    """(value, complaint_or_None) for a series that is supposed to be constant.

    The guard-rail primitive. Compares the NUMERIC subsequence only, so a `272 -> unavailable ->
    272` run reads as flat rather than raising on a float parse or reporting a reboot that never
    happened.
    """
    values = {value for _, value in series.samples}
    if not values:
        detail = f" ({series.non_numeric} non-numeric rows)" if series.non_numeric else ""
        return None, f"{series.name}: no numeric samples{detail}"
    if len(values) > 1:
        ordered = sorted(values)
        return None, (f"{series.name}: not constant, saw "
                      f"{', '.join(f'{v:g}' for v in ordered)}")
    if series.blackouts and not allow_blackouts:
        return values.pop(), f"{series.name}: constant but has {len(series.blackouts)} blackout(s)"
    return values.pop(), None


def monotone_increments(series):
    """Samples where the value stepped up by exactly one over the NUMERIC subsequence.

    "Count the transitions" is the wrong implementation: `heater_run_count` reads
    9 -> unavailable -> 9 across an HA restart, and counting changes double-counts an increment,
    which corrupts any period derived from it.
    """
    increments = []
    for index in range(1, len(series.samples)):
        if series.samples[index][1] == series.samples[index - 1][1] + 1:
            increments.append(series.samples[index])
    return increments


def column_total(column):
    """Sum over the non-None cells."""
    return sum(value for value in column.values if value is not None)


def column_mean(column):
    """Mean over the non-None cells, or None when there are none.

    NB this divides by the covered-cell count, not by len(anchors). For a column with `missing`
    cells the two differ -- on the reference window's `cca_failures` (92 missing of 3390) they are
    2.54 and 2.47. Say which one a printed figure used.
    """
    covered = [value for value in column.values if value is not None]
    return statistics.fmean(covered) if covered else None


def blackouts_over(series, seconds, include_edge=False):
    """Blackout runs longer than a threshold.

    Edge-anchored runs are excluded by default -- see Blackout's comment. Their duration is a
    property of when the export was cut, not of anything that happened.
    """
    return [blackout for blackout in series.blackouts
            if blackout.seconds > seconds and (include_edge or not blackout.edge)]


def ordinary_least_squares_slope(hours, values):
    """Slope and intercept of a simple linear fit, as a statistics.LinearRegression."""
    return statistics.linear_regression(hours, values)


def weighted_least_squares_slope(hours, values, weights):
    """Slope and intercept of a linear fit weighting each sample by `weights`.

    Same normal equations as the unweighted fit with w_i folded into every sum, so passing equal
    weights reproduces ordinary_least_squares_slope() exactly.

    The reason this exists: Home Assistant de-duplicates unchanged values, so a row is emitted
    when the value MOVES, and on this device the battery voltage moves with the diurnal
    temperature swing. Rows are therefore ~4x denser at midday than before dawn, and an
    unweighted fit silently weights the estimate toward whatever time of day is noisiest.
    Weighting each sample by how long it STOOD is the continuous-time fit -- the same
    reconstruction argument as forward-filling onto a cycle anchor, applied to a regression.

    NB this de-biases the sampling, which is a small effect here (0.5 % on a nine-day window,
    3 % on a two-day one). It does NOT remove the diurnal signal itself, which is a function of
    time and survives any reweighting.
    """
    total_w = sum(weights)
    if total_w <= 0.0:
        raise ValueError("weights sum to zero -- every sample was dropped")
    mean_x = sum(w * x for w, x in zip(weights, hours)) / total_w
    mean_y = sum(w * y for w, y in zip(weights, values)) / total_w
    covariance = sum(w * (x - mean_x) * (y - mean_y) for w, x, y in zip(weights, hours, values))
    variance = sum(w * (x - mean_x) ** 2 for w, x in zip(weights, hours))
    if variance <= 0.0:
        raise ValueError("weighted x-variance is zero -- the window has no time span")
    slope = covariance / variance
    return LinearFit(slope=slope, intercept=mean_y - slope * mean_x)


def slope_block_bootstrap(hours, values, weights=None, block_hours=BOOTSTRAP_BLOCK_HOURS,
                          replications=BOOTSTRAP_REPLICATIONS, confidence=BOOTSTRAP_CONFIDENCE,
                          seed=BOOTSTRAP_SEED):
    """Confidence interval on a least-squares slope by resampling residuals in whole blocks.

    `weights` makes the fit -- point estimate AND every replication -- a weighted one. It must be
    the SAME weighting the caller's headline slope uses: an interval computed on a different
    estimator than the number it is quoted beside is not an interval on that number.

    Whole blocks because the residuals are strongly autocorrelated -- this pack's terminal voltage
    swings 20-24 mV every day with the enclosure temperature, against a 28 mV signal across nine
    days -- and an i.i.d. bootstrap would treat that daily ripple as independent noise and report
    an interval several times too narrow.

    Every knob is pinned at module scope INCLUDING the seed, because the number this replaces was
    a bare +/-0.077 mA whose method was recorded only as "24 h residual block bootstrap" and could
    not be reproduced from that description. Re-running must give the same interval, or the
    interval is not evidence.

    Returns a Bootstrap. `half_width` is the half-width of the `confidence` interval, not a sigma;
    print it as an interval and name the confidence level.
    """
    def refit(sample_values):
        if weights is None:
            return ordinary_least_squares_slope(hours, sample_values)
        return weighted_least_squares_slope(hours, sample_values, weights)

    fit = refit(values)
    residuals = [values[i] - (fit.intercept + fit.slope * hours[i]) for i in range(len(hours))]

    blocks = []
    block_start = 0
    for index in range(len(hours)):
        if hours[index] - hours[block_start] >= block_hours:
            blocks.append((block_start, index))
            block_start = index
    blocks.append((block_start, len(hours)))

    rng = random.Random(seed)
    slopes = []
    for _ in range(replications):
        resampled = []
        while len(resampled) < len(hours):
            low, high = blocks[rng.randrange(len(blocks))]
            resampled.extend(residuals[low:high])
        resampled = resampled[:len(hours)]
        shuffled = [fit.intercept + fit.slope * hours[i] + resampled[i] for i in range(len(hours))]
        slopes.append(refit(shuffled).slope)
    slopes.sort()

    tail = (1.0 - confidence) / 2.0
    low_slope = percentile(slopes, tail)
    high_slope = percentile(slopes, 1.0 - tail)
    return Bootstrap(slope=fit.slope, sigma=statistics.pstdev(slopes),
                     half_width=(high_slope - low_slope) / 2.0, low=low_slope, high=high_slope,
                     confidence=confidence, block_hours=block_hours, replications=replications,
                     blocks=len(blocks), seed=seed)


def load_series(path):
    """Return (voltage_readings, temperature_readings) as sorted (timestamp, value) lists.

    Back-compatibility shim for battery_power_from_history.py, kept identical in behaviour to the
    version that used to live there. New code should use load_export() and series_for(): this one
    selects by `endswith`, which is unambiguous for these two suffixes but would not be for e.g.
    `_signal_strength`.
    """
    voltages = []
    temperatures = []
    with open_export(path) as handle:
        reader = csv.DictReader(handle)
        has_entity_column = reader.fieldnames is not None and "entity_id" in reader.fieldnames
        for row in reader:
            try:
                value = float(row["state"])
            except ValueError:
                continue  # e.g. "unavailable"
            timestamp = parse_timestamp(row["last_changed"])
            entity = row.get("entity_id", "") if has_entity_column else ""
            if not has_entity_column or entity.endswith(VOLTAGE_SUFFIX):
                voltages.append((timestamp, value))
            elif entity.endswith(TEMPERATURE_SUFFIX):
                temperatures.append((timestamp, value))
    voltages.sort(key=lambda r: r[0])
    temperatures.sort(key=lambda r: r[0])
    return voltages, temperatures
