# Plan: derive `power_model.py` inputs from Home Assistant history exports

**Status:** IMPLEMENTED 2026-09-09. See section 12 for what landed, what the implementation
changed about this design, and the four decisions that were open when it was written.
**Date:** 2026-09-09 (revised after review against the two fixture windows and the firmware;
re-verified 2026-09-09 by recomputing every number from the CSVs, `power_model.py` and the
firmware -- corrections are marked **[corrected 2026-09-09]** where a figure moved)
**Scope:** a new script that reads an HA history CSV and computes every argument
`tools/power_model.py` needs, so a real measurement window can be reconciled against the model
without hand-extracting numbers each time.

Every number in this document was recomputed from the two CSVs named in §6.

---

## 0. Prerequisite: fix the measured-current unit bug first

`battery_power_from_history.py` is the source of the "measured" column this whole exercise
reconciles against, and it is biased low by 6-7 %.

[`battery_power_from_history.py:166`](../tools/battery_power_from_history.py#L166) computes
`mean_ma = mean_mw / mean_v`, where `mean_mw` was built from `PACK_ENERGY_MWH = 3200 x 3.7`.
Expanded:

```
mean_ma = dpct/100 * 3200 * 3.7 / (mean_v * hours)      # what it does
mean_ma = dpct/100 * 3200      / hours                  # charge in, charge out
```

The stray factor is `3.7 / mean_v`. This is the identical error `power_model.runtime_days()`
documents having fixed on its own side ("Charge in, charge out -- no voltage belongs in this at
all"), still live on the measurement side.

| Window | mean_v | as printed today | charge-correct |
|---|---|---|---|
| ss=4, 2026-08-29 -> 09-07 | 3.931 V | 564.7 uA | **600.0 uA** |
| ss=16, 2026-08-20 -> 08-29 | 3.968 V | 1168.6 uA | **1253.4 uA** |

It does **not** cancel in a difference, because `mean_v` differs per window: the ss16-ss4 delta
that `LP_ACTIVE_CURRENT_UA` is back-solved from moves 604 -> 653 uA, +8 % on the 9 mA guess.
The error is 35 uA on the ss=4 window -- the size of `SLEEP_CURRENT_UA` itself.

Two consequences for this plan: the §6 fixtures are stated in charge-correct units, and
`runtime_days` in that script (`PACK_ENERGY_MWH / mean_mw / 24`) is *not* affected, because the
energy factor cancels there. Fix the current, leave the runtime.

The duplicated pack constants are the mechanism behind this bug; §5.4 resolves them.

---

## 1. The design question: cadence, or totals over the whole window?

**Decision: integrate over the window. Cadence becomes a derived reporting number, not an input.**

Five reasons, in order of how much they matter.

### 1.1 "Cadence" is a weather-dependent mixture parameter, not a device setting

This is the decisive one and it was missing from the first draft. Publishes fire on two
different triggers, so the gap distribution is bimodal, not unimodal:

| Window | safeguard-mode gap | share of cycles | remainder |
|---|---|---|---|
| ss=4 | 304.1 s (15 LP polls) | 56 % | threshold-triggered, 16-291 s |
| ss=16 | 325.5 s (15 LP polls) | 54 % | threshold-triggered, 16-312 s |

**[corrected 2026-09-09]** the remainder ranges were stated as "40-300" / "40-320"; the measured
minima are 16.1 s and 16.3 s, i.e. shorter than one LP poll period. That matters twice: it is the
floor a cycle-clustering epsilon has to stay under (§2.1), and it means a publish can follow its
predecessor faster than the LP core polls, so the low tail of the gap distribution is HP-side
jitter rather than an LP cadence.

The safeguard branch is `(maxSkip+1) x lp_poll_period`, with `maxSkip = gap_sec/poll_sec - 1`
([`main.cpp:127`](../main/main.cpp#L127)) -- so `max_publish_gap_sec: 300` over
`lp_poll_interval_sec: 20` gives exactly 15 polls, which is what both windows show. Everything
else is a dT/dRH threshold trip.

So the mean cadence is a *mixture weight* set by the weather. A window that drifts from 60 % to
40 % safeguard-mode moves "cadence" by ~10 % with no change in device behaviour whatsoever. Any
model keyed on a single cadence number silently absorbs the weather into the power estimate.
The integral does not.

**The extractor must therefore report the split** -- mode gap, mode share, and threshold-triggered
share -- not just the mean. Two windows with the same mean cadence and different mode shares are
not comparable.

### 1.2 It matches what is being measured

The battery voltage-decay measurement is `sum(charge) / T`. Building a "representative cycle" and
multiplying by a rate introduces a class of error that does not exist in the integral form. Match
the forms and that error disappears rather than being bounded.

### 1.3 The distributions are skewed, so "typical cycle" is ill-defined

From the ss=4 window:

| Metric | Mean | Median | Max |
|---|---|---|---|
| `radio_rx_time` | 364 ms | 242 ms | 39.5 s |
| `hp_awake_time` | 962 ms | 884 ms | 41.5 s |

Medians would understate energy badly. Only the integral is unambiguous.

### 1.4 Rows count successful publishes, not wakes -- and the firmware makes that safe

The first draft argued that outages consume without producing cycles. The real mechanism is more
specific, and it is the strongest argument for the totals form.

`hp_awake_stats_get_and_reset_us()` and `readLinkStats()` are both called **inside**
`if (bits & BIT_CONNECTED)` ([`mqtt_sender.cpp:1298-1303`](../main/mqtt_sender.cpp#L1298-L1303)).
A failed-connect cycle therefore resets neither accumulator, and the next successful cycle's row
covers both. That is where the 41.5 s `hp_awake_time` maximum comes from.

Two things follow:

* **Failed cycles are not lost.** Their awake time is carried into the next published row, so
  `sum(hp_awake)` really is the window's total HP-awake time and `sleep = T - sum(awake)` is
  sound. The totals form gets this for free; a per-cycle form would have to know the failure
  count to avoid double-counting.
* **`cadence = T/n_cycles` is a successful-publish period, not a wake period.** It is a valid
  `power_model` input only because the phase times are per-successful-publish too. That pairing
  is the whole content of §1.6's algebra -- state it there, not as folklore.

**Measured, not merely argued.** Window B contains a genuine 2 h 16 m outage
(2026-08-21T08:17:30 -> 10:34:04, 8194 s, 1.1 % of the window; **[corrected 2026-09-09]** -- the
draft called the same interval "2 h 11 m / 7834 s", but those two timestamps are 8194.5 s apart).
The recovery cycle reported
`hp_awake_time` 79.5 s, `radio_rx_time` 66.8 s, `radio_tx_time` 1.06 s, `tx_retries` 60 -- so
across 8194 s of no connectivity the HP core was awake for 79.5 s, about 1 %. That is direct
confirmation that `sleep = T - sum(awake)` holds through an outage: the device really does sleep
through them, and the accumulator really does carry the whole cost forward into one row. It also
sets the scale for the §4 outlier rule, since that single row is 2.9 % of window B's total
HP-awake time.

One more consequence: after a reboot the first `readLinkStats()` call reports no tx/rx at all
(baseline establishment, [`openthread_link.cpp:255`](../main/net/openthread_link.cpp#L255)), so
forward-fill would carry a pre-reboot value across the boundary. Safe only because the
`boot_count` guard rail in §4 refuses such a window outright.

### 1.5 HA de-duplication makes the observed samples a non-uniform sample of cycles

Home Assistant drops unchanged values, so the surviving samples over-represent *changing* cycles.
Measured on the ss=4 window, forward-filling onto a per-cycle anchor versus taking the mean of
observed rows:

| Metric | Coverage | mean(observed) | mean(forward-filled) | Bias |
|---|---|---|---|---|
| `radio_tx_time` | 96.3 % | 99.13 | 99.01 | +0.1 % |
| `radio_rx_time` | 100 % | 364.28 | 364.25 | +0.0 % |
| `hp_awake_time` | 100 % | 961.51 | 961.51 | +0.0 % |
| `mqtt_state_publish` | 96.0 % | 354.87 | 354.74 | +0.0 % |
| **`cca_failures`** | **59.2 %** | **3.88** | **2.54** | **+52.7 %** |

**[corrected 2026-09-09]** two cells above were computed under definitions this document does not
use elsewhere. `mqtt_state_publish` coverage was the raw-row rate (3385 rows / 3390 anchors =
99.9 %); §5.1 defines coverage as `sampled/len(anchors)`, which is 3254/3390 = 96.0 %.
`radio_tx_time` is 96.3 % under either definition, which is exactly why the inconsistency was
invisible in the row above it. And the filled `cca_failures` mean was 2.47 = the sum over all 3390
anchors, i.e. counting the 92 never-covered cycles as zero; `column_mean()` as §5.1 defines it
(mean over non-`None` entries) gives 2.54. Both readings show the same severe bias, but the
extractor must pick one and print which.

Negligible for the timing series, because they change every cycle and almost nothing is deduped.
**Severe for counters**, because repeated zeros are dropped and the survivors over-represent the
nonzero cycles. A metric's class cannot be known in advance, so **forward-fill onto a cycle anchor
always**.

**Forward-fill here is exact reconstruction, not interpolation** -- a stronger claim than the
draft made, and worth stating because it settles the "is filling legitimate?" question outright.
HA drops a row *only* when the value is unchanged, so the filled value is what the device
published. Verified directly: `radio_tx_time` has 428 distinct values across 3264 numeric
rows and **zero** surviving consecutive-equal pairs -- HA removed 126 exact repeats, and filling
puts back precisely those 126 (**[corrected 2026-09-09]**: 3390 - 3264 = 126, which is what §5.1
and §5.7 already say; "429 / 125" was off by one in both figures). Any API for this must therefore never silently interpolate; a
`fill_forward` that could also linearly interpolate would turn an exact operation into a guess.

### 1.6 The useful algebra

**Since §9.2 landed this is a note about the CLI, not load-bearing algebra.** The model's
primitive is now window totals; `--cadence` / `--phase-times` is a front end that multiplies up,
and the translation happens once, in `_build_parser`'s dispatch. Kept below because the trap it
describes is real and the implementation surfaced it: `ha_log_metrics.py --run` prints both forms
and they differ by 0.092 uA on window A, purely because cadence divides `T` by the `n-1` intervals
between anchors while the phase means divide the totals by the `n` cycles.

The two forms are *identical* provided:

```
cadence      := T / n_cycles
phase_times  := totals / n_cycles
```

Both definitions must use the *same* `n_cycles` -- the successful-publish count of §1.4 -- or the
per-cycle form silently mixes two different denominators. That is the only subtlety, and it is
why the extractor computes **totals as the source of truth** and emits the equivalent
cadence/phase-times pair as a derived convenience for the existing API.

The extractor therefore computes totals as the source of truth and emits the equivalent
cadence/phase-times pair for the current API. **§9.2 proposes retiring that translation entirely**
by making totals the model's primitive, at which point this equivalence stops being load-bearing
and becomes a note about the CLI convenience wrapper. Until then it is what keeps the two forms
reconcilable -- and both definitions must use the same `n_cycles`.

---

## 2. Per-quantity extraction

| Quantity | Source | Method | Pitfall |
|---|---|---|---|
| Cycle anchor | union of all per-cycle entities | cluster timestamps within 10 s -> `n_cycles`, `cadence = T/(n-1)` | **no single entity is safe** -- see below |
| HP awake | `hp_awake_time` | forward-fill, sum | already sleep-compensated by `hp_awake_stats.cpp`; do not subtract sleep again |
| TX time | `radio_tx_time` | forward-fill, sum | per-cycle delta of `otRadioTimeStats.mTxTime`; the OT counter is uint64, the *published delta* is truncated to uint32 |
| RX time | `radio_rx_time` | forward-fill, sum | same |
| CPU residual | `awake - tx - rx` | **per cycle, then sum** | can go negative on misaligned cycles -- clamp and report the count (0 occurrences in both fixture windows) |
| LP active | *not logged* | `n_polls x lp_active_time(N)` where `n_polls = T / lp_poll_period` | see below |
| LP poll period | `heater_run_count`, cross-checked against the safeguard mode | two independent estimators -- see below | |
| Heater busy-wait | publish gap ending at each increment | **now derived, no longer an estimate** -- see below | |
| `sensor_samples` | `number.*_sensor_samples` | must be constant across the window | |
| TX power | `tx_power_active` | must be constant across the window | otherwise the window is confounded |
| Measured current | `voltage` -> `battery` % | OLS slope, reuse `battery_power_from_history` | **apply §0 first** |
| Temperature multiplier | `temperature` | `power_model.arrhenius_multiplier()` | dwell-weighted, not sample-weighted; already implemented correctly |

### 2.1 The cycle anchor: no single entity

The first draft named `mqtt_client_start` as "the only metric safe to use as a cycle clock,
fires unconditionally every cycle". That is wrong. It is a duration in milliseconds and HA
dedupes it exactly like every other value. In the ss=4 window it has 3388 numeric rows where
`hp_awake_time` has 3390 -- it is not even the densest series. The resulting cadence is 234.48 s
from that anchor versus 234.34 s from the union.

Only 0.06 % here, but the rule is unsound and will bite on a flatter window. **Anchor on
timestamp clusters over the union of the per-cycle MEASUREMENT entities.** They ride one MQTT
message, so they land close together -- measured, the whole cluster spans at most 16 ms, not the
"about a second" the draft assumed -- and per-metric coverage falls out of the same pass for free.

**[corrected 2026-09-09] "all per-cycle entities" is the wrong union, and §6 was not computed with
it.** Recomputed on window A:

| anchor set | n_cycles | cadence |
|---|---|---|
| all 26 entities in the export | 3395 | 233.996 s |
| the 11 entities this pipeline reads (§6.1) | 3395 | 233.996 s |
| `hp_awake_time` alone | 3390 | 234.341 s |
| the 8 per-cycle **measurement** entities | **3390** | **234.341 s** |

Only the last reproduces §6's `3390 / 234.34`, which §6 marks *exact*. The five phantom anchors
come from entities whose value never changes: `boot_count`, `tx_power_active`, `sensor_samples`,
`parent_link_quality` and `tx_no_ack_expiry` each carry a row stamped exactly on an export
boundary (`2026-08-29T05:00:00.000Z`, and a second at `06:00:00.000Z`) that HA emits as a
window-edge state snapshot, not as a publish. So the anchor union must be

```python
PER_CYCLE_ENTITIES = ("hp_awake_time", "radio_tx_time", "radio_rx_time", "heater_run_count",
                      "voltage", "battery", "temperature", "cca_failures")
```

-- the series that carry a fresh value most cycles -- with the guard-rail constants
(`boot_count`, `tx_power_active`, `sensor_samples`) read but deliberately **excluded from the
anchor pass**. Two other places in this document silently use the wrong union and must move with
it: §5.2's `as_of`
comparison (its 354.71 ms is an all-26 number; on the correct anchor set the same `as_of` gives
371.0 ms) and §6.1's claim that an 11-entity fixture reproduces §6 bit-for-bit (it does not --
it reproduces 3395 anchors unless the three constants are excluded from the anchor pass).

**[corrected during implementation]** an earlier revision of this section also proposed discarding
"any row landing exactly on the export's first or last second". That would be wrong, and the
fixtures catch it: on window A the export-start snapshot at `05:00:00.000Z` carries a row for ALL
EIGHT measurement entities, so discarding it drops a real anchor and gives 3389 cycles against
§6's 3390. The snapshot's TIMESTAMP is synthetic but its VALUE is a real last-known publish. The
implementation therefore counts boundary-stamped rows per series (`Series.boundary_stamps`) and
reports them, and never filters on them; choosing the anchor set is what solves this.

**On the epsilon.** With a 16 ms cluster spread and a 16.1 s minimum inter-publish gap (§1.1),
anything from ~0.1 s to ~15 s works on these two windows, so 10 s is safe *here* -- but not
"at any cadence this device runs": `lp_poll_interval_sec` is runtime-tunable, and at a poll
period under ~10 s with `max_publish_gap_sec` at one poll the epsilon would merge real cycles.
Make it a parameter (it already is, §5.7) and have the extractor refuse when the epsilon exceeds
half the smallest observed gap.

**[changed 2026-09-24]** default 10 s -> **5 s**. The 2026-09-23 export has a 14.3 s minimum gap
(a failed publish retried one 20 s poll later, one of the pair landing late), which the refusal
rule -- implemented as `smallest gap < 1.5 x epsilon` -- turned into a Python traceback. 5 s is
still ~300x the cluster spread. A refusal is now `ha_history.AnchorError`, and the extractor
reports it as a refused window (exit 2) rather than raising.

### 2.2 LP poll period: two estimators that check each other

**Primary -- `heater_run_count`.** It advances once per exactly 4320 LP cycles
(`minutes_to_lp_cycles(1440, 20)`; the LP core increments `cycles_since_heater` every poll and
resets it to 0 in the same poll that runs the heater, so the interval really is 4320 polls), so
the period is

```
lp_poll_period = (t[last increment] - t[first increment]) / (4320 * (n_increments - 1))
```

Use the first-to-last span only. The draft's "measures the real period to within 0.1 %" conflated
*measurement precision* with *agreement with the model*: individual heater intervals in the ss=4
window span 20.2402-20.4141 s, a spread of +/-0.4 %, because each timestamp is quantized to the
next successful publish. Only the endpoint span averages that down. The uncertainty is

```
+/- cadence / (4320 * (n_increments - 1))
```

which is +/-0.033 % at 8 intervals and +/-0.051 % at 5. So replace the draft's binary "fall back
if no heater run in window" with a **minimum of 3 increments**, below which the estimator is
reported with its widened error bar rather than used silently.

**[corrected 2026-09-09] this estimator is not unbiased, and the bias is bigger than the error bar
just quoted.** Exactly one of every 4320 polls *is* the heater poll, so the span between two
increments is `4320 x P + stall`, not `4320 x P` -- and §2.3 below measures that stall at +96 s.
That is `96 / 4320` = **+22.3 ms, +0.11 %**, three times the +/-0.033 % above and in the same
direction in both windows. As written §2.2 and §2.3 contradict each other: §2.2 tells the
*safeguard* pool to drop the stalled gaps, then leaves the same stall inside the *primary*
estimator. Subtracting it gives

| Window | raw span/4320 | stall-corrected | model floor (`lp_poll_interval + lp_active_time`) |
|---|---|---|---|
| ss=4 | 20.3442 s | **20.3219 s** | 20.3440 s |
| ss=16 | 21.7868 s | **21.7644 s** | 21.6760 s |

Worth stating rather than quietly applying, because the corrected ss=4 figure lands 22 ms *below*
the model's own sleep+work floor -- which is not absurd: it implies the LP timer's RTC_SLOW source
runs about 0.11 % fast, entirely ordinary for an RC-derived clock, and it shrinks the ss=16
anomaly from +110.8 ms to +88.4 ms. Whichever way it is resolved, the printed error bar must
cover the stall term instead of ignoring it, and the "20.3442 vs 20.344 predicted, 0.1 %"
agreement recorded in `lp_poll_period_s()`'s docstring is partly this bias cancelling a real
clock offset, not a clean confirmation.

**A second, unguarded failure mode.** `heater_high_rh_trigger_minutes: 60` (180 polls) fires the
heater off-schedule whenever calibrated RH stays above `kHighRhThreshold` (90 %) that long, and it
resets `cycles_since_heater` too -- so one interval becomes short and the endpoint-span estimator
is silently wrong by whatever fraction that is. Neither fixture window trips it (max RH 80.3 %
and 72.0 %), so this path has **no test coverage at all**, on an outdoor sensor where >90 %RH is
a normal night. §4 gains a per-interval consistency check for it.

**Cross-check and fallback -- the safeguard-mode gap.** Roughly 1900 samples instead of 8
intervals. Take the densest 5 s bin of inter-publish gaps, pool the gaps within +/-10 s of it,
and divide the **p90** of that pool by `k = round(mode / lp_poll_period)`:

| Window | heater estimator | safeguard p90 / k | disagreement |
|---|---|---|---|
| ss=4 | 20.3442 s | 20.3559 s (k=15) | +0.057 % |
| ss=16 | 21.7868 s | 21.7928 s (k=15) | +0.028 % |

p90 specifically, and this is empirical: the median of the pool reads 0.36-0.40 % **low** in both
windows because threshold-triggered publishes are always *early* and contaminate the low side,
while p95 and above read 0.6-1.8 % high because outage-stretched gaps contaminate the top. p90
lands within 0.06 % of the unbiased heater estimator in both windows independently.

Exclude the gaps ending at a heater increment from the pool -- they carry the §2.3 stall.

`k` is an integer and both estimators must agree on it. If `round(mode / lp_poll_period)` is not
within 0.1 of an integer, something is wrong with the window and the extractor should say so
rather than pick a number.

The script then emits

```
--lp-poll-interval (measured_period - lp_active_time(N))
```

so the model's internal `sleep + work` reconstruction lands on the measured period rather than
the nominal one. On the fixtures that yields 20.0002 s (ss=4, i.e. exactly the configured 20 s)
and 20.1108 s (ss=16) -- reproducing the known +110.8 ms ss=16 anomaly already documented in
`lp_poll_period_s()`'s docstring, from a completely independent code path.

**This subtraction is a workaround, not a design.** It exists only because the model's flag is the
sleep duration rather than the period, so a measured quantity has to round-trip through a modelled
one. §9.1 removes it by having the model take `--lp-poll-period` directly; when that lands, delete
this paragraph and emit the measured 20.3442 s / 21.7868 s unchanged.

### 2.3 Heater busy-wait: derived, not estimated

The first draft listed this under "known-imperfect" as permanently unmeasurable. It is not. The
LP core's post-heater busy-wait stalls the next publish by its own duration, so the gap **ending
at** the increment's publish measures it directly:

| Window | stalled gaps | safeguard mode | busy-wait |
|---|---|---|---|
| ss=4 | 401, 401, 401, 400, 400 s | 304.1 s | **+96 s** (n=5 of 9) |
| ss=16 | 421, 423, 423, 428 s | 325.5 s | **+97 s** (n=4 of 6, see below) |

Two independent windows at different LP periods agree to 1 s, and the answer decomposes cleanly:
`kRhSettleMs` = 90 s ([`lp_core/main.cpp:92`](../components/lp_sensor_core/lp_core/main.cpp#L92))
plus about three `kCooldownPollMs` iterations. The `~110 s` figure in `device_total_ua()`'s
KNOWN OMISSION note is therefore ~13 % high, and the omission can be closed.

Two implementation notes, both of which cost a debugging session if missed:

* Use the gap **ending at** the increment, not the one starting at it. The count is published in
  the same message as the sensor values, so the increment's timestamp *is* an anchor;
  `bisect_right(anchors, ts) - 1` silently returns the wrong side and yields 303 s instead of
  401 s. This was the first draft reviewer's own mistake -- it is easy to make.
* Only 5 of 9 and 4 of 6 increments are usable. The heater perturbs T and RH, so the post-heater
  reading often trips a dT/dRH threshold and publishes early, splitting the stall across two
  gaps. Keep only increments whose ending gap exceeds the safeguard mode by a clear margin;
  median those. **[corrected 2026-09-09]** the draft's rule was "exceeds the safeguard mode",
  which does not reproduce its own "4 of 6": window B's six ending gaps are 421.1 / 357.0 /
  423.2 / 422.5 / 229.8 / 428.4 s and 357.0 s does exceed the 325.5 s mode, so that rule keeps
  five. Use `mode + 50 s` (window A: 158.4 / 137.5 / 401.4 / 400.6 / 177.8 / 400.9 / 217.9 /
  399.9 / 400.0 -> the same 5 either way; window B -> 4). The median is +96/+97 s under both
  cuts, so this changes the rule's honesty, not the answer.

### 2.4 `unavailable` and `unknown`

Absent from the first draft, and present in **every entity of both fixture windows** -- 3 to 4
occurrences each for the per-cycle sensors (more for the text/binary entities, whose every row is
non-numeric).

**They are HA restarts, not `expire_after` firing, and the distinction was checked rather than
assumed.** The firmware sets `expire_after = 2 x safeguardWakeSec = 2 x (14+3) x 20 = 680 s`
([`main.cpp:346`](../main/main.cpp#L346)). If expiry were the cause, the silence preceding each
flag would be a constant ~680 s. Measured, the silences are 224 / 263 / 505 / 558 s (window A) and 376 / 421 / 7251 s (window B)
-- and 224 s is well under any plausible expiry. (**[corrected 2026-09-09]**: the draft listed
"224 / 377 / 505 / 558 / 7251" as if complete; it omits window A's 263 s and window B's 421 s
flags. The argument is unaffected -- two more inconsistent values make it stronger.) Inconsistent silence with every entity flipping within 0.4 s of
the others (measured spread 0.32-0.37 s, not the "0.3 s" the draft rounded to) is the signature of
HA restarting, which marks entities unavailable immediately regardless of how recently they
published.

Three things break without explicit handling:

* **Guard rails read as value comparisons.** `boot_count` reads `[272, "unavailable", 272]`. A
  naive float parse raises; a naive "is it flat?" reports a reboot that never happened.
* **`heater_run_count` goes `9 -> unavailable -> 9`.** A "count the transitions" implementation
  double-counts an increment and corrupts §2.2 outright. Count only `v[i] == v[i-1] + 1` over the
  numeric subsequence.
* **Forward-filling across the gap.** Harmless for an HA restart, wrong for a real device outage
  -- and the two are **not** distinguishable from the CSV. Window A's blackouts are 41.9 / 48.6 /
  77.8 / 79.4 s, sitting inside anchor gaps of 304.8 / 606.9 / 582.9 / 303.8 s respectively.
  **[corrected 2026-09-09]** the draft listed three blackouts and said "each sits inside a ~600 s
  anchor gap"; there are four, and only two of them do -- the other two are contained entirely
  within an ordinary safeguard-length gap. That makes the point *sharper*, not weaker: a blackout
  need not disturb the cycle rhythm at all, so gap length is not a usable discriminator either. So
  the extractor must not try to classify them: it takes a dwell cap as a *parameter*, holds within
  it, yields `None` past it, and *reports* blackout count and duration alongside every anchor gap
  over the safeguard mode.

**On the dwell cap.** `arrhenius_multiplier()` already hardcodes 7200 s for the sibling problem,
and reusing it keeps one number in the codebase. But it is a *chosen* threshold, not a derived
one -- the tempting derivation ("7251 s of silence preceded a flag, so 7200 s is `expire_after`")
is a coincidence, and the paragraph above is why. The honest rationale is that the safeguard mode
is 304-326 s, so anything beyond ~20x that is certainly a gap and not a de-duplicated repeat.
Keep it a named parameter so a future window can move it without touching the logic.

**An unrelated observation this turned up, worth its own investigation.** During the 2 h 11 m
outage in window B (§1.4), HA did *not* mark the entities unavailable at 680 s as configured --
the flag appears 7251 s in, alongside all the others. Either the published `expire_after` is not
taking effect in HA, or it is not the value the firmware computes. That is a discovery-config
question, not a blocker for this plan, but it means `expire_after` cannot currently be relied on
to mark real outages.
---

## 3. Uncertainty budget

New section. The draft used the bootstrap figure only as a window-length gate; it belongs in
every number the script prints.

The 24 h residual block bootstrap gives **a 95 % interval of +/-0.077 mA at 9 days** (that figure
comes from the 2026-08-29->09-07 power-budget report, which states the confidence level; the draft
quoted it bare, and §5.3's `BOOTSTRAP_UA = 77.0` sitting next to a `+/-` reads as 1 sigma, which it
is not -- label it). Against a 600 uA measurement that is **+/-13 %** -- larger than the §0 unit
bug (-5.9 %), larger than the temperature correction (-10.8 % on the ss=4 window), and roughly
twice `SLEEP_CURRENT_UA`.

**[corrected 2026-09-09] the number is method-sensitive and the extractor must pin the method.**
Re-running a 24 h residual block bootstrap over 2000 replications on the same window reproduces
the point estimate exactly (600.2 uA fitting HA's `battery` % series directly, with no voltage
anywhere -- an independent confirmation of §0) but gives 1 sigma = 26-29 uA and a 95 % half-width
of ~56 uA, against the report's 77 uA; the gap is block-boundary construction. At 56 uA the
ordering above still holds against the §0 bug and `SLEEP_CURRENT_UA` but **not** against the
temperature correction (65 uA), so §3's rhetorical ranking is inside its own reproduction spread.
Fix by fixing the recipe: state block length, replication count, and confidence level in
`ha_history.py`, and print the interval rather than a bare `+/-`.

So: the measurement side carries +/-13 %, and the model side carries a `LP_ACTIVE_CURRENT_UA`
that is a guess spanning 1.4-14.7 mA. Quoting either to four significant figures invites a future
reader to chase differences that are noise. **Every measured quantity the extractor prints must
carry its error bar, and §6's fixtures carry tolerances.** The derived phase times are the
precise part of this pipeline (they are sums of logged values, good to ~0.1 %); the reconciliation
against a decay measurement is the imprecise part. The output must not present them in the same
typeface.

---

## 4. Guard rails

Every one of these has already invalidated a window in this project. **Fatal by default, with a
`--force` escape hatch** -- see §8 Q3.

- `boot_count` not flat -> reboot or OTA inside the window, measurement invalid
- `tx_power_active` not constant -> mixed operating points
- `sensor_samples` not constant -> mixed operating points
- `unavailable`/`unknown` runs longer than the dwell cap -> real outage, not an HA restart; report
  the count and total duration either way
- per-metric coverage below threshold -> report it; never silently average a sparse series
- fewer than 3 `heater_run_count` increments -> LP period falls back to the safeguard estimator
  with a widened error bar (§2.2), not to the nominal
  **[2026-09-24] the fallback was never implemented.** With 2 increments the heater estimator
  runs on its one interval (its error bar widens by itself); with fewer, the new fatal rail
  "LP period measured" refuses the window, and `--force` runs the model on the NOMINAL period.
  Before that, such a window crashed on `findings.lp` being None.
- **[added 2026-09-09]** any single `heater_run_count` interval more than 1 % off the median
  interval -> a humidity-triggered heater run reset `cycles_since_heater`, so the endpoint-span
  estimator of §2.2 is measuring fewer than `4320 x (n-1)` polls. Fatal: it corrupts the primary
  LP-period estimator silently. (Window A's eight intervals span 20.2402-20.4141 s, +/-0.43 %, so
  a 1 % cut passes both fixtures with margin.)
- the two LP estimators disagree by more than 0.5 %, or `k` is not within 0.1 of an integer
- window shorter than ~5 days -> the bootstrap gives +/-0.077 mA at 9 days; shorter windows are
  not worth quoting
- counter wraparound on `radio_tx_time` / `radio_rx_time`. Note the risk is *not* where the draft
  put it: the OT counters are uint64 us, and it is the **published per-cycle delta** that is
  truncated to uint32 ([`openthread_link.cpp:260`](../main/net/openthread_link.cpp#L260)). A cycle
  would need 71.6 minutes of pure radio time to wrap, which only a multi-hour outage could
  approach -- and it cannot be detected from the delta alone, only inferred from the gap length.
- outliers integrated **separately** and reported as their own uA contribution, so they are
  visible rather than buried. ss=4 window: 15 cycles over 5 s totalling 297.8 s, which is 9.1 %
  of all HP-awake time and about 12 uA rail / 11 uA battery at the mid CPU current.

---

## 5. Files

Standard library only, per the project's standing no-dependency rule. `ota_push.py` imports
`paho.mqtt`, so that rule is scoped to the analysis tools, not the repo.

### 5.1 `tools/ha_history.py` (new, shared)

CSV -> per-entity series; cycle anchoring; forward-fill; as-of join; dwell weighting; the §2.4
blackout policy. Both `battery_power_from_history.py` and the new extractor need this, so it
**removes the duplicate rather than adding a third copy**.

```python
NOT_A_NUMBER    = ("unavailable", "unknown")   # HA's two non-value states
MAX_HOLD_S      = 7200.0                       # dwell cap; see 2.4 -- chosen, not derived
CYCLE_EPSILON_S = 10.0                         # anchor cluster width; see 2.1 (5.0 since 2026-09-24)

Blackout   = namedtuple("Blackout", "start end seconds marker")
Series     = namedtuple("Series", "name entity_id samples blackouts")
Export     = namedtuple("Export", "path device_prefix series")
Column     = namedtuple("Column", "name values sampled held missing")
CycleTable = namedtuple("CycleTable", "anchors columns epsilon_s max_hold_s")
```

A **series** is `Series.samples`: a chronological list of plain `(datetime, float)` 2-tuples,
numeric rows only. Deliberately a plain tuple rather than a record type, because that is exactly
what `power_model.arrhenius_multiplier()` already unpacks -- so
`arrhenius_multiplier(export.series["temperature"].samples)` works with no adapter.

A **cycle-anchored table** is `CycleTable`: `anchors` is a list of datetimes, `columns` maps short
name -> `Column`, and each `Column.values` is the same length as `anchors` with entries `float` or
`None`. Column-major, because every consumer sums or means a whole column and the CPU residual is
a zip of three of them.

`Column.sampled / held / missing` split a word §1.5 overloads. `sampled/len(anchors)` is
"coverage" -- 96.3 % on `radio_tx_time` is a *de-duplication rate*, entirely normal. `missing` is
genuinely absent data, and is 0 on every timing series in both windows. The invariant
`sampled + held + missing == len(anchors)` makes every filled cell auditable: `held` is exactly
the count of HA-dropped repeats put back, which on window A's `radio_tx_time` is 125 -- §1.5's
proof, now a runtime assertion instead of a note.

| Function | Purpose |
|---|---|
| `parse_timestamp(text)` | ISO-with-`Z` -> aware datetime |
| `load_export(path, device_prefix=None)` | CSV or `.csv.gz` -> `Export`; sorts, splits numeric from blackouts |
| `device_prefix_of(entity_ids)` | common prefix of domain-stripped ids, trimmed to the last `_` |
| `series_for(export, name, required=True)` | one `Series` by exact short name |
| `short_names(export)` | sorted short names, for `--list-entities` |
| `cycle_anchors(export, names, epsilon_s=...)` | union of timestamps -> cluster starts (§2.1) |
| `cycle_table(export, anchors, names, epsilon_s=..., max_hold_s=...)` | **cluster-scoped** assign-then-hold -> `CycleTable` |
| `as_of(series, stamp, max_hold_s=..., stamps=None)` | most-recent-at-or-before, `None` past the cap |
| `as_of_join(stamps, series, max_hold_s=...)` | vectorised `as_of`, for **independent** series |
| `dwell_weighted_mean(samples, max_dwell_s=...)` | the `arrhenius_multiplier` weighting, minus the Arrhenius |
| `anchor_gaps(anchors, min_seconds=None)` | `[(start, end, seconds)]` |
| `gap_histogram(anchors, bin_s=5.0)` | feeds the §2.2 safeguard-mode detector |
| `constant_value(series, allow_blackouts=True)` | `(value, complaint_or_None)` -- the guard-rail primitive |
| `monotone_increments(series)` | rows where `v[i] == v[i-1] + 1` over the numeric subsequence (§2.4) |
| `column_total(column)` / `column_mean(column)` | sum / mean over non-`None` entries |
| `blackouts_over(series, seconds)` | blackout runs longer than a threshold |
| `load_series(path)` | back-compat shim: `(voltages, temperatures)` exactly as today |

### 5.2 Two fill primitives, not one flagged function

This is the load-bearing API decision and it is measured, not stylistic. All ~18 per-cycle
entities ride one MQTT publish, so they land within about a second of each other but not in a
fixed order. As-of joining them onto the cluster's first timestamp shifts any entity that
published *after* the anchor back by a whole cycle:

| Method | RX mean | negative CPU residuals |
|---|---|---|
| `as_of` onto union anchors | 371.0 ms | **12** |
| `cycle_table` (cluster-scoped) | **364.25 ms** | **0** |

**[corrected 2026-09-09]** the draft's 354.71 ms is an all-26-entity-anchor number (recomputed:
354.55 ms, 12 negatives) and so is not comparable with the 364.25 ms beside it, which is a
§6-anchor number; on the §6 anchor set (§2.1) the same `as_of` gives 371.0 ms. The twelve
fabricated negatives are identical under both anchor sets, so the conclusion is untouched -- only
the size and sign of the RX error move, from -2.6 % to +1.9 %.

Only the second reproduces §6. A ~2 % error in RX and twelve fabricated negative residuals is
too easy to introduce from a single wrong keyword argument, so the two operations get two names
and two docstrings that point at each other:

* **`cycle_table(...)`** for series **co-published with the anchor**. Assigns the last sample in
  `[anchor, anchor + epsilon_s)`; only if the entity published nothing in that cluster does it
  hold the previous value forward, and only within `max_hold_s`.
* **`as_of(...)` / `as_of_join(...)`** for series **independent of the anchor** -- temperature
  against voltage in `battery_power_from_history`. Its docstring must say in as many words: not
  for per-cycle metrics, use `cycle_table`, and here is what it costs.

There is deliberately **no interpolation code in the module and no `method=` parameter** for
anyone to add one to. §1.5 establishes that filling is exact reconstruction; the way to keep it
that way is to make the alternative unimplemented rather than discouraged.

### 5.3 `tools/ha_log_metrics.py` (new)

```python
PER_CYCLE_ENTITIES        = (...)    # section 2.1's eight measurement entities; anchor union
GUARD_ENTITIES            = (...)    # boot_count / tx_power_active / sensor_samples: read, but
                                     # NEVER anchored on -- they carry export-boundary rows
HEATER_TICKS_PER_STEP     = 4320     # minutes_to_lp_cycles(1440, 20), main/main.cpp
MIN_HEATER_INCREMENTS     = 3        # 2.2
LP_ESTIMATOR_DISAGREEMENT = 0.005    # 0.5 %
OUTLIER_THRESHOLD_MS      = 5000.0
MIN_WINDOW_DAYS           = 5.0
BOOTSTRAP_UA              = 77.0     # 24 h residual block bootstrap, 95 % interval, 9 d; sec 3
HEATER_INTERVAL_TOLERANCE = 0.01     # 1 %; catches an off-schedule high-RH heater run, sec 2.2

Window      = namedtuple("Window", "path start end seconds n_cycles cadence_s table")
PhaseTotals = namedtuple("PhaseTotals", "tx_s rx_s cpu_s awake_s negatives outlier_count outlier_s")
LpPeriod    = namedtuple("LpPeriod", "seconds source uncertainty k cross_check disagreement")
Measured    = namedtuple("Measured", "ua ua_corrected mean_v pct_delta hours bootstrap_ua")
Guard       = namedtuple("Guard", "name ok detail")
Findings    = namedtuple("Findings", "window phases lp measured multiplier heater_stall_s guards")
```

| Function | Purpose |
|---|---|
| `build_window(export, epsilon_s=..., max_hold_s=...)` | anchors + `CycleTable` + `cadence = T/(n-1)` |
| `phase_totals(window, outlier_ms=...)` | per-cycle `awake-tx-rx`, clamp, sum, count negatives and outliers |
| `safeguard_mode(window, lp_period_s=None)` | `(mode_s, share, pool)` -- densest 5 s bin, pooled +/-10 s |
| `lp_period_from_heater(export, window, ticks=...)` | primary estimator + its error bar |
| `lp_period_from_safeguard(window, mode_s, nominal_period_s)` | p90 of the pool / integer `k` |
| `resolve_lp_period(export, window, sensor_samples)` | picks primary, cross-checks -> `LpPeriod` |
| `lp_poll_interval(lp_period_s, sensor_samples)` | `period - power_model.lp_active_time_s(N)` |
| `heater_stall_seconds(export, window, mode_s)` | median of gaps **ending at** each increment, minus mode |
| `measured_current(export)` | delegates to `battery_power_from_history` |
| `check_guards(export, window, phases, lp)` | `[Guard]`, in §4 order |
| `derive(export, ...)` | one `Findings` -- the whole extraction, no printing |
| `model_command_line(findings)` | the `power_model.py ...` string |
| `report(findings, run_model=True, stream=sys.stdout)` | the printer |
| `_build_parser()` / `main(argv=None)` | CLI |

Namedtuple returns throughout, so callers write `phases.rx_s` and nobody needs the
`a, b, c = f()` unpack the project's one-declaration-per-line rule forbids.

### 5.4 `tools/battery_power_from_history.py` (refactor -- a prerequisite, not a side effect)

The file has ~25 top-level statements and **no `if __name__ == "__main__"` guard**, with
`CSV_PATH = "voltage_history.csv"` at module scope, so `import battery_power_from_history` runs
the whole analysis -- printing eight lines of unrelated output when the cwd happens to be the repo
root (verified), and raising `FileNotFoundError` from anywhere else. The silent-success case is
the worse of the two. §2's "reuse `battery_power_from_history`" is
blocked until this is split.

* **Stays:** `etalon_soc`, `voltage_to_percent`, `ordinary_least_squares`,
  `temperature_corrected_fit`, `BATTERY_CURVE`.
* **Moves out:** `load_series` -> `ha_history` (re-exported here, so any existing import keeps
  working); `CSV_PATH` -> an argparse `--csv` default.
* **New:** `decay_fit(readings)`, `join_temperatures(...)`, `print_decay(fit)`,
  `print_temperature_correction(...)`, `_build_parser()`, `main(argv=None)`, and the guard.
* **Back-compat:** with no arguments it still reads `voltage_history.csv` from the cwd and prints
  the same eight lines. Nothing in the repo imports either module, so the CLI text is the entire
  compatibility surface.

**Pack constants land in `power_model.py`**, not a new module -- it already carries the
best-documented copies with the capacity-parameter essay attached, and a fourth file is churn.
The import forces a resolution that is worth stating because it *is* the §0 mechanism:
`battery_power_from_history.PACK_MEAN_VOLTAGE = 3.7` is a misnamed duplicate of
`power_model.PACK_NOMINAL_VOLTAGE`, **not** of `power_model.PACK_MEAN_VOLTAGE = 3.931`. Bind the
nominal, or the byte-identity test in §5.5 fails on the first run -- which is the point of having
it.

### 5.5 Sequencing: two commits, and why the order matters

Byte-identity and §0 are in direct conflict: §0 *deliberately* changes the printed number
(`voltage_history.csv`: 0.437 -> 0.483 mA; window A: 0.565 -> 0.600 mA). So:

1. **Pure extraction.** Functions, main guard, `--csv`, shared `load_series`, shared pack
   constants. Gate: CLI stdout is byte-identical. This is the only mechanically checkable
   correctness property the refactor has -- do not spend it.
2. **The unit fix.** One line, plus a `CAUTION` block in the module docstring in exactly the
   register `power_model.py` already uses for its 2026-09-09 constant changes, listing the
   old -> new pairs for every window quoted in `project_power_estimates`. The regression
   expectation is re-blessed in the same commit, so the diff shows the number moving next to the
   reason.

Mixing them makes the extraction unreviewable, because every changed digit could be either the
intended fix or a refactor slip.

Five details in the current code will silently move the last printed digit if "cleaned up", and
all must be preserved through commit 1: `total_hours = hours[-1]` (not last-minus-first);
`pct_start` is the regression *intercept*, not `percents[0]`; `mean_v` is over all readings while
the corrected fit runs on the joined subset and still divides by that same `mean_v`;
`runtime_days` keeps the *energy* form because that is where the 3.7 cancels (§0); and the
`len(matched_hours) > 100` gate is `>`, not `>=`.

**Also in the §0 commit:** `power_model.py`'s docstring and its `--help` epilog examples quote the
pre-§0 565 / 1169 uA, and `MEASURED_PHASE_TIMES_S`'s comment quotes the unfilled phase times
`(0.09913, 0.36428, 0.49810)`. The filled equivalents are `(0.09901, 0.36425, 0.49826)`. That
comment also says "means over 3391 cycles"; §6 establishes 3390. Update all four spots, or the
model's own examples contradict the extractor's output on day one.

### 5.6 Entity naming: exact keys, not `endswith`

`device_prefix_of()` takes the common prefix of the domain-stripped ids trimmed back to the last
`_`, which yields `esp32_ot_mqtt_outdoor_` on both windows (26 entities each, spanning `sensor.`,
`number.` and `binary_sensor.`), and returns `""` when fewer than two entities make it meaningful.
Series are then keyed by **exact** short name.

This is not cosmetic, though it is a latent hazard rather than a live bug: the two suffixes the
current code actually matches (`_voltage`, `_temperature`) are unambiguous in every export here.
One step further and they are not -- `endswith("_signal_strength")` matches both `signal_strength`
and `uplink_signal_strength`, and `endswith("_mqtt_state_publish")` matches
`mqtt_state_publish_raw`; all four are present in every metrics export. Exact keying removes the
failure mode before it is reached instead of documenting it.
`series_for(..., required=True)` raises with the available names listed; `--device PREFIX`
overrides the inference.

A single-entity export has no `entity_id` column; its one series is stored under the empty name
and `series_for()` resolves any requested name to it, since such an export cannot be ambiguous.
All 18 exports currently in the repo carry `entity_id`, so this path has **no real-data
coverage** and needs an explicit synthetic fixture.
### 5.7 CLI and output

```
ha_log_metrics.py --csv metrics_history_2026-08-29__to__2026-09-07.csv --run
```

| Flag | Default | Purpose |
|---|---|---|
| `--csv PATH` | *required* | HA history export (`.csv` or `.csv.gz`) |
| `--device PREFIX` | inferred | override the entity-prefix inference (§5.6) |
| `--sensor-samples N` | from `number.*_sensor_samples` | override when the entity is absent |
| `--tx-power DBM` | from `tx_power_active` | ditto |
| `--capacity MAH` | `power_model.PACK_CAPACITY_MAH` | passed through to the model |
| `--cycle-epsilon SECONDS` | `10.0` (`5.0` since 2026-09-24) | anchor cluster width (§2.1) |
| `--max-hold SECONDS` | `7200.0` | forward-fill dwell cap (§2.4) |
| `--outlier-threshold MS` | `5000.0` | per-cycle HP-awake outlier cut |
| `--start ISO` / `--end ISO` | none | trim the window without re-exporting |
| `--temperature-corrected` | off | reconcile against the temperature-corrected current |
| `--run` / `--no-run` | `--run` | invoke `power_model.summarize()` inline (§8 Q1) |
| `--list-entities` | off | print short names and row counts, then exit |
| `--force` | off | downgrade fatal guard rails to warnings (§8 Q3) |
| `--quiet` | off | emit only the `power_model.py` command line, for scripting |

`--run` calls `power_model.summarize()` in-process rather than re-shelling -- same module, no argv
round-trip -- while the printed command line stays the reproducible artifact.

Three properties of the output that are design rather than formatting:

1. **Every derived line carries its provenance inline** -- `cadence 234.34 s = T/(n-1)`,
   `poll period 20.3442 s +/-0.033 % (heater_run_count, 8 increments x 4320 ticks)`,
   `sampled 3264/3390 (96.3 %), held 126, missing 0`. A number pasted into a memory file then
   arrives with its derivation attached, which is the failure mode this project's records show.
2. **Precision is typography.** Phase totals are sums of logged values, good to ~0.1 %, and print
   to two decimals. The measured current is +/-13 % and prints as `600 uA +/-77 uA` -- three
   significant figures at most, never four. §3 asks for this; making it a formatting rule is how
   it survives contact with a hurried edit.
3. **The command line prints even on `--run`, and even under `--force`** (annotated
   `# FORCED: boot_count not flat`). A reproducible artifact must be reproducible, and a forced
   run must be self-incriminating in shell history.

Guard-rail behaviour: a failed rail prints the guard block, exits **2**, and prints nothing else --
no partial numbers to copy. Under `--force` every `[fail]` becomes `[FORCED]`, the banner is
printed both before *and* after the numbers, and each violation appears as a comment on the emitted
command line. Exit codes: `0` clean, `1` warnings, `2` fatal, `3` forced-past-fatal, so a wrapper
can tell "quotable" from "experiment" without parsing text. `--force` never suppresses a warning.

One asymmetry worth stating: the LP-estimator disagreement and non-integer `k` checks are fatal,
while low per-metric coverage is only a warning. Coverage is a property of HA de-duplication
(96.3 % is normal, not degraded), whereas two independent LP-period estimators disagreeing means
one is measuring something other than what it claims -- which invalidates `--lp-poll-interval`,
and through `sensor_samples_extra_ua()` that is a ~147 uA term, a quarter of the budget.

---

## 6. Verification

Regression fixtures recomputed under the definitions above. The script must reproduce these
within the stated tolerance or it is wrong.

**Re-verified 2026-09-09**, independently, from the raw CSVs: every value in both tables below
reproduces, *provided* the anchor is the eight-entity per-cycle measurement union of §2.1. Under
the draft's "all per-cycle entities" wording window A gives 3395 / 233.996 s instead, which is why
§2.1 now names the set explicitly. Two mean values below are `total / (n - 1)` rather than
`total / n` because one cycle in window B has a `None` in `radio_tx_time`; both forms sit inside
the +/-0.1 % tolerance, but the extractor must state which it uses.

**Window A -- ss=4, 2026-08-29T05:00 -> 2026-09-07T09:36** (`metrics_history_2026-08-29__to__2026-09-07.csv`)

| Quantity | Value | Tolerance |
|---|---|---|
| cycles / T | 3390 / 794182 s | exact |
| cadence | 234.34 s | +/-0.5 s (anchor-definition sensitive) |
| safeguard mode / share | 304.1 s / 56 % | +/-1 s / +/-2 pp |
| TX total / mean | 335.6 s / 99.01 ms | +/-0.1 % |
| RX total / mean | 1234.8 s / 364.25 ms | +/-0.1 % |
| CPU total / mean | 1689.1 s / 498.26 ms | +/-0.1 % |
| negative-residual cycles | 0 | exact |
| LP period (heater) | 20.3442 s | +/-0.033 % |
| LP period (safeguard p90) | 20.3559 s | +/-0.1 % |
| `--lp-poll-interval` | 20.0002 s | +/-0.01 s |
| heater busy-wait | +96 s | +/-3 s |
| outliers > 5 s | 15 cycles / 297.8 s | exact / +/-1 s |
| Arrhenius multiplier | 0.9195 | +/-0.001 |
| **measured current** | **600.0 uA** | **+/-77 uA (bootstrap)** |

**Window B -- ss=16, 2026-08-20T20:00 -> 2026-08-29T04:13** (`metrics_history_2026-08-21__to__2026-08-29.csv`)

Note this window is **not outage-free**: it contains the 2 h 11 m outage of §1.4. That is
deliberate -- it is what makes it a useful fixture, because it exercises the outlier path and the
blackout path that window A does not. It is also why its three outlier cycles carry 91.4 s.

| Quantity | Value | Tolerance |
|---|---|---|
| cycles / T | 3015 / 720791 s | exact |
| cadence | 239.15 s | +/-0.5 s |
| safeguard mode / share | 325.5 s / 54 % | +/-1 s / +/-2 pp |
| TX total / mean | 297.1 s / 98.57 ms | +/-0.1 % |
| RX total / mean | 954.7 s / 316.76 ms | +/-0.1 % |
| CPU total / mean | 1514.3 s / 502.41 ms | +/-0.1 % |
| negative-residual cycles | 0 | exact |
| LP period (heater) | 21.7868 s | +/-0.051 % |
| LP period (safeguard p90) | 21.7928 s | +/-0.1 % |
| `--lp-poll-interval` | 20.1108 s | +/-0.01 s |
| heater busy-wait | +97 s | +/-3 s |
| outliers > 5 s | 3 cycles / 91.4 s | exact / +/-1 s |
| Arrhenius multiplier | 1.0327 | +/-0.001 |
| **measured current** | **1253.4 uA** | **+/-77 uA (bootstrap)** |

Two independent cross-checks that the pipeline already passes, worth keeping as assertions
because they connect this script to prior work:

* Window B's `--lp-poll-interval` of 20.1108 s reproduces the +110.8 ms anomaly recorded in
  `lp_poll_period_s()`'s docstring, from a different code path.
* Both Arrhenius multipliers (0.9195, 1.0327) match the "0.920, 1.033" pair already listed in
  `arrhenius_multiplier()`'s docstring.

Both windows genuinely pass every §4 guard rail -- `boot_count` flat at 272, `tx_power_active`
flat at 20, `sensor_samples` flat at 4 and 16. That was verified, not assumed.

### 6.1 How the fixtures are actually run

Stdlib `unittest`, three tiers, no config file and no packaging:

```
python3 -m unittest discover -s tools -p "test_*.py" -v
```

Discovery puts `tools/` on `sys.path[0]`, so `import ha_history` resolves with no `__init__.py`.
That is also the entire CI job, whenever CI arrives -- there is no `.github/` today.

* **Tier 1 -- synthetic, ground truth by construction.** Generate CSVs into a
  `TemporaryDirectory()` from a seeded `random.Random`: a planted cadence, a 4320-tick heater
  counter, per-cycle phase values, a de-duplication pass that drops unchanged repeats, HA-restart
  blackout clusters, one outage, and a headerless variant. Assert the extractor recovers the
  planted values exactly. This is the only tier that can prove the fill restores *precisely* the
  rows the generator dropped, and the only coverage the headerless path (§5.6) gets at all.
* **Tier 2 -- distilled real fixtures, committed.** Do **not** commit the 3.8 MB / 3.4 MB raw
  exports. Commit `tools/testdata/window_a_ss4.csv.gz` and `window_b_ss16.csv.gz`: the same two
  windows filtered to the 11 entities this pipeline reads, at ~175 KB and ~160 KB gzipped
  (**[corrected 2026-09-09]**: measured, level 9; the draft said 192/170 KB) -- the same order as
  the already-tracked `voltage_history.csv` (138 KB), so §6 becomes a real gate on every machine
  instead of one that silently skips. They reproduce every §6 value **only once the three constant
  entities are excluded from the anchor pass** (§2.1): an 11-entity union anchors 3395 cycles, not
  3390, so "bit-for-bit" is a property of the anchor rule, not of the fixture. Cost is a two-line `gzip.open` branch in `load_export`, which usefully lets the
  user keep their own exports compressed too. A `tools/testdata/README` records the filter command
  so they are regenerable. The raw CSVs stay untracked as working exports; a third tier reads them
  under `skipUnless(os.path.exists(...))` for the few values the 11-entity subset cannot carry
  (mode share, `cca_failures` dedup bias).
* **Tier 3 -- byte-identity** for §5.5 commit 1: run the CLI through `subprocess.run` against the
  tracked `voltage_history.csv` and compare stdout to a committed 8-line string. Re-blessed once,
  in the §0 commit.

Tolerances live in one module-level table per window, one field per line, each with its §6
tolerance and a comment naming its source. A single helper carries both the absolute and relative
forms and fails with the fixture line that was violated, not just two numbers. Exact-valued
quantities (`n_cycles`, negative-residual count, outlier count, guard verdicts) use `assertEqual`
-- §6 already marks them exact, and a tolerance there would hide precisely the off-by-one-cycle
bug that §5.2 exists to prevent. No JSON fixture format: same reasoning as the
`secrets.yaml` / `device_config.yaml` precedent, don't invent a data format for something the
repo can express in its own language.

---

## 7. Known-imperfect, flagged rather than hidden

The heater busy-wait has moved out of this section into §2.3, where it is now derived.

- **LP/HP overlap.** LP-active and HP-awake windows can overlap (the cores run independently), so
  summing both double-counts. Quantified: HP-awake is 3259.5 s of 794182 s = 0.410 % of
  wall-clock, against an LP duty of 1.69 % (`0.344 / 20.3442`; the draft said ~1.8 %), so the
  overlap is bounded by ~0.007 % of the window.
  Negligible, but real.
- **`LP_ACTIVE_CURRENT_UA` is a guess** (9 mA, pinned 2026-09-09) admitting 1.4-14.7 mA.
  Everything the LP term feeds is provisional until it is measured directly -- see its `#todo` in
  `power_model.py`. Note §0 shifts the back-solve that produced it by +8 %.
- **The ADC feeds both sides.** `battery` % and `voltage` come from the same GPIO2 read, so the
  measured current and any voltage-derived correction share a systematic error that no amount of
  fitting separates.

---

## 8. Open questions for review

The draft's three questions are answered; these replace them.

**Answered.** Q1 (emit a command line, or run the model?) -> **both**: reconciliation is the only
reason the script exists, so print provenance *and* run it, with the command line included so the
run is reproducible by hand. Q2 (add `model_avg_current_ua_from_duty()`?) -> **no, and the
question was framed too narrowly** -- it assumed the module's signature was fixed. It is not, so
the answer is to remove cadence as an input rather than add an alternative beside it; §9 is that
argument in full. Q3 (refuse, or
warn?) -> **fatal with `--force`**: this project's history is a list of windows invalidated after
the fact, and a warning printed above 40 lines of output is a warning that gets pasted into a
memory file as a result. `--force` keeps the escape hatch and puts the override in the shell
history next to the number.

**Still open.**

1. **Should the extractor attribute a current to the outlier cycles, or only report their
   duration?** They are 9.1 % of HP-awake time in window A, and what current to assign them is
   exactly the unknown (`radio_rx` at 74 mA? retry TX at 305 mA?) that makes them interesting.
   Reporting duration only is honest; reporting a uA figure is what makes them comparable to the
   rest of the budget. Currently §4 does the latter with an assumption baked in.

2. **Does the safeguard-mode share belong in the guard rails?** §1.1 argues two windows with
   different mode shares are not comparable, but 56 % vs 54 % across the two fixtures is a
   coincidence, not a demonstration that it is stable. Refusing on a mode-share mismatch could
   reject every genuinely interesting window; not refusing means the weather quietly enters every
   comparison. A third window would settle it.

3. **Is the +96 s heater stall a per-run constant, or does it track ambient?** The cooldown loop
   is temperature-triggered (`kCooldownEpsilonC`, capped at `kCooldownMaxMs`), so a hot day should
   cool more slowly and stall longer. Two windows nine days apart both give 96-97 s, which either
   means the `kRhSettleMs` 90 s term dominates or means the two windows had similar weather. If it
   tracks ambient, the term belongs in the model as a function rather than a constant.

4. **SETTLED: §9.1 landed before the extractor.** The counter-argument won -- emitting
   `--lp-poll-interval` for even one revision would have baked the measured-to-modelled round trip
   into a second file. §9.2 still landed last, as §9.4 argued. See §12.

---

## 9. Changes to `power_model.py` itself

The first draft assumed this module's interface was fixed and worked around it. It is not fixed --
nothing outside this plan imports it, all four call sites are internal, and its docstring already
records one breaking change ("this used to take bare positional arguments ... old invocations need
the flags"). Four changes, in priority order.

### 9.0 First: give the model a regression anchor, because it has none

Its four documented calibration points are correct but **unlabelled as to TX power**, and the
file's default moved to 20 dBm on 2026-09-09:

| cadence | documented in the file | at the 20 dBm default | at `--tx-power 6` |
|---|---|---|---|
| 290 s | 189.4 uA | 236.6 uA | 189.4 uA |
| 600 s | 109.6 uA | 132.4 uA | 109.6 uA |
| 900 s | 84.8 uA | 100.0 uA | 84.8 uA |
| 1800 s | 59.9 uA | 67.5 uA | 59.9 uA |

They reproduce exactly at 6 dBm (verified 2026-09-09, all four), so nothing is broken -- but they
live in a comment about the *sleep-current* change and never say which TX power they assume, so
re-running the documented command shows 236.6 and reads as a regression.

**[corrected 2026-09-09] the draft's second claim here was itself wrong.** It said the "+64 uA"
figure for the 6 -> 20 dBm move should be +47.2 uA. That is the value at the *argparse default*
cadence of 290 s with the *calibration* TX time: `(305-153) mA x 0.090 s / 290 s` = 47.2 uA. But
the sentence carrying "+64 uA" is not in the `CAUTION` block at all -- it is in `TX_POWER_DBM`'s
own comment, whose subject is "every calibration point recorded in `project_power_estimates`",
i.e. this device's real ~234 s cadence with its measured TX time:
`(305-153) mA x 0.09901 s / 234.34 s` = **64.2 uA**. The existing number is right; what is missing
is its referent. Fix the ambiguity, not the arithmetic.

Nothing below is safely refactorable until that is fixed, because there is no table that says what
"unchanged behaviour" means. Pin this first, with TX power named per row -- **all nine rows below
were re-run against the current `power_model.py` on 2026-09-09 and reproduce exactly**, as do the
`sensor_samples_extra_ua` and `lp_poll_period_s` invariants beneath the table:

| case | cadence | N | `lp_poll_interval` | dBm | phases | mid uA | range | `device_total_ua` |
|---|---|---|---|---|---|---|---|---|
| historic calibration | 290.00 | 1 | 20.0000 | 6 | default | 189.4 | 180.7-198.2 | 186.3 |
| historic calibration | 600.00 | 1 | 20.0000 | 6 | default | 109.6 | 105.4-113.9 | 115.0 |
| historic calibration | 900.00 | 1 | 20.0000 | 6 | default | 84.8 | 81.9-87.6 | 92.8 |
| historic calibration | 1800.00 | 1 | 20.0000 | 6 | default | 59.9 | 58.5-61.3 | 70.6 |
| current default | 290.00 | 1 | 20.0000 | 20 | default | 236.6 | 227.9-245.3 | 228.4 |
| current default | 1800.00 | 1 | 20.0000 | 20 | default | 67.5 | 66.1-68.9 | 77.4 |
| window A (ss=4) | 234.34 | 4 | 20.0002 | 20 | A | 494.5 | 482.8-506.2 | 458.7 |
| window B (ss=16) | 239.15 | 16 | 20.1108 | 20 | B | 1011.6 | 1000.1-1023.2 | 920.5 |
| LP term off | 234.34 | 1 | 20.0002 | 20 | A | 347.8 | 336.2-359.5 | 327.7 |

with phases A = `(0.09901, 0.36425, 0.49826)` and B = `(0.09857, 0.31676, 0.50241)` -- the
forward-filled §6 values, not the mean-of-observed ones currently in `MEASURED_PHASE_TIMES_S`.
Supporting invariants worth asserting alongside: `sensor_samples_extra_ua` is 146.66 uA at window
A's settings, 684.75 uA at window B's, and exactly 0.0 at N=1; and `lp_poll_period_s` reconstructs
20.3442 s and 21.7868 s, matching §6's measured periods.

Two things this table makes visible that prose did not. The LP term is **146.66 uA at ss=4 and
684.75 uA at ss=16** -- 30 % and 68 % of the modelled rail current, resting entirely on the 9 mA
guess. And the model-vs-measured residual is 141.3 uA for window A against 332.9 uA for window B,
so it is not a constant offset -- consistent with what `LP_ACTIVE_CURRENT_UA`'s `#todo` already
says, now with two clean windows behind it.

### 9.1 `--lp-poll-interval` -> `--lp-poll-period`

Highest value, near-zero cost. §2.2's extractor measures the wake-to-wake period (20.3442 s),
subtracts `lp_active_time_s(N)` to emit 20.0002 s, and the model adds the same term straight back.
A *measured* quantity round-trips through a *modelled* one.

For ss=16 that is actively harmful: `lp_active_time_s(16)` is known wrong by +110.8 ms (§2.2), so
the subtraction bakes the error into the input and the model re-adds it. It cancels only because
the extractor performed the exact inverse; any hand invocation, or any future change to
`LP_MEASURE_S` / `LP_INTER_SAMPLE_S`, breaks the cancellation silently.

Take the measured period directly and derive the interval internally, where `duty(1)` in
`sensor_samples_extra_ua()` is the one place that genuinely needs it. The arithmetic is unchanged;
the input becomes the thing that was actually measured. This deletes §2.2's closing paragraph and
the standing warning in `lp_poll_period_s()`'s docstring.

### 9.2 Make totals primitive; derive cadence

```python
model_avg_current_ua(window_s, tx_s, rx_s, cpu_s, ...)     # totals over the window
sleep_s = window_s - (tx_s + rx_s + cpu_s)
```

This *is* §1's integral. As it stands, §1 argues at length that totals are the honest form and
then hands the model a per-cycle reconstruction to re-multiply -- the equivalence in §1.6 exists
only to justify that detour. Making totals primitive retires §1.6 from load-bearing algebra to a
footnote about the CLI.

Second benefit: `sleep_s = max(cadence_s - awake_s, 0.0)` currently clamps. In the totals form,
awake exceeding the window is not a pessimistic edge case, it is a bug in the extraction, and it
should raise rather than floor at zero.

Keep `--cadence` plus `--phase-times` as a hand-use front-end that multiplies up to totals, so
`power_model.py --cadence 290` still works for a back-of-envelope run.

### 9.3 Delete `--measured-phases`; split `summarize()`

`--measured-phases` is a frozen snapshot of window A, and it is already the wrong statistic: it
holds `(0.09913, 0.36428, 0.49810)`, the mean-of-observed that §1.5 rejects, against the
forward-filled `(0.09901, 0.36425, 0.49826)`. Once `ha_log_metrics.py` exists the extractor is the
source and the snapshot is a third copy to keep in sync (§5.5 currently has to patch it).

Separately, `summarize()` computes and prints in one pass -- the same shape §5.4 is untangling in
`battery_power_from_history.py`, for the same reason. Split it into a function returning a
namedtuple plus a printer, so §5.7's `--run` reconciles *numerically* rather than by shelling
text. That makes "self-discharge residual > 0" available as a guard rail instead of something a
human has to notice in 40 lines of output.

### 9.4 Sequencing

§9.0 is a prerequisite for everything else. After that, §9.1 and §9.3 are independent and safe in
either order. **§9.2 should land after the extractor and its §6 fixtures exist**, not before: it
touches every arithmetic path in the module, and the fixtures are what make "behaviour-preserving"
checkable rather than asserted. See §8's fourth open question for the counter-argument.

Migration is one `CAUTION` block in the module docstring, in the register the file already uses
for its 2026-09-09 constant changes -- and this time the block should name the TX power its
figures assume, which is exactly the omission §9.0 exists to fix.

### 9.5 LANDED: integer-dBm TX table and a basis error bar

Implemented 2026-09-09, purely additive -- all nine §9.0 rows reproduce unchanged, and
`tx_current_ma()` is untouched, so every recorded figure stays reproducible.

`--tx-power` and linear interpolation between the four datasheet points already existed. What was
missing was not a finer step but an error bar, because the interpolation **basis** is worth far
more than the step size: the same four points read linearly in dBm (what the model does) or
linearly in mW disagree by up to 25 mA. The sting is where that lands on this device's history --
**20 dBm is a published anchor carrying no interpolation error at all, while 6 dBm is the exact
midpoint of the 0-12 dBm gap and carries the maximum, 20.3 mA.** Every figure recorded in
`project_power_estimates` before 2026-09-09 was computed at 6 dBm.

Added: `TX_POWER_MIN_DBM` / `TX_POWER_MAX_DBM`, `tx_current_mw_basis_ma()`,
`tx_current_uncertainty_ma()`, `tx_current_table()`, `print_tx_current_table()`,
`--list-tx-table`, and a `summarize()` line that names the basis uncertainty whenever the
requested power is not one of the four anchors. Those functions' docstrings carry the derivation,
including why neither basis is globally right -- not repeated here.

For scale: 1 dB is worth 6.2 uA at 20 dBm, 2.4 uA at 0 dBm, 0.8 uA at -15 dBm (window A cadence),
against §3's +/-77 uA measurement uncertainty. That is why the step size was never the problem.

### 9.6 Not worth changing

`--measured` (§0 changes its value, not its meaning), `--capacity`, `--tx-power`, and
`--temperature-multiplier`. The last is awkward for hand use, since the caller must run
`arrhenius_multiplier()` themselves, but the extractor computes it and a `--temperature-csv`
alternative would duplicate `ha_history.py` inside the model.

---

## 10. Implementation order

The work is spread across §0, §5 and §9, each of which sequences itself but not the others. This
is the single ordered list. Nothing here is new -- it is the other sections' steps, merged and
put in dependency order.

### Step 1 -- `power_model.py` housekeeping (small, unblocks §9)

Independent of everything else, and §9.4 puts it first because the module currently has no
statement of what "unchanged behaviour" means.

1. Name the cadence behind `TX_POWER_DBM`'s "+64 uA" for the 6 -> 20 dBm move. **Do not change
   the number** -- it is correct at the device's real cadence
   (`(305-153) mA x 0.09901 s / 234.34 s` = 64.2 uA), and only reads as wrong because "at this
   cadence" has no antecedent. (An earlier revision of this plan proposed replacing it with
   +47.2 uA, which is the value at the unrelated 290 s argparse default.) While there: the
   sentence is in `TX_POWER_DBM`'s comment, not in the module docstring's `CAUTION` block.
2. Label the four calibration points in `SLEEP_CURRENT_UA`'s comment as **6 dBm**. They are
   correct and reproduce exactly at `--tx-power 6`; they simply do not say so, while the module
   default is 20 dBm -- so re-running them reads as a regression (§9.0).
3. Commit §9.0's nine-row regression table into the module, TX power named per row, plus the
   `sensor_samples_extra_ua` and `lp_poll_period_s` invariants listed there.

Already done: §9.5's integer-dBm TX table and basis error bar.

### Step 2 -- `battery_power_from_history.py`, two commits (§5.4, §5.5)

4. **Pure extraction.** Functions, `if __name__ == "__main__"` guard, `--csv` flag, `load_series`
   moved to `ha_history.py`, pack constants imported from `power_model`. Gate: CLI stdout
   byte-identical against the tracked `voltage_history.csv`. Preserve the five traps in §5.5.
5. **The §0 unit fix.** One line, plus a `CAUTION` block naming the old -> new pairs for every
   window quoted in `project_power_estimates`, plus the three stale spots in `power_model.py`
   (docstring, `--help` epilog, `MEASURED_PHASE_TIMES_S`). Re-bless the expectation in the same
   commit so the number moves next to its reason.

Steps 1 and 2 are independent; either can go first. Both must precede step 4.

### Step 3 -- `ha_history.py` (§5.1, §5.2, §5.6)

6. Parsing, `Export`/`Series`/`CycleTable`, blackout policy, the **two** fill primitives, exact
   short-name keying. No interpolation code, by construction.
7. Tier 1 synthetic tests (§6.1) alongside, not after -- they are the only coverage the headerless
   path and the de-duplication reconstruction get.

### Step 4 -- `ha_log_metrics.py` (§2, §4, §5.3, §5.7)

8. `derive()` and the guard rails, then `report()` and the CLI.
9. Distil and commit the two `.csv.gz` fixtures (§6.1 tier 2), then pin §6's two tables.

At the end of this step the plan's original goal is met: a window can be reconciled against the
model without hand-extracting anything.

### Step 5 -- the `power_model.py` API changes (§9.1, §9.2, §9.3)

10. `--lp-poll-interval` -> `--lp-poll-period` (§9.1), which then deletes §2.2's workaround
    paragraph and the emit-side subtraction in step 8.
11. Delete `--measured-phases`; split `summarize()` into compute + print (§9.3), which lets
    step 8's `--run` reconcile numerically.
12. Totals primitive, cadence derived (§9.2). Last, because it touches every arithmetic path and
    step 3's regression table plus §6's fixtures are what make it checkable.

§8's fourth open question argues step 10 could be pulled forward to before step 8, to avoid baking
the round-trip into a second file. That is the one ordering decision left open here.

### What is deliberately not scheduled

The three open questions in §8 (outlier current attribution, mode share as a guard rail, whether
the +96 s heater stall tracks ambient) and the four items in §7. None blocks any step above; all
need either a third measurement window or a current probe, not code.

---

## 11. Verification log (2026-09-09)

Every statement above was re-derived from the repository rather than carried forward. What was
checked, and against what:

**Firmware, line by line.** `publish_gap_sec_to_skip_cycles` at `main/main.cpp:127` (§1.1);
`expire_after_sec = 2 x safeguardWakeSec` at `main/main.cpp:346` with
`safeguardWakeSec = (maxSkip + 3) x pollSec` in `main/sensorstask.h:172`, giving 680 s from
`device_config.yaml`'s `max_publish_gap_sec: 300` / `lp_poll_interval_sec: 20` (§2.4);
`readLinkStats()` and `hp_awake_stats_get_and_reset_us()` both inside `if (bits & BIT_CONNECTED)`
opened at `main/mqtt_sender.cpp:1274` (§1.4); the boot-baseline branch and the uint64 -> uint32
delta truncation at `main/net/openthread_link.cpp:258-261`, against `otRadioTimeStats::mTxTime`
being `uint64_t` in OpenThread's `radio_stats.h` -- 71.6 min to wrap, as §4 says;
`kRhSettleMs = 90000` at `components/lp_sensor_core/lp_core/main.cpp:92` with
`kCooldownPollMs = 2000`, matching §2.3's "90 s + about three cooldown iterations" decomposition;
and `cycles_since_heater`'s increment/reset, which is what proves the §2.2 bias found above.

**Reproduced exactly** (both fixture CSVs, recomputed from raw rows): §6's two tables in full --
cycle counts, window lengths, cadences, safeguard mode and share, TX/RX/CPU totals and means,
zero negative residuals, both LP-period estimators and their error bars, both `--lp-poll-interval`
values including the +110.8 ms ss=16 anomaly, the heater stall gaps (401/401/401/400/400 and the
303 s wrong-side value), the outlier counts and durations, both Arrhenius multipliers, and the
guard-rail constants (`boot_count` 272, `tx_power_active` 20, `sensor_samples` 4 / 16). Also §0 in
full (564.7 -> 600.0, 1168.6 -> 1253.4, `voltage_history.csv` 0.437 -> 0.483, and the 604 -> 653
delta), §1.3's mean/median/max, §1.4's recovery-cycle telemetry, §4's outlier uA, §7's 0.410 %,
and every one of §9.0's nine rows plus §9.5's 20.3 / 25.4 mA and 6.2 / 2.4 / 0.8 uA figures.

**Independent corroboration of §0.** Fitting HA's `battery` percentage series directly -- no
voltage curve, no pack voltage, no energy conversion anywhere in the path -- gives 600.2 uA on
window A, against §0's charge-corrected 600.0 uA and the script's printed 564.7 uA. The unit bug
is real and the correction lands on the right number.

**Corrected here.** The anchor-set contradiction (§2.1, propagating to §5.2 and §6.1), the heater
estimator's uncorrected stall bias and its missing high-RH guard rail (§2.2, §4), the "+64 uA"
non-error (§9.0, step 1), the outage duration (§1.4), the 125/126 and 429/428 off-by-ones (§1.5),
two mixed coverage/mean definitions (§1.5), the §2.3 keep-rule that did not reproduce its own
count, the §2.4 blackout and silence lists, §3's unlabelled confidence level, and the smaller
figures in §5.4, §5.6, §6.1 and §7.

**Not reproduced, flagged rather than fixed.** §3's +/-0.077 mA: the point estimate reproduces,
the interval does not (1 sigma 26-29 uA, 95 % half-width ~56 uA under a 24 h residual block
bootstrap over 2000 replications). The recipe needs pinning before the number is quoted again.

---

## 12. What landed (2026-09-09)

Implemented in full, in the dependency order of §10 with §9.1 pulled forward. Every §6 value in
both tables above is reproduced by `tools/ha_log_metrics.py` and pinned as a test.

    python3 -m unittest discover -s tools -p "test_*.py"      # 32 tests
    python3 tools/power_model.py --check-regression
    python3 tools/ha_log_metrics.py --csv <export>.csv --run

### The four decisions that were open

| Question | Decision | Where it shows |
|---|---|---|
| Scope | all of §10 | steps 1-12 all landed |
| §2.2's heater-stall bias | **report both** figures, error bar spans them | `LpPeriod.raw_s` / `.corrected_s`, `--lp-period-basis` |
| §3's unreproducible +/-0.077 mA | **re-derive** with the recipe pinned in code | `ha_history.slope_block_bootstrap`, seed included |
| §8 Q4 ordering | **§9.1 first** | `--lp-poll-period` landed before the extractor |

The bootstrap now prints what it computes: **600 uA +/-56 uA** on window A and
**1253 uA +/-68 uA** on window B, both 95 % intervals from a 24 h residual block bootstrap over
2000 replications with the seed fixed. `BOOTSTRAP_UA = 77.0` was retired rather than carried.

### Files

| File | Status |
|---|---|
| `tools/power_model.py` | REGRESSION_TABLE + `--check-regression`; `--lp-poll-interval` -> `--lp-poll-period`; `summarize()` split into `power_budget()` + `print_budget()`; `--measured-phases` deleted; window totals now the primitive with `cadence_current_ua()` the front end |
| `tools/battery_power_from_history.py` | split into functions with a `main` guard and `--csv`; §0 unit fix applied |
| `tools/ha_history.py` | new; the shared loader, two fill primitives, blackout policy, bootstrap |
| `tools/ha_log_metrics.py` | new; the extractor, guard rails, report, CLI |
| `tools/test_ha_history.py` | new; 20 tier-1 synthetic tests |
| `tools/test_ha_log_metrics.py` | new; 12 tier-2/3 tests pinning §6 |
| `tools/testdata/*.csv.gz` + `README` | new; 175 KB / 160 KB distilled fixtures |

### Three things the implementation changed about this design

1. **Boundary rows are reported, not filtered** -- see §2.1. Filtering them would have broken §6.
2. **Edge-anchored blackouts are a warning, not a fatal rail.** §4 says an `unavailable` run
   longer than the dwell cap invalidates a window. Window B fails that rule outright: HA does not
   know `heater_run_count` for the first 26 hours of the export, because the entity only changes
   once a day, so it reads `unavailable` from the export's first instant. That is HA's ignorance
   at a boundary, not a device event. `Blackout.edge` marks a run that begins at the first row or
   never recovers before the last; only INTERIOR runs are fatal. Without this §6's "both windows
   genuinely pass every guard rail" is false.
3. **`series_for()` resolves any name on a single-series export.** The plan scoped this to
   headerless exports, but the repo's own `voltage_history.csv` HAS an `entity_id` column with
   exactly one entity -- and a lone id yields no meaningful device prefix, so its series is keyed
   `esp32_ot_mqtt_outdoor_voltage`. Any export with one series cannot be ambiguous, so any
   requested name resolves to it.

### Still open

The three questions in §8 (outlier current attribution -- the extractor prints a uA figure and
labels it an assumption; mode share as a guard rail; whether the +96 s heater stall tracks
ambient) and the four items in §7. All need a third measurement window or a current probe.

One new item: the **high-RH heater guard rail has no real-data coverage**. `kHighRhThreshold` is
90 %RH and neither fixture window exceeds 80.3 %; the rail exists on the strength of the firmware
alone, and an outdoor sensor will eventually exercise it.

