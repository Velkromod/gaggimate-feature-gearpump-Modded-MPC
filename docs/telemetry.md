# Shot telemetry workflow

This repository includes two telemetry layers:

1. Firmware-side shot markers and CSV telemetry emitted over serial.
2. Host-side capture tooling that converts each shot into a clean `.csv` plus a `.metadata.json` sidecar.

## Firmware behavior

`DimmedPump.cpp` emits:
- `=== SHOT TELEMETRY START ===`
- a legend block using `# key=value` lines
- a CSV header row
- shot rows every telemetry tick
- `=== SHOT TELEMETRY END ===`

The visual recipe reference and the control reference are split into:
- `sp_recipe`: the intended visual profile reference
- `sp_ctrl`: the internally conditioned control reference

## Host-side capture

Use `tools/telemetry/capture_shots.py` to record one file set per shot:
- `shot_*.csv`
- `shot_*.metadata.json`
- `shot_*.raw.log`

The metadata sidecar is the source of truth for channel descriptions and shot context.

## Recommended workflow

1. Run the firmware with serial telemetry enabled.
2. Start the host capture script.
3. Pull a shot.
4. Share the generated CSV and JSON together during analysis.

## Tachometer channels

The firmware can now append tachometer-derived channels to the CSV when a pump tach signal is wired in:

- `tach_period_us`: robust tach period estimate after hardware/software deglitching
- `tach_rpm_inst`: instantaneous RPM computed from the validated period estimate
- `tach_rpm_ema`: lightly filtered published RPM for easier visual inspection
- `tach_rpm_count_window`: RPM derived from accepted pulse counts over a fixed time window
- `tach_rpm_pub`: RPM selected for operational use after comparing the period and count-window estimates
- `tach_quality_ok`: `1` when period and count-window RPM agree within tolerance
- `tach_rpm_source`: `0=none`, `1=period branch`, `2=count-window fallback`
- `tach_pulse_count`: running count of accepted tach pulses
- `tach_glitch_rejects`: combined count of glitch/outlier rejects used to stabilise the tach signal
- `tach_timeout`: `1` when RPM is forced to zero because pulses stopped arriving
