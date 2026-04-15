# Automatic shot capture: CSV + metadata.json

This bundle captures shot telemetry from a serial port and automatically writes:
- one clean CSV per shot
- one metadata JSON sidecar per shot
- one raw serial log per shot

## Why this workflow

CSV is best kept as a clean table. Rich metadata belongs in a sidecar JSON file.
This makes the files easier to parse, compare, and archive.

## Files

- `capture_shots.py`: serial capture script
- `requirements.txt`: Python dependency list
- `telemetry_metadata_example.json`: example sidecar format
- `firmware_markers_example.txt`: minimal firmware output contract

## Install

```bash
python -m pip install -r requirements.txt
```

## Run

Linux/macOS example:

```bash
python capture_shots.py --port /dev/ttyUSB0 --baud 115200 --outdir ./shots --profile "Cremina_Lever"
```

Windows example:

```bash
python capture_shots.py --port COM5 --baud 115200 --outdir .\shots --profile "Cremina_Lever"
```

## What the script listens for

Start markers accepted:
- `=== SHOT TELEMETRY START ===`
- `# SHOT START`
- `SHOT START`

End markers accepted:
- `=== SHOT TELEMETRY END ===`
- `# SHOT END`
- `SHOT END`

Between those markers, the script accepts:
- comment lines starting with `#`
- one CSV header line
- CSV data rows

## Recommended firmware output contract

Use a clean block like this:

```text
=== SHOT TELEMETRY START ===
# firmware=gaggimate-v3-shadow
# build=stage01
# profile=Cremina Lever
# legend_version=1
# time_base=ms_since_boot
# dt_s=0.03
# raw_p=Raw pressure sensor reading.
# flt_p=Filtered pressure used by controller.
# sp_recipe=Visual recipe target.
# sp_ctrl=Conditioned control reference.
ms,raw_p,flt_p,sp_raw,sp_flt,sp_recipe,sp_ctrl,...
123,1.23,1.20,0.00,0.00,0.00,0.00,...
=== SHOT TELEMETRY END ===
```

## Output example

For a shot captured at 2026-04-15 21:34:10 with profile `Cremina_Lever`, you will get:

- `Cremina_Lever_2026-04-15_21-34-10.csv`
- `Cremina_Lever_2026-04-15_21-34-10.metadata.json`
- `Cremina_Lever_2026-04-15_21-34-10.raw.log`

## Notes

- The CSV contains only the header and data rows.
- The metadata JSON stores firmware/build/profile information and channel descriptions.
- The raw log is helpful when debugging malformed rows or missing markers.
