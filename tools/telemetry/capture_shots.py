#!/usr/bin/env python3
"""
Serial shot capture for GaggiMate-style telemetry.

Features:
- Detects shot boundaries from serial markers.
- Writes one clean CSV per shot.
- Writes one metadata JSON sidecar per shot.
- Optionally writes a raw log for each shot.
- Tolerates comment lines and variant start/end markers.

Recommended firmware markers:
    === SHOT TELEMETRY START ===
    # firmware=...
    # build=...
    # profile=...
    # legend_version=1
    # raw_p = raw pressure sensor reading
    ...
    ms,raw_p,flt_p,...
    123,1.23,1.20,...
    === SHOT TELEMETRY END ===
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import signal
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

try:
    import serial  # type: ignore
except Exception as exc:  # pragma: no cover
    print("Missing dependency: pyserial. Install with: pip install pyserial", file=sys.stderr)
    raise


START_MARKERS = {
    "=== SHOT TELEMETRY START ===",
    "# SHOT START",
    "SHOT START",
}
END_MARKERS = {
    "=== SHOT TELEMETRY END ===",
    "# SHOT END",
    "SHOT END",
}

COMMENT_PREFIX = "#"
KV_RE = re.compile(r"^#\s*([A-Za-z0-9_.-]+)\s*=\s*(.+?)\s*$")
DESC_RE = re.compile(r"^#\s*([A-Za-z0-9_.-]+)\s*=\s*(.+?)\s*$")


DEFAULT_CHANNEL_DESCRIPTIONS: Dict[str, Dict[str, str]] = {
    "ms": {"unit": "ms", "description": "Monotonic time since boot or controller startup."},
    "t_s": {"unit": "s", "description": "Elapsed shot time in seconds."},
    "raw_p": {"unit": "bar", "description": "Raw pressure sensor reading."},
    "flt_p": {"unit": "bar", "description": "Filtered pressure used by the controller."},
    "sp_raw": {"unit": "bar", "description": "Raw setpoint currently held by the profile engine."},
    "sp_flt": {"unit": "bar", "description": "Filtered setpoint used internally by the controller."},
    "sp_recipe": {"unit": "bar", "description": "Visual recipe reference; ideal profile target."},
    "sp_ctrl": {"unit": "bar", "description": "Conditioned control reference after filtering/guards."},
    "sp_flt_d": {"unit": "bar/s", "description": "Derivative of the filtered setpoint."},
    "ctrl_out": {"unit": "%", "description": "Final controller output command in percent."},
    "pump_duty": {"unit": "ratio", "description": "Internal normalized pump duty estimate."},
    "pump_flow": {"unit": "ml/s", "description": "Estimated pump flow."},
    "coffee_flow": {"unit": "ml/s", "description": "Estimated coffee flow through the puck."},
    "u_raw": {"unit": "%", "description": "Pressure branch output before final slew limiting."},
    "u_applied": {"unit": "%", "description": "Pressure branch output after final slew limiting."},
    "u_delta": {"unit": "%", "description": "Difference between raw and applied output."},
    "limiter_active_up": {"unit": "bool", "description": "1 when rise slew limiter is active."},
    "limiter_active_down": {"unit": "bool", "description": "1 when drop slew limiter is active."},
    "error_integral": {"unit": "controller_state", "description": "Integral state of the legacy pressure controller."},
    "u_unclamped_raw": {"unit": "%", "description": "Controller output before 0..100 clamp."},
    "u_clamped_raw": {"unit": "%", "description": "Controller output after clamp and before slew limiting."},
    "u_clamp_delta": {"unit": "%", "description": "Difference introduced by output clamping."},
    "clamp_active_high": {"unit": "bool", "description": "1 when upper clamp is active."},
    "clamp_active_low": {"unit": "bool", "description": "1 when lower clamp is active."},
    "u_fb": {"unit": "%", "description": "Legacy pressure feedback contribution."},
    "u_ff_hold": {"unit": "%", "description": "Static feedforward contribution."},
    "u_ff_dyn": {"unit": "%", "description": "Dynamic feedforward contribution."},
    "u_ff_total": {"unit": "%", "description": "Total feedforward contribution."},
    "mpc_shadow_enabled": {"unit": "bool", "description": "1 when shadow MPC is computed in parallel."},
    "mpc_u_shadow": {"unit": "%", "description": "Shadow MPC suggested total output; not applied in shadow mode."},
    "mpc_u_ss": {"unit": "%", "description": "Steady-state component suggested by shadow MPC."},
    "mpc_u_trim": {"unit": "%", "description": "Corrective trim component suggested by shadow MPC."},
    "mpc_p1_pred": {"unit": "bar", "description": "One-step pressure prediction from the shadow MPC model."},
    "mpc_pn_pred": {"unit": "bar", "description": "Terminal pressure prediction at horizon end."},
    "mpc_qout_est": {"unit": "ml/s", "description": "Estimated outflow/load used by the shadow MPC model."},
    "mpc_residual": {"unit": "bar", "description": "Measured minus predicted pressure residual."},
    "mpc_cost": {"unit": "objective", "description": "Shadow MPC objective value."},
    "power_cmd": {"unit": "%", "description": "Floating-point power command before integer quantization."},
    "power_psm_quantized": {"unit": "%", "description": "Integer power sent to the phase-angle modulator."},
    "power_quant_residual": {"unit": "%", "description": "Sigma-delta residual carried to the next quantization step."},
}


@dataclass
class ShotSession:
    base_path: Path
    shot_started_at: datetime
    port: str
    baud: int
    cli_profile: Optional[str] = None
    write_raw_log: bool = True
    metadata: Dict[str, str] = field(default_factory=dict)
    channel_notes: Dict[str, str] = field(default_factory=dict)
    header: Optional[List[str]] = None
    csv_file: Optional[object] = None
    csv_writer: Optional[csv.writer] = None
    raw_file: Optional[object] = None
    rows_written: int = 0

    def open_files(self) -> None:
        self.base_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.base_path.with_suffix(".csv").open("w", newline="", encoding="utf-8")
        if self.write_raw_log:
            self.raw_file = self.base_path.with_suffix(".raw.log").open("w", encoding="utf-8")

    def write_raw(self, line: str) -> None:
        if self.raw_file is not None:
            self.raw_file.write(line + "\n")

    def set_header(self, header_line: str) -> None:
        columns = [c.strip() for c in header_line.split(",")]
        self.header = columns
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(columns)

    def write_row(self, row_line: str) -> None:
        if self.csv_writer is None:
            raise RuntimeError("CSV header not set before data row.")
        cols = [c.strip() for c in row_line.split(",")]
        if self.header is not None and len(cols) != len(self.header):
            # Preserve raw log but do not corrupt CSV structure.
            return
        self.csv_writer.writerow(cols)
        self.rows_written += 1

    def build_metadata_document(self) -> Dict[str, object]:
        started = self.shot_started_at.isoformat(timespec="seconds")
        data_file = self.base_path.with_suffix(".csv").name
        metadata_file = self.base_path.with_suffix(".metadata.json").name
        profile = self.metadata.get("profile") or self.cli_profile or "unknown"

        channels: Dict[str, Dict[str, str]] = {}
        for name in self.header or []:
            info = dict(DEFAULT_CHANNEL_DESCRIPTIONS.get(name, {}))
            if name in self.channel_notes:
                info["description"] = self.channel_notes[name]
            if not info:
                info = {"unit": "unknown", "description": "No description captured for this channel."}
            channels[name] = info

        return {
            "telemetry_schema_version": 1,
            "generated_by": "capture_shots.py",
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "shot_started_at": started,
            "serial": {
                "port": self.port,
                "baud": self.baud,
            },
            "shot": {
                "profile": profile,
                "rows_written": self.rows_written,
                "data_file": data_file,
                "metadata_file": metadata_file,
            },
            "firmware": {
                "firmware": self.metadata.get("firmware", "unknown"),
                "build": self.metadata.get("build", "unknown"),
                "legend_version": self.metadata.get("legend_version", "unknown"),
                "time_base": self.metadata.get("time_base", "unknown"),
                "dt_s": self._parse_float(self.metadata.get("dt_s")),
            },
            "channels": channels,
            "notes": self.metadata,
        }

    @staticmethod
    def _parse_float(value: Optional[str]) -> Optional[float]:
        if value is None:
            return None
        try:
            return float(value)
        except ValueError:
            return None

    def close(self) -> Path:
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        if self.raw_file is not None:
            self.raw_file.flush()
            self.raw_file.close()
        metadata_path = self.base_path.with_suffix(".metadata.json")
        doc = self.build_metadata_document()
        metadata_path.write_text(json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")
        return metadata_path


def is_header_line(line: str) -> bool:
    if not line or line.startswith(COMMENT_PREFIX):
        return False
    low = line.lower()
    return ("raw_p" in low and "flt_p" in low) or ("sp_raw" in low and "ctrl_out" in low)


def is_probable_csv_row(line: str) -> bool:
    if not line or line.startswith(COMMENT_PREFIX):
        return False
    if "," not in line:
        return False
    first = line.split(",", 1)[0].strip()
    try:
        float(first)
        return True
    except ValueError:
        return False


def make_base_path(out_dir: Path, profile: Optional[str]) -> Path:
    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    safe_profile = re.sub(r"[^A-Za-z0-9_.-]+", "_", (profile or "shot").strip())
    return out_dir / f"{safe_profile}_{ts}"


def run_capture(args: argparse.Namespace) -> int:
    out_dir = Path(args.outdir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    ser = serial.Serial(args.port, args.baud, timeout=1)
    current: Optional[ShotSession] = None
    stop_requested = False

    def handle_sigint(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, handle_sigint)

    print(f"Listening on {args.port} @ {args.baud}. Output directory: {out_dir}")
    print("Waiting for shot markers...")

    while not stop_requested:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        stripped = line.strip()

        if stripped in START_MARKERS:
            if current is not None:
                current.close()
                print("Closed previous shot due to a new start marker.")
            current = ShotSession(
                base_path=make_base_path(out_dir, args.profile),
                shot_started_at=datetime.now(),
                port=args.port,
                baud=args.baud,
                cli_profile=args.profile,
                write_raw_log=not args.no_raw_log,
            )
            current.open_files()
            current.write_raw(line)
            print(f"Shot started -> {current.base_path.with_suffix('.csv').name}")
            continue

        if current is None:
            continue

        current.write_raw(line)

        if stripped in END_MARKERS:
            metadata_path = current.close()
            print(f"Shot finished -> {metadata_path.name}")
            current = None
            continue

        if stripped.startswith(COMMENT_PREFIX):
            m = KV_RE.match(stripped)
            if m:
                key, value = m.groups()
                if key in DEFAULT_CHANNEL_DESCRIPTIONS:
                    current.channel_notes[key] = value
                else:
                    current.metadata[key] = value
            continue

        if current.header is None and is_header_line(stripped):
            current.set_header(stripped)
            continue

        if current.header is not None and is_probable_csv_row(stripped):
            current.write_row(stripped)
            continue

    if current is not None:
        metadata_path = current.close()
        print(f"Capture interrupted. Closed current shot -> {metadata_path.name}")

    ser.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Automatically capture shot telemetry into CSV + metadata JSON.")
    p.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    p.add_argument("--outdir", default="./shots", help="Output directory for captured files")
    p.add_argument("--profile", default=None, help="Optional profile name to embed in file names and metadata")
    p.add_argument("--no-raw-log", action="store_true", help="Do not save the per-shot raw serial log")
    return p


if __name__ == "__main__":
    raise SystemExit(run_capture(build_parser().parse_args()))
