# arm2d.py
"""
High-level 2-DOF arm API (thin wrapper over Teensy serial commands).

Overview:
    This module provides a minimal Python interface around your existing
    Teensy/Arduino sketch. It does NOT change firmware behavior; it only
    formats single-line commands and parses the textual status the sketch
    prints (e.g., lines like "[J1] raw_deg=... math_deg=..." and "[EE] x=...").

Responsibilities:
    - Provide convenience calls for homing, absolute/relative joint moves,
      absolute/relative Cartesian (IK) moves, and status queries.
    - Parse key numbers from the sketch's human-readable prints using regex.

Non-Responsibilities:
    - No trajectory generation, safety interlocks, or timing guarantees here.
    - No unit conversion beyond passing through degrees/mm used by firmware.

Typical usage:
    >>> from arm2d import Arm2D
    >>> arm = Arm2D()                 # auto-detects serial port via TeensyLink
    >>> arm.initialize()              # runs homing, raises on failure
    >>> arm.move_math(180, 90)        # absolute math angles in degrees
    >>> arm.move_delta_xyz(dx=10)     # nudge end-effector +10 mm in X
    >>> st = arm.status()["parsed"]   # dict with j1_math, j2_math, x_mm, ...
    >>> arm.close()

Protocol assumptions (must match your sketch):
    - "h" → performs full homing and prints a line containing "HOMING DONE"
    - "p" → prints a multi-line status that includes:
        "[J1] ... raw_deg=<f> ... math_deg=<f> ..."
        "[J2] ... raw_deg=<f> ... math_deg=<f> ..."
        "[EE] x=<f> mm y=<f> mm z=<f> mm"
    - "M <j1_deg> <j2_deg>" → executes absolute math-angle move and prints
      some lines that either include "[EE] ... mm" or "MOVE:MATH"
    - "G <x> <y> <z>" → executes Cartesian IK move and prints a line with "[EE] ... mm"
    - "Q <x> <y> <z>" → analyze reachability (no motion), prints text reply

Units:
    - Angles are in degrees (math angles defined by your firmware).
    - Positions are in millimeters in the base frame used by your sketch.
"""

from typing import Dict, Any, List, Optional
from dataclasses import dataclass
import re
from teensy_link import TeensyLink


@dataclass
class MoveResult:
    """Container for motion outcomes returned by the firmware."""
    ok: bool                  # Heuristic success flag based on printed lines
    reply: List[str]          # Raw text lines printed by the Teensy sketch


class Arm2D:
    """
    Thin wrapper over your Arduino serial commands.
    Your Teensy sketch stays exactly as-is.

    Args:
        port (Optional[str]): Serial port to open (e.g., "COM5", "/dev/ttyACM0").
                              If None, TeensyLink will try to auto-detect.
        baud (int): UART baud rate (must match the sketch; default 115200).

    Attributes:
        link (TeensyLink): Underlying line-based serial transport.
    """
    def __init__(self, port: Optional[str]=None, baud: int=115200):
        self.link = TeensyLink(port=port, baud=baud)

    def close(self):
        """Close the underlying serial link (best-effort cleanup)."""
        self.link.close()

    # ---- Initialization (homing) ----
    def initialize(self) -> Dict[str, Any]:
        """
        Run full homing then return parsed status.

        Returns:
            Dict[str, Any]: {"reply": [...], "parsed": {...}} from status(); this
            method returns only the "parsed" dict for convenience.

        Raises:
            RuntimeError: If homing does not report success per sketch prints.
        """
        res = self.home()
        if not res.ok:
            raise RuntimeError("Homing failed or blocked. See reply:\n" + "\n".join(res.reply))
        return self.status()["parsed"] or {}

    def home(self) -> MoveResult:
        """
        Invoke the firmware's homing routine.

        Returns:
            MoveResult: ok=True iff any returned line contains "HOMING DONE"
                        (case-insensitive check done via .upper()).
        """
        lines = self.link.send_command("h", overall_timeout=15.0)
        ok = any("HOMING DONE" in ln.upper() for ln in lines)
        return MoveResult(ok=ok, reply=lines)

    # ---- Status ----
    def status(self) -> Dict[str, Any]:
        """
        Query current pose and joint angles as printed by the sketch.

        Returns:
            Dict[str, Any]:
                {
                  "reply": [<raw lines>],
                  "parsed": {
                      "j1_raw": float,
                      "j1_math": float,
                      "j2_raw": float,
                      "j2_math": float,
                      "x_mm": float, "y_mm": float, "z_mm": float
                  } or None if parsing failed
                }
        """
        lines = self.link.send_command("p", overall_timeout=2.5)
        parsed = parse_status(lines)
        return {"reply": lines, "parsed": parsed}

    # ---- Motion by angles (math angles) ----
    def move_math(self, j1_deg: float, j2_deg: float) -> MoveResult:
        """
        Command absolute math angles for J1 and J2.

        Args:
            j1_deg (float): Target math angle for J1 in degrees.
            j2_deg (float): Target math angle for J2 in degrees.

        Returns:
            MoveResult: ok=True if response contains either an end-effector line
            with "mm" or an explicit "MOVE:MATH" marker printed by the sketch.
        """
        lines = self.link.send_command(f"M {j1_deg:.3f} {j2_deg:.3f}", overall_timeout=10.0)
        ok = any("[EE]" in ln and "mm" in ln for ln in lines) or any("MOVE:MATH" in ln for ln in lines)
        return MoveResult(ok=ok, reply=lines)

    # ---- Motion by Cartesian IK ----
    def move_xyz(self, x_mm: float, y_mm: float, z_mm: float) -> MoveResult:
        """
        Command an IK move to an absolute Cartesian target.

        Args:
            x_mm (float): Target X position in millimeters.
            y_mm (float): Target Y position in millimeters.
            z_mm (float): Target Z position in millimeters.

        Returns:
            MoveResult: ok=True if an "[EE] ... mm" line is observed.
        """
        lines = self.link.send_command(f"G {x_mm:.3f} {y_mm:.3f} {z_mm:.3f}", overall_timeout=12.0)
        ok = any("[EE]" in ln and "mm" in ln for ln in lines)
        return MoveResult(ok=ok, reply=lines)

    # ---- Analyze reachability (no motion) ----
    def analyze_xyz(self, x_mm: float, y_mm: float, z_mm: float) -> Dict[str, Any]:
        """
        Ask firmware to analyze reachability for a Cartesian point (no move).

        Returns:
            Dict[str, Any]: {"reply": [<raw lines>]} – pass/fail semantics are
            left to the firmware text and the caller.
        """
        lines = self.link.send_command(f"Q {x_mm:.3f} {y_mm:.3f} {z_mm:.3f}", overall_timeout=4.0)
        return {"reply": lines}
    
        # ---- NEW: change math angles by deltas from current ----
    def move_delta_math(self, dJ1: float | None = None, dJ2: float | None = None):
        """
        Increment current math angles by deltas.

        Args:
            dJ1 (float | None): Delta for J1 in degrees. None → leave unchanged.
            dJ2 (float | None): Delta for J2 in degrees. None → leave unchanged.

        Returns:
            MoveResult: Result of calling `move_math` with the computed targets.

        Raises:
            RuntimeError: If current math angles cannot be parsed from status().
        """
        st = self.status().get("parsed")  # read current pose
        if not st or "j1_math" not in st or "j2_math" not in st:
            raise RuntimeError("Cannot read current math angles from status().")

        j1_target = st["j1_math"] + (dJ1 if dJ1 is not None else 0.0)
        j2_target = st["j2_math"] + (dJ2 if dJ2 is not None else 0.0)

        return self.move_math(j1_target, j2_target)

    # ---- NEW: change XYZ by deltas from current ----
    def move_delta_xyz(self, dx: float | None = None, dy: float | None = None, dz: float | None = None):
        """
        Increment current Cartesian position by deltas.

        Args:
            dx (float | None): Delta X in mm. None → leave unchanged.
            dy (float | None): Delta Y in mm. None → leave unchanged.
            dz (float | None): Delta Z in mm. None → leave unchanged.

        Returns:
            MoveResult: Result of calling `move_xyz` with the computed target.

        Raises:
            RuntimeError: If current XYZ cannot be parsed from status().
        """
        st = self.status().get("parsed")  # read current pose
        if not st or not all(k in st for k in ("x_mm", "y_mm", "z_mm")):
            raise RuntimeError("Cannot read current XYZ from status().")

        x_target = st["x_mm"] + (dx if dx is not None else 0.0)
        y_target = st["y_mm"] + (dy if dy is not None else 0.0)
        z_target = st["z_mm"] + (dz if dz is not None else 0.0)

        return self.move_xyz(x_target, y_target, z_target)


# ---------- parse your existing status prints into numbers ----------
# Regexes assume lines like:
#   "[J1] ... raw_deg=-12.3 ... math_deg=45.6"
#   "[J2] ... raw_deg=...     ... math_deg=..."
#   "[EE] x=123.4 mm y=56.7 mm z=-8.9 mm"
_J1 = re.compile(r"\[J1\].*raw_deg=([-\d\.]+).*math_deg=([-\d\.]+)", re.IGNORECASE)
_J2 = re.compile(r"\[J2\].*raw_deg=([-\d\.]+).*math_deg=([-\d\.]+)", re.IGNORECASE)
_EE = re.compile(r"\[EE\]\s*x=([-\d\.]+)\s*mm\s*y=([-\d\.]+)\s*mm\s*z=([-\d\.]+)\s*mm", re.IGNORECASE)

def parse_status(lines: List[str]) -> Optional[Dict[str, float]]:
    """
    Extract numeric fields from the sketch's status printout.

    Args:
        lines (List[str]): Raw lines returned by `Arm2D.status()["reply"]` or
                           any command that prints the same status format.

    Returns:
        Optional[Dict[str, float]]: Dict with any of the keys
            {"j1_raw","j1_math","j2_raw","j2_math","x_mm","y_mm","z_mm"} if
            parsing succeeded for those fields; None if nothing matched.
    """
    data: Dict[str, float] = {}
    for ln in lines:
        m = _J1.search(ln)
        if m:
            data["j1_raw"]  = float(m.group(1))
            data["j1_math"] = float(m.group(2))
        m = _J2.search(ln)
        if m:
            data["j2_raw"]  = float(m.group(1))
            data["j2_math"] = float(m.group(2))
        m = _EE.search(ln)
        if m:
            data["x_mm"] = float(m.group(1))
            data["y_mm"] = float(m.group(2))
            data["z_mm"] = float(m.group(3))
    return data or None
