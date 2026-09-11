"""
Warehouse Configuration & Simulation Constants for PiTuPi Fleet Coordination.
"""

from __future__ import annotations
from typing import Dict, List, Tuple

# Grid Dimensions
COLS: int = 11
ROWS: int = 8
CELL_SIZE: int = 78

# Canvas Dimensions
CANVAS_W: int = 900
CANVAS_H: int = 600

# Timing & Speed Constants
TICK_MS: int = 33
DEFAULT_SIM_SPEED: float = 1.0
SLOW_TICKS: int = 40
ROBOT_BASE_SPEED: float = 0.008

# Color Palette
BG: str = "#12161B"
PANEL: str = "#191E25"
PANEL2: str = "#1E242C"
LINE: str = "#2A323C"
TEXT: str = "#E7EAEE"
MUTED: str = "#8B93A1"
AMBER: str = "#E8A23A"
CYAN: str = "#4FC3D9"
GREEN: str = "#5FBE7A"
RED: str = "#E0615C"
VIOLET: str = "#9C8CE0"

ROBOT_COLORS: Dict[str, str] = {
    "A": CYAN,
    "B": AMBER,
    "C": VIOLET,
    "D": GREEN,
    "E": RED,
}

# Physical Warehouse Shelves (col, row, width, height)
SHELVES: List[Tuple[int, int, int, int]] = [
    (2, 1, 1, 2), (2, 5, 1, 2),
    (5, 1, 1, 2), (5, 5, 1, 2),
    (8, 1, 1, 2), (8, 5, 1, 2),
]

# Named Warehouse Stations
STATIONS: List[Tuple[str, int, int]] = [
    ("P1", 0, 0),
    ("P2", 10, 0),
    ("P3", 0, 7),
    ("P4", 10, 7),
    ("DOCK", 5, 7),
]

# MAPF Planning Parameters
MAX_TIME_HORIZON: int = 64
WAIT_STEP_COST: float = 1.2
CONGESTION_DECAY: float = 0.96
CONGESTION_WEIGHT: float = 0.8
CORNER_RISK_PENALTY: float = 0.3
COMMS_DEADZONE_PENALTY: float = 2.0
