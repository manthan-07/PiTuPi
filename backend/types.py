"""
Shared Data Types and Structures for PiTuPi Decentralized AMR Fleet.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

Point = Tuple[int, int]
SpaceTimePoint = Tuple[int, int, int]  # (c, r, t)


class TaskPriority(Enum):
    NORMAL = 1
    EXPRESS = 2
    RETURN_TO_DOCK = 3


class ConflictType(Enum):
    NONE = 0
    VERTEX = 1             # Both agents contest the exact same cell at time t
    SWAP_EDGE = 2          # Agents cross paths along the same edge in opposite directions (u->v vs v->u)
    CORRIDOR_HEAD_ON = 3   # Agents face each other in a narrow 1-wide aisle
    STATIONARY_OBSTACLE = 4 # Target cell occupied by a stationary or disabled robot / live obstacle


class ResolutionAction(Enum):
    PROCEED = "proceed"
    YIELD_SLOW = "yield_slow"
    WAIT_STEP = "wait_step"
    SPACE_TIME_DETOUR = "space_time_detour"
    SIDESTEP = "sidestep"
    STOP_BLOCKED = "stop_blocked"


@dataclass
class Task:
    task_id: str
    station_name: str
    target_pos: Point
    priority: TaskPriority = TaskPriority.NORMAL
    assigned_to: Optional[str] = None
    completed: bool = False


@dataclass
class AuctionBid:
    robot_id: str
    bid_cost: float
    distance: int
    battery_level: float
    is_idle: bool


@dataclass
class ConflictEvent:
    robot_a: str
    robot_b: Optional[str]
    conflict_type: ConflictType
    cell: Point
    time_step: int
    winner_id: Optional[str]
    action: ResolutionAction
    explanation: str


@dataclass
class Robot:
    id: str
    c: float
    r: float
    battery: float
    goal: Tuple[str, int, int]
    path: Optional[List[Point]]
    path_idx: int = 0
    progress: float = 0.0
    state: str = "moving"
    comms: bool = True
    alive: bool = True
    task_id: str = "t000"
    speed: float = 0.008
    blocked_ticks: int = 0
    steps_taken: int = 0
    conflict_ticks: int = 0
    waiting_ticks: int = 0
    temp_blocked: Optional[Point] = None
    idle_until: float = 0.0
    priority_score: float = 0.0
    task_priority: TaskPriority = TaskPriority.NORMAL

    @property
    def current_cell(self) -> Point:
        return (round(self.c), round(self.r))

    @property
    def next_cell(self) -> Optional[Point]:
        if not self.path or self.path_idx >= len(self.path) - 1:
            return None
        return self.path[self.path_idx + 1]

    @property
    def path_remaining(self) -> int:
        if not self.path:
            return 999
        return max(0, len(self.path) - self.path_idx)
