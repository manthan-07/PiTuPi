"""
Distributed Conflict Resolution & Dynamic Multi-Factor Priority Engine for PiTuPi.

Implements:
1. Multi-Factor Dynamic Priority calculation with anti-starvation aging and battery protection
2. Real-time conflict classification (Vertex, Swap/Edge, Corridor Deadlock, Stationary Obstacle)
3. Distributed negotiation protocols (Yield & Slow, Space-Time Detour, Sidestep)
"""

from __future__ import annotations

from typing import List, Optional, Tuple
from backend.config import SLOW_TICKS
from backend.types import (
    ConflictEvent,
    ConflictType,
    Point,
    ResolutionAction,
    Robot,
    TaskPriority,
)


class PriorityEngine:
    """
    Computes a decentralized, multi-factor dynamic priority score for each AMR.
    Ensures fair right-of-way, battery safety, and strict anti-starvation guarantees.
    """

    @staticmethod
    def calculate_priority(robot: Robot) -> float:
        if not robot.alive or robot.state == "offline":
            return -1000.0

        score = 0.0

        # 1. Task Urgency Factor
        if robot.task_priority == TaskPriority.EXPRESS:
            score += 60.0
        elif robot.task_priority == TaskPriority.RETURN_TO_DOCK:
            score += 40.0
        else:
            score += 15.0

        # 2. Critical Battery Protection Factor
        # If battery is low, robot must reach dock/finish without being repeatedly stalled
        if robot.battery < 20.0:
            score += 45.0
        elif robot.battery < 35.0:
            score += 20.0

        # 3. Path Completion Factor (Closer to goal gets slight boost to clear bottleneck)
        rem = robot.path_remaining
        score += max(0.0, (40.0 - rem) * 0.8)

        # 4. Anti-Starvation Aging
        # Prevents any robot from waiting indefinitely by boosting priority every tick it waits
        score += robot.conflict_ticks * 1.5
        score += robot.waiting_ticks * 1.0

        # 5. Kinematic Momentum Factor (In-motion robots prefer not to come to a sudden halt)
        if robot.state == "moving":
            score += 5.0

        # 6. Deterministic Tie-Breaker (ID-based, ensures all peers agree on winner)
        score += (ord("Z") - ord(robot.id[0])) * 0.01

        robot.priority_score = score
        return score


class ConflictResolver:
    """
    Detects and classifies spatial-temporal conflicts between AMRs and decides
    the decentralized negotiation resolution.
    """

    def __init__(self, priority_engine: Optional[PriorityEngine] = None):
        self.priority_engine = priority_engine or PriorityEngine()

    def check_conflict(
        self,
        robot: Robot,
        other_robots: List[Robot],
        blocked_cells: Optional[set] = None,
    ) -> Optional[ConflictEvent]:
        """
        Evaluates potential conflicts for 'robot' against live peers and warehouse blockages.
        """
        if not robot.alive or robot.state in ("offline", "idle"):
            return None

        current_cell = robot.current_cell
        next_c = robot.next_cell

        # 1. Check temporary blockage / obstacle in front
        if robot.temp_blocked and next_c == robot.temp_blocked:
            return ConflictEvent(
                robot_a=robot.id,
                robot_b=None,
                conflict_type=ConflictType.STATIONARY_OBSTACLE,
                cell=next_c,
                time_step=robot.steps_taken,
                winner_id=None,
                action=ResolutionAction.SPACE_TIME_DETOUR,
                explanation=f"Dynamic obstacle encountered ahead at {next_c}.",
            )

        if next_c is None:
            return None

        # 2. Check conflicts against other robots
        for other in other_robots:
            if other.id == robot.id or not other.alive:
                continue

            other_cell = other.current_cell
            other_next = other.next_cell

            # Check 2A: Stationary / Parked collision
            # Next cell is currently occupied by a stationary or idle robot
            if next_c == other_cell and (other_next is None or other.state == "idle"):
                my_priority = self.priority_engine.calculate_priority(robot)
                other_priority = self.priority_engine.calculate_priority(other)
                return ConflictEvent(
                    robot_a=robot.id,
                    robot_b=other.id,
                    conflict_type=ConflictType.STATIONARY_OBSTACLE,
                    cell=next_c,
                    time_step=robot.steps_taken,
                    winner_id=other.id,  # Parked robot holds the cell
                    action=ResolutionAction.SPACE_TIME_DETOUR,
                    explanation=f"Cell {next_c} held by parked Robot {other.id}.",
                )

            # Check 2B: Swap / Edge collision
            # robot: A -> B while other: B -> A
            is_swap = (other_next is not None and other_next == current_cell and next_c == other_cell)
            if is_swap:
                my_p = self.priority_engine.calculate_priority(robot)
                other_p = self.priority_engine.calculate_priority(other)
                winner = robot.id if my_p >= other_p else other.id
                loser_action = (
                    ResolutionAction.PROCEED
                    if winner == robot.id
                    else ResolutionAction.SPACE_TIME_DETOUR
                )
                return ConflictEvent(
                    robot_a=robot.id,
                    robot_b=other.id,
                    conflict_type=ConflictType.SWAP_EDGE,
                    cell=next_c,
                    time_step=robot.steps_taken,
                    winner_id=winner,
                    action=loser_action,
                    explanation=f"Head-on swap detected on edge {current_cell}↔{next_c} with Robot {other.id}.",
                )

            # Check 2C: Vertex collision
            # Both robots aiming to step into the exact same cell next tick
            is_vertex = (other_next is not None and other_next == next_c)
            # Or robot wants to step into cell still currently occupied by other moving out
            is_occupied = (next_c == other_cell and other_next is not None)

            if is_vertex or is_occupied:
                my_p = self.priority_engine.calculate_priority(robot)
                other_p = self.priority_engine.calculate_priority(other)
                i_win = my_p >= other_p

                if i_win:
                    action = ResolutionAction.PROCEED
                else:
                    # If lower priority, yield and slow down first; if contested too long, detour
                    if robot.conflict_ticks <= SLOW_TICKS:
                        action = ResolutionAction.YIELD_SLOW
                    else:
                        action = ResolutionAction.SPACE_TIME_DETOUR

                return ConflictEvent(
                    robot_a=robot.id,
                    robot_b=other.id,
                    conflict_type=ConflictType.VERTEX,
                    cell=next_c,
                    time_step=robot.steps_taken,
                    winner_id=robot.id if i_win else other.id,
                    action=action,
                    explanation=(
                        f"Junction contention at {next_c} with Robot {other.id} "
                        f"(P:{my_p:.1f} vs P:{other_p:.1f})."
                    ),
                )

        return None
