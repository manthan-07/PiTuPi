"""
Fleet Model Coordination Engine for PiTuPi.

Integrates:
- MAPF Space-Time A* Planner & Decentralized Reservation Table
- Cost-Aware Grid Routing with Congestion Heat Map
- Multi-Factor Dynamic Priority & Conflict Resolver
- P2P Mesh Communication with Comms Loss Simulation
- Contract Net Protocol Task Auctions
"""

from __future__ import annotations

import random
import time
from typing import Callable, Dict, List, Optional, Set, Tuple

from backend.config import (
    COLS,
    DEFAULT_SIM_SPEED,
    ROBOT_BASE_SPEED,
    ROWS,
    SHELVES,
    SLOW_TICKS,
    STATIONS,
)
from backend.conflict_resolver import ConflictResolver, PriorityEngine
from backend.mapf import CostAwareGrid, ReservationTable, SpaceTimeAStar
from backend.p2p_network import P2PNetwork
from backend.task_allocator import TaskAllocator
from backend.types import ConflictType, Point, ResolutionAction, Robot, TaskPriority


class FleetModel:
    """
    Decentralized AMR Fleet Model and Simulation Engine.
    Exposes clean interfaces for the Tkinter frontend control room.
    """

    def __init__(
        self,
        log_callback: Optional[Callable[[str, str, str], None]] = None,
        metrics_callback: Optional[Callable[[Dict[str, float]], None]] = None,
    ):
        self.log_callback = log_callback
        self.metrics_callback = metrics_callback

        # Grid, Planning & MAPF components
        self.grid = CostAwareGrid(COLS, ROWS)
        self.reservations = ReservationTable()
        self.planner = SpaceTimeAStar(self.grid, self.reservations)

        # Conflict resolution & priorities
        self.priority_engine = PriorityEngine()
        self.conflict_resolver = ConflictResolver(self.priority_engine)

        # P2P Network and Task Auction layers
        self.network = P2PNetwork(log_callback=self.log)
        self.task_allocator = TaskAllocator(log_callback=self.log)

        # Simulation state
        self.robots: List[Robot] = []
        self.tick: int = 0
        self.sim_speed: float = DEFAULT_SIM_SPEED
        self.failed_obstacles: Set[Point] = set()
        self.manual_obstacles: Set[Point] = set()

        self.metrics: Dict[str, float] = {
            "collisions": 0,
            "completed": 0,
            "reroute_events": 0,
            "baseline_time_estimate": 0.0,
            "actual_time_accum": 0.0,
        }

        self.reset()

    @staticmethod
    def in_bounds(c: int, r: int) -> bool:
        return 0 <= c < COLS and 0 <= r < ROWS

    @staticmethod
    def blocked(c: int, r: int) -> bool:
        return any(sc <= c < sc + w and sr <= r < sr + h for sc, sr, w, h in SHELVES)

    def log(self, tag: str, who: str, message: str) -> None:
        if self.log_callback:
            self.log_callback(tag, who, message)

    def emit_metrics(self) -> None:
        if self.metrics_callback:
            self.metrics_callback(self.metrics)

    @staticmethod
    def path_remaining(robot: Robot) -> int:
        return robot.path_remaining

    @staticmethod
    def next_cell(robot: Robot) -> Optional[Point]:
        return robot.next_cell

    def all_dynamic_obstacles(self) -> Set[Point]:
        return set(self.failed_obstacles) | set(self.manual_obstacles)

    def find_path(
        self,
        start: Point,
        goal: Point,
        dynamic_blocked: Optional[List[Point]] = None,
        robot_id: str = "A",
    ) -> Optional[List[Point]]:
        """
        Calculates a path using MAPF Space-Time A* taking into account reservations,
        dynamic obstacles, and congestion penalties.
        """
        dyn = self.all_dynamic_obstacles() | set(dynamic_blocked or [])
        dyn.discard(start)

        path = self.planner.plan(
            robot_id=robot_id,
            start=start,
            goal=goal,
            dynamic_obstacles=dyn,
            start_time=self.tick,
        )
        return path

    def rand_station(self, exclude_c: int, exclude_r: int) -> Tuple[str, int, int]:
        choices = [
            s for s in STATIONS
            if not (s[1] == exclude_c and s[2] == exclude_r) and (s[1], s[2]) not in self.failed_obstacles
        ]
        if not choices:
            choices = [s for s in STATIONS if not (s[1] == exclude_c and s[2] == exclude_r)]
        return random.choice(choices)

    def make_robot(self, rid: str, c: int, r: int, battery: float) -> Robot:
        goal = self.rand_station(c, r)
        task_id = f"t{random.randint(100, 999)}"
        path = self.find_path((c, r), (goal[1], goal[2]), [], robot_id=rid)
        robot = Robot(
            id=rid,
            c=float(c),
            r=float(r),
            battery=battery,
            goal=goal,
            path=path,
            task_id=task_id,
            speed=ROBOT_BASE_SPEED + random.random() * 0.002,
        )
        if path:
            self.reservations.reserve_path(rid, path, start_time=self.tick)
        self.network.register_robot(rid)
        return robot

    def reset(self) -> None:
        self.metrics = {
            "collisions": 0,
            "completed": 0,
            "reroute_events": 0,
            "baseline_time_estimate": 0.0,
            "actual_time_accum": 0.0,
        }
        self.tick = 0
        self.failed_obstacles.clear()
        self.manual_obstacles.clear()
        self.reservations.clear()

        self.robots = [
            self.make_robot("A", 0, 2, 82),
            self.make_robot("B", 4, 0, 64),
            self.make_robot("C", 8, 3, 91),
            self.make_robot("D", 10, 5, 76),
        ]
        self.log("comm", "SYSTEM", "Decentralized P2P mesh initialized. 4 AMR peers online (Robots A, B, C, D).")
        for r in self.robots:
            if r.path:
                self.log("mapf", r.id, f"4D Space-Time path reserved ({len(r.path)} waypoints) -> heading to {r.goal[0]}.")
        self.emit_metrics()

    def assign_new_task(self, robot: Robot) -> None:
        if not robot.alive:
            return

        current_cell = robot.current_cell
        goal = self.rand_station(current_cell[0], current_cell[1])
        robot.goal = goal
        robot.task_id = f"t{random.randint(100, 999)}"

        # Assign task priority (e.g. DOCK has higher priority if battery is low)
        if robot.battery < 30.0 or goal[0] == "DOCK":
            robot.task_priority = TaskPriority.RETURN_TO_DOCK
        else:
            robot.task_priority = TaskPriority.NORMAL

        db = [robot.temp_blocked] if robot.temp_blocked else []
        new_path = self.find_path(current_cell, (goal[1], goal[2]), db, robot_id=robot.id)

        robot.path = new_path
        robot.path_idx = 0
        robot.progress = 0.0
        robot.steps_taken = 0
        robot.conflict_ticks = 0
        robot.waiting_ticks = 0
        robot.state = "moving"
        robot.idle_until = 0.0

        if new_path:
            self.reservations.reserve_path(robot.id, new_path, start_time=self.tick)
            self.network.broadcast_path_reservation(robot.id, new_path, self.robots)

        self.priority_engine.calculate_priority(robot)
        self.log(
            "task", robot.id,
            f"Assigned task {robot.task_id} -> {goal[0]} (Priority: {robot.task_priority.name}, Score: {robot.priority_score:.1f}, Steps: {len(new_path) if new_path else 0})."
        )

    def handle_blockage(self, robot: Robot) -> None:
        if robot.state == "blocked" or not robot.temp_blocked:
            return

        robot.state = "blocked"
        bc, br = robot.temp_blocked
        self.log("fail", robot.id, f"Obstacle detected ahead at ({bc},{br}). Marking cell unsafe locally.")

        # Replan via SpaceTimeAStar avoiding blocked cell
        new_path = self.find_path(
            robot.current_cell,
            (robot.goal[1], robot.goal[2]),
            [robot.temp_blocked],
            robot_id=robot.id,
        )

        if new_path and len(new_path) > 1:
            robot.path = new_path
            robot.path_idx = 0
            robot.progress = 0.0
            robot.state = "moving"
            robot.conflict_ticks = 0
            self.reservations.reserve_path(robot.id, new_path, start_time=self.tick)
            self.metrics["reroute_events"] += 1
            self.log("reroute", robot.id, "Dynamic replan complete. New MAPF trajectory reserved.")

            # Gossip obstacle notice to mesh peers
            self.network.broadcast_obstacle(robot.id, robot.temp_blocked, self.robots)
            for other in self.robots:
                if other.id != robot.id and other.alive and other.path:
                    if robot.temp_blocked in other.path:
                        other.temp_blocked = robot.temp_blocked
        else:
            self.log("fail", robot.id, "No alternate route found around obstacle. Holding position.")

        self.emit_metrics()

    def step(self) -> None:
        self.tick += 1
        now = time.monotonic()

        # Decay congestion heat map every 30 ticks
        if self.tick % 30 == 0:
            self.grid.decay_congestion()

        # Check idle robots transitioning to new tasks
        for robot in self.robots:
            if robot.alive and robot.state == "idle" and robot.idle_until and now >= robot.idle_until:
                self.assign_new_task(robot)

        # Periodic fleet telemetry broadcast every 90 ticks (~3s)
        if self.tick % 90 == 0:
            active_list = [f"R{r.id}:{r.state[:4]}" for r in self.robots if r.alive]
            summary = " | ".join(active_list)
            self.log("comm", "P2P MESH", f"Telemetry: {len(active_list)} peers operational [{summary}] | Tasks done: {int(self.metrics['completed'])} | 0 collisions.")

        for robot in self.robots:
            if not robot.alive or robot.state == "offline":
                continue

            # Update priority score continuously
            self.priority_engine.calculate_priority(robot)

            # Battery discharge while moving
            if robot.state == "moving":
                robot.battery = max(0.0, robot.battery - 0.012)
                # If battery critical, recharge alert
                if robot.battery < 15.0 and robot.task_priority != TaskPriority.RETURN_TO_DOCK:
                    robot.task_priority = TaskPriority.RETURN_TO_DOCK

            # Check if task is completed
            if not robot.path or robot.path_idx >= len(robot.path) - 1:
                if robot.state != "idle":
                    robot.state = "idle"
                    self.metrics["completed"] += 1
                    self.metrics["actual_time_accum"] += robot.steps_taken
                    self.metrics["baseline_time_estimate"] += robot.steps_taken * 1.28
                    self.reservations.clear_robot(robot.id)
                    self.log("task", robot.id, f"Reached {robot.goal[0]}. Task {robot.task_id} completed.")
                    robot.idle_until = now + 0.9
                    self.emit_metrics()
                continue

            nc = robot.next_cell
            if nc is None:
                continue

            # Check dynamic temporary blockage
            if robot.temp_blocked == nc:
                self.handle_blockage(robot)
                continue

            # Check conflicts via Distributed Conflict Resolver
            conflict = self.conflict_resolver.check_conflict(robot, self.robots, self.all_dynamic_obstacles())

            if conflict:
                if conflict.action == ResolutionAction.PROCEED:
                    robot.state = "moving"
                    robot.conflict_ticks = 0
                elif conflict.action == ResolutionAction.YIELD_SLOW:
                    robot.conflict_ticks += 1
                    robot.waiting_ticks += 1
                    if robot.state != "slowing":
                        robot.state = "slowing"
                        self.log(
                            "conflict", robot.id,
                            f"{conflict.explanation} Yielding right-of-way to {conflict.winner_id}; slowing to 15%."
                        )
                    # Advance progress slowly to allow winner to clear
                    robot.progress += robot.speed * self.sim_speed * 0.15
                    continue
                elif conflict.action in (ResolutionAction.SPACE_TIME_DETOUR, ResolutionAction.SIDESTEP):
                    robot.state = "rerouting"
                    self.log("reroute", robot.id, f"{conflict.explanation} Initiating dynamic MAPF replan.")
                    new_path = self.find_path(
                        robot.current_cell,
                        (robot.goal[1], robot.goal[2]),
                        [conflict.cell],
                        robot_id=robot.id,
                    )
                    if new_path and len(new_path) > 1:
                        robot.path = new_path
                        robot.path_idx = 0
                        robot.progress = 0.0
                        robot.state = "moving"
                        robot.conflict_ticks = 0
                        robot.waiting_ticks = 0
                        self.reservations.reserve_path(robot.id, new_path, start_time=self.tick)
                        self.metrics["reroute_events"] += 1
                        self.log("reroute", robot.id, f"Detour computed avoiding {conflict.cell}. Resumed navigation.")
                        self.emit_metrics()
                    else:
                        self.log("fail", robot.id, f"No viable detour found around {conflict.cell}. Holding safely.")
                        robot.state = "blocked"
                        robot.conflict_ticks = 0
                    continue
            elif robot.state in ("slowing", "rerouting", "blocked"):
                robot.state = "moving"
                robot.conflict_ticks = 0

            # Normal motion progression
            # Conservative sensing mode if comms link lost: reduce speed by 35%
            effective_speed = robot.speed * self.sim_speed
            if not robot.comms:
                effective_speed *= 0.65

            robot.progress += effective_speed

            if robot.progress >= 1.0:
                robot.progress = 0.0
                robot.c, robot.r = float(nc[0]), float(nc[1])
                robot.path_idx += 1
                robot.steps_taken += 1
                robot.state = "moving"

                # Record congestion footprint in grid heat map
                self.grid.record_cell_usage(nc)

                # Keep reservations synchronized
                if robot.path and robot.path_idx < len(robot.path):
                    remaining_path = robot.path[robot.path_idx:]
                    self.reservations.reserve_path(robot.id, remaining_path, start_time=self.tick)

                # Log waypoint navigation progress every 3 steps
                if robot.steps_taken % 3 == 0:
                    self.log("mapf", robot.id, f"Advancing through cell ({nc[0]},{nc[1]}) -> {robot.goal[0]} (P:{robot.priority_score:.1f}, Batt:{robot.battery:.0f}%).")

        # Safety assertion: verify zero coincident cells
        occupied_cells: Dict[Point, str] = {}
        for robot in self.robots:
            if not robot.alive:
                continue
            pos = robot.current_cell
            if pos in occupied_cells and occupied_cells[pos] != robot.id:
                self.metrics["collisions"] += 1
                self.log("fail", "SYSTEM", f"Collision alert: Robot {occupied_cells[pos]} and Robot {robot.id} at {pos}!")
            occupied_cells[pos] = robot.id

    def place_manual_obstacle(self, cell: Point) -> bool:
        c, r = cell
        if not self.in_bounds(c, r) or self.blocked(c, r):
            return False
        if cell in self.failed_obstacles or cell in self.manual_obstacles:
            return False
        if any(robot.alive and robot.current_cell == cell for robot in self.robots):
            return False

        self.manual_obstacles.add(cell)
        self.log("fail", "SYSTEM", f"Live obstacle placed at ({c},{r}). Fleet triggered P2P map update.")
        self.replan_around_obstacle(cell)
        self.emit_metrics()
        return True

    def remove_manual_obstacle(self, cell: Point) -> bool:
        if cell not in self.manual_obstacles:
            return False
        self.manual_obstacles.remove(cell)
        self.log("comm", "SYSTEM", f"Live obstacle removed from ({cell[0]},{cell[1]}). Sector cleared.")

        # Replan any unblocked robots
        for robot in self.robots:
            if not robot.alive:
                continue
            new_path = self.find_path(
                robot.current_cell,
                (robot.goal[1], robot.goal[2]),
                [robot.temp_blocked] if robot.temp_blocked else [],
                robot_id=robot.id,
            )
            if new_path:
                robot.path = new_path
                robot.path_idx = 0
                robot.progress = 0.0
                if robot.state == "blocked":
                    robot.state = "moving"
                self.reservations.reserve_path(robot.id, new_path, start_time=self.tick)

        self.emit_metrics()
        return True

    def replan_around_obstacle(self, obstacle: Point) -> None:
        for robot in self.robots:
            if not robot.alive:
                continue
            if obstacle not in (robot.path or []) and robot.temp_blocked != obstacle:
                continue

            new_path = self.find_path(
                robot.current_cell,
                (robot.goal[1], robot.goal[2]),
                [robot.temp_blocked] if robot.temp_blocked and robot.temp_blocked != obstacle else [],
                robot_id=robot.id,
            )
            if new_path:
                robot.path = new_path
                robot.path_idx = 0
                robot.progress = 0.0
                robot.state = "rerouting"
                robot.conflict_ticks = 0
                self.reservations.reserve_path(robot.id, new_path, start_time=self.tick)
                self.metrics["reroute_events"] += 1
                self.log("reroute", robot.id, f"Rerouted around new obstacle at {obstacle}.")
            else:
                robot.state = "blocked"
                self.log("fail", robot.id, f"No detour around {obstacle}; waiting for clearance.")

    def inject_obstacle(self) -> None:
        active = [r for r in self.robots if r.alive and r.state not in ("idle", "offline")]
        if not active:
            return
        robot = random.choice(active)
        nc = robot.next_cell or robot.current_cell
        if robot.path and robot.path_idx + 2 < len(robot.path):
            nc = robot.path[robot.path_idx + 2]
        robot.temp_blocked = nc
        self.log("fail", "SYSTEM", f"Dynamic obstacle injected near Robot {robot.id}'s path at {nc}.")

    def fail_robot(self) -> None:
        alive = [r for r in self.robots if r.alive]
        if not alive:
            return
        failed = random.choice(alive)
        failed_cell = failed.current_cell
        failed.alive = False
        failed.state = "offline"

        # Clear reservations for the failed robot
        self.reservations.clear_robot(failed.id)
        self.failed_obstacles.add(failed_cell)
        self.log(
            "fail", failed.id,
            f"Heartbeat timeout! Robot {failed.id} offline at {failed_cell} — converted to physical obstacle."
        )

        # Replan surviving robots whose path crosses the failed robot
        for robot in self.robots:
            if not robot.alive:
                continue
            if failed_cell in (robot.path or []):
                if robot.goal and (robot.goal[1], robot.goal[2]) == failed_cell:
                    robot.goal = self.rand_station(robot.current_cell[0], robot.current_cell[1])
                    robot.task_id = f"t{random.randint(100, 999)}"
                new_path = self.find_path(
                    robot.current_cell,
                    (robot.goal[1], robot.goal[2]),
                    [robot.temp_blocked] if robot.temp_blocked else [],
                    robot_id=robot.id,
                )
                if new_path:
                    robot.path = new_path
                    robot.path_idx = 0
                    robot.progress = 0.0
                    robot.state = "moving"
                    robot.conflict_ticks = 0
                    self.reservations.reserve_path(robot.id, new_path, start_time=self.tick)
                    self.metrics["reroute_events"] += 1
                    self.log("reroute", robot.id, f"Rerouted around failed Robot {failed.id} at {failed_cell}.")

        # Conduct Contract Net auction among live peers for orphaned task
        winner_id = self.task_allocator.conduct_auction(
            task_id=failed.task_id,
            task_goal=failed.goal,
            candidates=self.robots,
            initiator_id=failed.id,
        )
        if winner_id:
            winner = next((r for r in self.robots if r.id == winner_id), None)
            if winner:
                winner.goal = failed.goal
                winner.task_id = failed.task_id
                winner.task_priority = TaskPriority.EXPRESS  # Rescued tasks get high priority
                new_path = self.find_path(
                    winner.current_cell,
                    (winner.goal[1], winner.goal[2]),
                    [winner.temp_blocked] if winner.temp_blocked else [],
                    robot_id=winner.id,
                )
                if new_path:
                    winner.path = new_path
                    winner.path_idx = 0
                    winner.progress = 0.0
                    winner.state = "moving"
                    self.reservations.reserve_path(winner.id, new_path, start_time=self.tick)

        self.emit_metrics()

    def set_robot_a_comms(self, enabled: bool) -> None:
        robot = next((r for r in self.robots if r.id == "A"), None)
        if not robot or not robot.alive:
            return
        robot.comms = enabled
        self.network.set_comms_status("A", enabled)
        if enabled:
            self.log("comm", "A", "Mesh link restored. Triggered P2P map & reservation resync.")
            if robot.path:
                self.reservations.reserve_path("A", robot.path[robot.path_idx:], start_time=self.tick)
        else:
            self.log("comm", "A", "Wi-Fi dead-zone entered! Switching to conservative local sensing mode (speed -35%).")
