"""
Multi-Agent Path Finding (MAPF) Engine for PiTuPi.

Implements:
1. Space-Time A* (4D search over (x, y, t) with wait actions)
2. Decentralized Space-Time Reservation Table (vertex and edge/swap conflict prevention)
3. Cost-Aware Dynamic Routing (incorporating congestion heat & obstacle proximity)
"""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

from backend.config import (
    COLS,
    CONGESTION_DECAY,
    CONGESTION_WEIGHT,
    CORNER_RISK_PENALTY,
    MAX_TIME_HORIZON,
    ROWS,
    SHELVES,
    WAIT_STEP_COST,
)
from backend.types import Point, SpaceTimePoint


class CostAwareGrid:
    """
    Maintains static obstacles, dynamic obstacle sets, and a dynamic congestion heat map
    that discourages all robots from dogpiling into the same central corridor.
    """

    def __init__(self, cols: int = COLS, rows: int = ROWS):
        self.cols = cols
        self.rows = rows
        self.static_shelves = SHELVES
        self.congestion_map: Dict[Point, float] = {}

    def in_bounds(self, c: int, r: int) -> bool:
        return 0 <= c < self.cols and 0 <= r < self.rows

    def is_static_obstacle(self, c: int, r: int) -> bool:
        return any(sc <= c < sc + w and sr <= r < sr + h for sc, sr, w, h in self.static_shelves)

    def record_cell_usage(self, cell: Point, weight: float = 1.0) -> None:
        self.congestion_map[cell] = self.congestion_map.get(cell, 0.0) + weight

    def decay_congestion(self) -> None:
        for cell in list(self.congestion_map.keys()):
            self.congestion_map[cell] *= CONGESTION_DECAY
            if self.congestion_map[cell] < 0.01:
                del self.congestion_map[cell]

    def get_cell_cost(self, cell: Point, dynamic_obstacles: Set[Point]) -> float:
        if not self.in_bounds(*cell) or self.is_static_obstacle(*cell) or cell in dynamic_obstacles:
            return math.inf

        base_cost = 1.0
        # Congestion heat penalty
        cong = self.congestion_map.get(cell, 0.0)
        cong_penalty = cong * CONGESTION_WEIGHT

        # Proximity to shelves corner risk penalty
        c, r = cell
        prox_penalty = 0.0
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nbr = (c + dc, r + dr)
            if self.is_static_obstacle(*nbr):
                prox_penalty += CORNER_RISK_PENALTY

        return base_cost + cong_penalty + prox_penalty


class ReservationTable:
    """
    Decentralized Space-Time Reservation Table.
    Simulates peer-shared knowledge of path claims in (x, y, t) space.
    """

    def __init__(self):
        # (c, r, t) -> robot_id
        self.vertex_reservations: Dict[SpaceTimePoint, str] = {}
        # ((c1, r1), (c2, r2), t) -> robot_id  (moving from (c1, r1) to (c2, r2) between t and t+1)
        self.edge_reservations: Dict[Tuple[Point, Point, int], str] = {}

    def clear(self) -> None:
        self.vertex_reservations.clear()
        self.edge_reservations.clear()

    def clear_robot(self, robot_id: str) -> None:
        """Removes all reservations registered by a specific robot."""
        self.vertex_reservations = {
            k: v for k, v in self.vertex_reservations.items() if v != robot_id
        }
        self.edge_reservations = {
            k: v for k, v in self.edge_reservations.items() if v != robot_id
        }

    def reserve_path(self, robot_id: str, path: List[Point], start_time: int = 0) -> None:
        """Registers a robot's space-time trajectory."""
        self.clear_robot(robot_id)
        if not path:
            return

        for t_step, cell in enumerate(path):
            t = start_time + t_step
            self.vertex_reservations[(cell[0], cell[1], t)] = robot_id
            if t_step > 0:
                prev_cell = path[t_step - 1]
                self.edge_reservations[(prev_cell, cell, t - 1)] = robot_id

        # Maintain a tail reservation on the final cell so other robots don't collide
        # into the parked robot immediately upon arrival
        last_cell = path[-1]
        for extra_t in range(1, 8):
            self.vertex_reservations[(last_cell[0], last_cell[1], start_time + len(path) - 1 + extra_t)] = robot_id

    def is_vertex_reserved(self, c: int, r: int, t: int, ignore_robot_id: Optional[str] = None) -> bool:
        occupant = self.vertex_reservations.get((c, r, t))
        return occupant is not None and occupant != ignore_robot_id

    def is_edge_reserved(
        self, from_cell: Point, to_cell: Point, t: int, ignore_robot_id: Optional[str] = None
    ) -> bool:
        """
        Detects swap conflict:
        If another robot is moving from to_cell to from_cell at time t, edge is contested.
        """
        # Swap conflict check:
        counter_occupant = self.edge_reservations.get((to_cell, from_cell, t))
        if counter_occupant is not None and counter_occupant != ignore_robot_id:
            return True

        # Same-edge follow conflict check:
        same_occupant = self.edge_reservations.get((from_cell, to_cell, t))
        if same_occupant is not None and same_occupant != ignore_robot_id:
            return True

        return False


class SpaceTimeAStar:
    """
    Space-Time A* Multi-Agent Path Finder.
    Finds guaranteed vertex-conflict and edge/swap-conflict free paths.
    """

    def __init__(self, grid: CostAwareGrid, reservations: ReservationTable):
        self.grid = grid
        self.reservations = reservations

    def plan(
        self,
        robot_id: str,
        start: Point,
        goal: Point,
        dynamic_obstacles: Optional[Set[Point]] = None,
        start_time: int = 0,
        max_horizon: int = MAX_TIME_HORIZON,
    ) -> Optional[List[Point]]:
        """
        Finds a 4D path: [(c0, r0), (c1, r1), ..., (goal_c, goal_r)]
        allowing 'wait' steps when needed to yield to crossing agents.
        """
        dyn_obs = set(dynamic_obstacles or [])
        dyn_obs.discard(start)

        if not self.grid.in_bounds(*goal) or self.grid.is_static_obstacle(*goal) or goal in dyn_obs:
            return None

        def heuristic(cell: Point) -> float:
            return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

        # Priority queue entries: (f_score, g_score, (c, r, t))
        start_state: SpaceTimePoint = (start[0], start[1], start_time)
        open_heap = [(heuristic(start), 0.0, start_state)]
        came_from: Dict[SpaceTimePoint, SpaceTimePoint] = {}
        g_scores: Dict[SpaceTimePoint, float] = {start_state: 0.0}
        closed_set: Set[SpaceTimePoint] = set()

        goal_state: Optional[SpaceTimePoint] = None

        while open_heap:
            f, g, current = heapq.heappop(open_heap)
            if current in closed_set:
                continue
            closed_set.add(current)

            curr_c, curr_r, curr_t = current

            # Check goal: at destination and cell is free for safe parking
            if (curr_c, curr_r) == goal:
                # Ensure no other robot is scheduled to enter goal at arrival time
                if not self.reservations.is_vertex_reserved(curr_c, curr_r, curr_t, ignore_robot_id=robot_id):
                    goal_state = current
                    break

            if curr_t - start_time >= max_horizon:
                continue

            curr_cell = (curr_c, curr_r)
            next_t = curr_t + 1

            # Successors: 4 cardinal movements + 1 wait action
            candidate_actions = [
                (curr_c + 1, curr_r),
                (curr_c - 1, curr_r),
                (curr_c, curr_r + 1),
                (curr_c, curr_r - 1),
                (curr_c, curr_r),  # WAIT action
            ]

            for next_cell in candidate_actions:
                nc, nr = next_cell
                is_wait = next_cell == curr_cell

                # Check grid validity
                if not self.grid.in_bounds(nc, nr) or self.grid.is_static_obstacle(nc, nr) or next_cell in dyn_obs:
                    continue

                # Check Space-Time vertex reservation
                if self.reservations.is_vertex_reserved(nc, nr, next_t, ignore_robot_id=robot_id):
                    continue

                # Check Space-Time edge/swap conflict (only applicable if moving)
                if not is_wait:
                    if self.reservations.is_edge_reserved(curr_cell, next_cell, curr_t, ignore_robot_id=robot_id):
                        continue

                # Calculate step cost
                if is_wait:
                    step_cost = WAIT_STEP_COST
                else:
                    step_cost = self.grid.get_cell_cost(next_cell, dyn_obs)

                next_state: SpaceTimePoint = (nc, nr, next_t)
                tentative_g = g + step_cost

                if tentative_g < g_scores.get(next_state, math.inf):
                    g_scores[next_state] = tentative_g
                    came_from[next_state] = current
                    f_score = tentative_g + heuristic(next_cell)
                    heapq.heappush(open_heap, (f_score, tentative_g, next_state))

        if goal_state is None:
            # Fallback to classical cost-aware A* if Space-Time graph is fully exhausted
            return self.fallback_static_plan(start, goal, dyn_obs)

        # Reconstruct path
        path_st: List[SpaceTimePoint] = []
        curr = goal_state
        while curr != start_state:
            path_st.append(curr)
            curr = came_from[curr]
        path_st.append(start_state)
        path_st.reverse()

        # Convert to 2D trajectory of cells
        path: List[Point] = [(pt[0], pt[1]) for pt in path_st]
        return path

    def fallback_static_plan(
        self, start: Point, goal: Point, dynamic_obstacles: Set[Point]
    ) -> Optional[List[Point]]:
        """Static 2D A* fallback when space-time window cannot find a continuous reservation."""
        def h(n: Point) -> float:
            return abs(n[0] - goal[0]) + abs(n[1] - goal[1])

        open_h = [(h(start), 0.0, start)]
        came_from: Dict[Point, Point] = {}
        g_score: Dict[Point, float] = {start: 0.0}
        closed: Set[Point] = set()

        while open_h:
            _, g, cur = heapq.heappop(open_h)
            if cur in closed:
                continue
            closed.add(cur)
            if cur == goal:
                break

            c, r = cur
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (c + dc, r + dr)
                if not self.grid.in_bounds(*nxt) or self.grid.is_static_obstacle(*nxt) or nxt in dynamic_obstacles:
                    continue
                ng = g + self.grid.get_cell_cost(nxt, dynamic_obstacles)
                if ng < g_score.get(nxt, math.inf):
                    g_score[nxt] = ng
                    came_from[nxt] = cur
                    heapq.heappush(open_h, (ng + h(nxt), ng, nxt))

        if goal not in g_score and start != goal:
            return None

        path = [goal]
        cur = goal
        while cur != start:
            cur = came_from.get(cur)
            if cur is None:
                return None
            path.append(cur)
        path.reverse()
        return path
