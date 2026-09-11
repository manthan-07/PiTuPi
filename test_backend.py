"""
Automated Backend Verification Suite for PiTuPi.
Tests Space-Time MAPF, Reservation Table, Conflict Resolver, P2P network, and Task Auction.
"""

from __future__ import annotations
import sys
from backend.config import COLS, ROWS, SHELVES, STATIONS
from backend.types import Robot, TaskPriority, ConflictType, ResolutionAction
from backend.mapf import CostAwareGrid, ReservationTable, SpaceTimeAStar
from backend.conflict_resolver import PriorityEngine, ConflictResolver
from backend.task_allocator import TaskAllocator
from backend.fleet_model import FleetModel


def test_mapf_planning():
    print("[TEST] Running Space-Time MAPF Planning Tests...")
    grid = CostAwareGrid(COLS, ROWS)
    reservations = ReservationTable()
    planner = SpaceTimeAStar(grid, reservations)

    # Test 1: Simple unobstructed path
    start = (0, 0)
    goal = (3, 0)
    path = planner.plan(robot_id="A", start=start, goal=goal, start_time=0)
    assert path is not None, "Failed to find basic path"
    assert path[0] == start, f"Start mismatch: {path[0]} vs {start}"
    assert path[-1] == goal, f"Goal mismatch: {path[-1]} vs {goal}"
    print("  -> Basic Space-Time A* path: PASS")

    # Test 2: Space-Time Vertex Conflict Avoidance
    # Robot B reserves cell (1, 0) at time t=1
    reservations.reserve_path("B", [(1, 0)], start_time=1)
    path_avoid = planner.plan(robot_id="A", start=start, goal=goal, start_time=0)
    assert path_avoid is not None, "Failed to find alternate path"
    # Ensure Robot A does not step into (1, 0) at time 1
    assert path_avoid[1] != (1, 0), f"Robot A stepped into reserved vertex at t=1: {path_avoid}"
    print("  -> Space-Time vertex conflict avoidance: PASS")

    # Test 3: Swap / Edge Collision Prevention
    reservations.clear()
    # Robot B moves from (1, 0) to (0, 0) between t=0 and t=1
    reservations.edge_reservations[((1, 0), (0, 0), 0)] = "B"
    reservations.vertex_reservations[(0, 0, 1)] = "B"
    path_swap = planner.plan(robot_id="A", start=(0, 0), goal=(2, 0), start_time=0)
    assert path_swap is not None, "Failed to plan around swap"
    # Robot A should either WAIT at (0,0) or take a lateral step to avoid head-on swap
    assert path_swap[1] != (1, 0), "Swap collision occurred!"
    print("  -> Space-Time edge/swap collision avoidance: PASS")


def test_priority_engine():
    print("[TEST] Running Dynamic Priority Engine Tests...")
    r_normal = Robot(
        id="A", c=0.0, r=0.0, battery=90.0, goal=("P2", 10, 0),
        path=[(0,0), (1,0), (2,0)], task_priority=TaskPriority.NORMAL
    )
    r_urgent = Robot(
        id="B", c=0.0, r=0.0, battery=15.0, goal=("DOCK", 5, 7), # Low battery
        path=[(0,0), (1,0)], task_priority=TaskPriority.RETURN_TO_DOCK
    )

    p_norm = PriorityEngine.calculate_priority(r_normal)
    p_urg = PriorityEngine.calculate_priority(r_urgent)
    assert p_urg > p_norm, f"Urgent/low battery should have higher priority ({p_urg} <= {p_norm})"
    print("  -> Battery protection priority boost: PASS")

    # Anti-starvation aging test
    r_waiting = Robot(
        id="C", c=0.0, r=0.0, battery=90.0, goal=("P2", 10, 0),
        path=[(0,0), (1,0), (2,0)], conflict_ticks=50, waiting_ticks=30
    )
    p_aged = PriorityEngine.calculate_priority(r_waiting)
    assert p_aged > p_norm + 50, f"Anti-starvation aging failed: {p_aged} vs {p_norm}"
    print("  -> Anti-starvation aging priority: PASS")


def test_contract_net_auction():
    print("[TEST] Running Contract Net Task Auction Tests...")
    allocator = TaskAllocator()
    r1 = Robot(id="A", c=0.0, r=0.0, battery=80.0, goal=("P1", 0, 0), path=[])
    r2 = Robot(id="B", c=9.0, r=0.0, battery=85.0, goal=("P2", 10, 0), path=[])
    # Target station is at (10, 0)
    task_goal = ("P2", 10, 0)

    winner = allocator.conduct_auction(
        task_id="t999",
        task_goal=task_goal,
        candidates=[r1, r2],
        initiator_id="C"
    )
    assert winner == "B", f"Expected closest robot B to win, got {winner}"
    print("  -> Contract Net auction winner determination: PASS")


def test_fleet_simulation_model():
    print("[TEST] Running Full FleetModel Simulation Tests...")
    logs = []
    metrics_history = []

    def mock_log(tag, who, msg):
        logs.append((tag, who, msg))

    def mock_metrics(m):
        metrics_history.append(dict(m))

    model = FleetModel(log_callback=mock_log, metrics_callback=mock_metrics)
    assert len(model.robots) == 4, f"Fleet should initialize with 4 robots, got {len(model.robots)}"

    # Step simulation for 150 ticks
    for _ in range(150):
        model.step()

    assert model.tick == 150, "Tick count incorrect"
    assert model.metrics["collisions"] == 0, f"Collisions detected: {model.metrics['collisions']}"
    print("  -> 150 ticks simulation with 0 collisions: PASS")

    # Test dynamic obstacle placement on an unoccupied free cell and replan
    occupied = {r.current_cell for r in model.robots}
    free_cells = [
        (c, r) for c in range(COLS) for r in range(ROWS)
        if not model.blocked(c, r) and (c, r) not in occupied
    ]
    test_cell = free_cells[0]
    placed = model.place_manual_obstacle(test_cell)
    assert placed, f"Manual obstacle placement failed on {test_cell}"
    assert test_cell in model.manual_obstacles
    print(f"  -> Manual obstacle placement on {test_cell}: PASS")

    # Test robot failure and task auction
    prev_active = sum(1 for r in model.robots if r.alive)
    model.fail_robot()
    new_active = sum(1 for r in model.robots if r.alive)
    assert new_active == prev_active - 1, "Robot failure did not reduce active count"
    assert len(model.failed_obstacles) == 1, "Failed robot not registered as obstacle"
    print("  -> Robot failure injection & obstacle conversion: PASS")

    # Run another 50 ticks to ensure stability post-failure
    for _ in range(50):
        model.step()

    # Comms loss test
    model.set_robot_a_comms(False)
    assert not model.network.is_comms_active("A"), "Comms status not updated"
    model.set_robot_a_comms(True)
    assert model.network.is_comms_active("A"), "Comms restore failed"
    print("  -> Robot A comms drop & restore: PASS")


if __name__ == "__main__":
    test_mapf_planning()
    test_priority_engine()
    test_contract_net_auction()
    test_fleet_simulation_model()
    print("\nALL BACKEND AUTOMATED TESTS PASSED SUCCESSFULLY!")
