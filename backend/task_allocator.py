"""
Decentralized Task Allocation via Contract Net Protocol (Auction) for PiTuPi.

Implements:
1. Dynamic task auction on AMR failure or mission completion
2. Marginal cost bidding considering distance, current path length, and battery level
3. Decentralized winner determination without central coordinator
"""

from __future__ import annotations

import math
from typing import Callable, List, Optional, Tuple
from backend.types import AuctionBid, Point, Robot


class TaskAllocator:
    """
    Decentralized Contract Net Protocol auction engine.
    """

    def __init__(self, log_callback: Optional[Callable[[str, str, str], None]] = None):
        self.log_callback = log_callback

    def log(self, tag: str, who: str, msg: str) -> None:
        if self.log_callback:
            self.log_callback(tag, who, msg)

    def calculate_bid(self, robot: Robot, task_goal: Tuple[str, int, int]) -> Optional[AuctionBid]:
        """
        Calculates an AMR's marginal cost to take over a task.
        Returns None if robot is ineligible (offline, failed, or critically low battery).
        """
        if not robot.alive or not robot.comms or robot.state == "offline":
            return None

        if robot.battery < 15.0:
            return None

        # Distance from current position to task destination
        dist = abs(round(robot.c) - task_goal[1]) + abs(round(robot.r) - task_goal[2])

        # Penalize if already carrying a long route
        workload_penalty = robot.path_remaining * 1.5

        # Penalize low battery
        battery_penalty = max(0.0, (100.0 - robot.battery) * 0.3)

        # Idle bonus (idle robots are eager to accept tasks)
        idle_discount = -10.0 if robot.state == "idle" else 0.0

        bid_cost = dist + workload_penalty + battery_penalty + idle_discount

        return AuctionBid(
            robot_id=robot.id,
            bid_cost=bid_cost,
            distance=dist,
            battery_level=robot.battery,
            is_idle=(robot.state == "idle"),
        )

    def conduct_auction(
        self,
        task_id: str,
        task_goal: Tuple[str, int, int],
        candidates: List[Robot],
        initiator_id: str = "SYSTEM",
    ) -> Optional[str]:
        """
        Conducts a decentralized auction among active peers.
        The candidate with the lowest bid cost wins.
        """
        bids: List[AuctionBid] = []
        for robot in candidates:
            if robot.id == initiator_id:
                continue
            bid = self.calculate_bid(robot, task_goal)
            if bid is not None:
                bids.append(bid)

        if not bids:
            self.log("fail", initiator_id, f"Auction for task {task_id} failed: No active peers eligible to bid.")
            return None

        # Sort bids: lowest marginal cost wins
        bids.sort(key=lambda b: b.bid_cost)
        winner = bids[0]

        self.log(
            "task", winner.robot_id,
            f"Won decentralized auction for task {task_id} -> {task_goal[0]} "
            f"(Bid Cost: {winner.bid_cost:.1f}, dist: {winner.distance}, batt: {winner.battery_level:.0f}%)."
        )

        return winner.robot_id
