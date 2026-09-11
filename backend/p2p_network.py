"""
Decentralized P2P Mesh Communication Simulation for PiTuPi.

Simulates ad-hoc peer-to-peer messaging between autonomous mobile robots without
a central coordination server. Handles gossip-based obstacle broadcasts, heartbeat
liveness checks, and comms dead-zones.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional, Set
from backend.types import Point, Robot


class P2PNetwork:
    """
    Virtual P2P Mesh Network routing messages between AMR nodes.
    Supports communication dropouts, dead-zones, and gossip dissemination.
    """

    def __init__(self, log_callback: Optional[Callable[[str, str, str], None]] = None):
        self.log_callback = log_callback
        # Track active peer nodes: robot_id -> bool (is_online)
        self.peer_status: Dict[str, bool] = {}
        # Known obstacles discovered by fleet: cell -> set of aware robot_ids
        self.known_obstacles: Dict[Point, Set[str]] = {}

    def log(self, tag: str, who: str, msg: str) -> None:
        if self.log_callback:
            self.log_callback(tag, who, msg)

    def register_robot(self, robot_id: str) -> None:
        self.peer_status[robot_id] = True

    def set_comms_status(self, robot_id: str, enabled: bool) -> None:
        self.peer_status[robot_id] = enabled

    def is_comms_active(self, robot_id: str) -> bool:
        return self.peer_status.get(robot_id, False)

    def broadcast_obstacle(
        self, sender_id: str, obstacle: Point, active_robots: List[Robot]
    ) -> List[str]:
        """
        Gossip-based broadcast: sender informs all mesh-reachable peers of a new obstacle.
        Returns list of peer IDs that received and acknowledged the alert.
        """
        if not self.is_comms_active(sender_id):
            self.log("comm", sender_id, f"Comms offline — unable to broadcast obstacle at {obstacle}.")
            return []

        notified_peers: List[str] = []
        if obstacle not in self.known_obstacles:
            self.known_obstacles[obstacle] = set()
        self.known_obstacles[obstacle].add(sender_id)

        for peer in active_robots:
            if peer.id == sender_id or not peer.alive:
                continue

            # Peer must also have working comms to receive the broadcast
            if self.is_comms_active(peer.id):
                if sender_id not in self.known_obstacles[obstacle] or peer.id not in self.known_obstacles[obstacle]:
                    self.known_obstacles[obstacle].add(peer.id)
                    notified_peers.append(peer.id)
                    self.log(
                        "comm", peer.id,
                        f"Received P2P obstacle notice at {obstacle} from {sender_id} — updating local map."
                    )

        return notified_peers

    def broadcast_path_reservation(
        self, sender_id: str, path_cells: List[Point], active_robots: List[Robot]
    ) -> None:
        """
        Informs peers of a newly claimed space-time corridor.
        """
        if not self.is_comms_active(sender_id):
            return

        # Peers update their decentralized reservation tables
        # (simulated directly in FleetModel's reservation table)
