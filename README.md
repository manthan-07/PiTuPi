# PiTuPi — Self-Healing Decentralised AMR Fleet

> Smart India Hackathon 2026 

## 1. Overview

PiTuPi is a self-healing, decentralized fleet coordination system for Autonomous Mobile Robots (AMRs) operating in dynamic warehouse environments. The system enables multiple robots to coordinate in real time without relying on a central controller, allowing them to plan paths, resolve local conflicts, recover from failures, and continue operation even when connectivity is degraded.

Traditional warehouse automation often depends on a central scheduler that becomes a bottleneck during failures, congestion, or communication loss. PiTuPi addresses this challenge by using peer-to-peer coordination, decentralized path reservation, dynamic priority management, and task reallocation based on local conditions.

---

## 2. Problem Statement

Modern warehouses are increasingly dependent on autonomous material handling fleets. However, centralised fleet control introduces several critical issues:

- Single point of failure during server or network disruption
- Delayed response to congestion and unforeseen obstacles
- Poor scalability when many robots compete for narrow aisles and intersections
- Inability to recover gracefully when a robot or communication link fails
- High downtime and operational inefficiency during abnormal events

A resilient warehouse AMR system should therefore be capable of distributed decision-making, local replanning, and autonomous recovery without full dependency on a central authority.

---

## 3. Solution

PiTuPi proposes a decentralized coordination framework in which each AMR acts as an intelligent node in a mesh network. The fleet jointly manages:

- path planning using space-time reservation techniques,
- conflict prevention at intersections and shared corridors,
- task reassignment through distributed auction-based bidding,
- communication-aware behaviour under link failures,
- dynamic replanning when obstacles or robot failures appear,
- local fault recovery and priority escalation for battery-critical or blocked robots.

The system is designed to be fast, lightweight, and simulation-compatible for further real-world deployment on edge robots or industrial gateways.

---

## 4. Key Features

- Multi-AMR coordination for warehouse navigation and task completion
- Peer-to-peer communication network with simulated comms degradation
- Space-Time A* path planning with reservation-table conflict avoidance
- Cost-aware routing considering congestion, obstacle proximity, and dead-zone risk
- Dynamic obstacle detection and local replanning
- Priority-based conflict resolution to prevent head-on collisions and deadlocks
- Auction-based task allocation using Contract Net Protocol
- Self-healing behaviour for robot failure and task reassignment
- Visual monitoring dashboard for fleet status, path traces, obstacles, and metrics

---

## 5. System Architecture

```text
                         ┌──────────────────────────────────────────────┐
                         │            Decentralized Fleet Layer          │
                         │                                              │
                         │  AMR A  ◄──►  AMR B  ◄──►  AMR C  ◄──►  AMR D │
                         │      │          │          │          │       │
                         │      └──────────┴──────────┴──────────┴───────┘
                         │            Peer-to-Peer Mesh / Gossip          │
                         └──────────────────────────────────────────────┘
                                                  │
                                                  │
                      ┌───────────────────────┴────────────────────────┐
                      │                                                │
                      ▼                                                ▼
      ┌──────────────────────────────┐        ┌──────────────────────────────┐
      │   MAPF Planner & Pathing     │        │    Conflict Resolver          │
      │   - Space-Time A*             │        │    - Priority scoring         │
      │   - Reservation table         │        │    - Deadlock avoidance       │
      │   - Congestion-aware routing  │        │    - Collision prevention     │
      └──────────────────────────────┘        └──────────────────────────────┘
                      │                                                │
                      │                                                │
                      └──────────────────────┬─────────────────────────┘
                                             │
                                             ▼
                        ┌────────────────────────────────┐
                        │ Fleet Simulation & Control    │
                        │ - Robot states               │
                        │ - Task assignment            │
                        │ - Dynamic replanning         │
                        │ - Failure handling           │
                        │ - Metrics & logs             │
                        └────────────────────────────────┘
                                             │
                                             │
                                             ▼
                        ┌────────────────────────────────┐
                        │   Task Allocation Layer       │
                        │   - Auction-based dispatch    │
                        │   - Battery-aware bidding     │
                        │   - Reassignment on failure   │
                        └────────────────────────────────┘
                                             │
                                             ▼
                        ┌────────────────────────────────┐
                        │   Monitoring & Visualization  │
                        │   Dashboard | Alerts | Logs    │
                        │   (Independent of control)     │
                        └────────────────────────────────┘
```

The coordination layer is independent from the visualization layer, which prevents the dashboard from becoming a central point of failure.

---

## 6. Core Functional Modules

### 6.1 Fleet Model
The main simulation engine maintains robot states, reservations, dynamic obstacles, task assignments, and overall fleet metrics. It simulates motion, task completion, and failure scenarios in a warehouse grid.

### 6.2 MAPF Planner
A cost-aware Space-Time A* planner generates collision-free paths in 4D space by considering:

- static warehouse shelves
- dynamic obstacles
- reservation conflicts
- congestion penalties
- local wait behaviour to avoid deadlock and swap conflicts

### 6.3 Reservation Table
A decentralized reservation system prevents vertex and edge conflicts by reserving both positions and transitions across time.

### 6.4 Conflict Resolver
A priority engine ranks robots based on urgency such as battery protection, return-to-dock conditions, waiting time, and anti-starvation ageing. This allows the fleet to resolve access contention safely and fairly.

### 6.5 P2P Network Simulation
Robots exchange local information using a gossip-style communication model. If a robot loses connectivity, its neighbours can still continue coordination based on locally known state and shared reservations.

### 6.6 Task Allocation
The task allocator uses decentralized auctioning to reassign workload when a robot fails, becomes blocked, or is low on battery. This improves resilience and system throughput.

---

## 7. Technology Stack

- Python 3
- Object-oriented simulation engine
- A* and Space-Time MAPF planning
- Reservation-table coordination logic
- Decentralized communication modelling
- Tkinter-based monitoring dashboard
- Backend validation and regression suite

---

## 8. Implementation Highlights

This repository includes a working simulation and validation suite for the following core behaviours:

- robot path generation without collisions
- dynamic replanning around blocked cells
- conflict avoidance in intersections and corridor crossings
- low-battery task prioritization and return-to-dock logic
- anti-starvation priority aging
- contract-net task auction winner selection
- robot communication loss handling
- manual obstacle injection and failure simulation

---

## 9. Evaluation Metrics

The system is evaluated against a conventional stop-and-wait coordination baseline using metrics such as:

- total task completion time
- collision count
- deadlock occurrence
- reroute frequency
- task reassignment efficiency
- communication loss resilience
- recovery time after robot failure
- fleet throughput and average waiting time

### Target Outcomes

- Zero inter-robot collisions in tested scenarios
- Low event-driven reroute overhead
- Stable operation even under simulated comms loss
- Improved resilience versus centralised stop-and-wait baselines

---

## 10. Validation and Verification

The backend has been validated with automated checks. Verified execution command:

```bash
python test_backend.py
```

Result summary:

- Basic Space-Time A* path planning: PASS
- Vertex conflict avoidance: PASS
- Edge/swap collision prevention: PASS
- Dynamic priority engine: PASS
- Contract Net auction: PASS
- 150-tick fleet simulation with 0 collisions: PASS
- Manual obstacle placement: PASS
- Robot failure injection and recovery handling: PASS
- Comms drop and restore: PASS

This confirms that the system is operational in simulation and suitable for final submission and further extension.

---

## 11. Run Instructions

Clone the project and run the backend validation suite from the project root:

```bash
python test_backend.py
```

To explore the dashboard or simulation interface, open the project files in the repository and launch the dashboard module as configured in the project workflow.

---

## 12. Impact and Relevance

PiTuPi is relevant to the growing need for resilient autonomous logistics in warehouses, factories, and smart industrial campuses. The project demonstrates that robust fleet coordination can be achieved without central control by combining local intelligence, communication awareness, and distributed planning.

This makes the system particularly suitable for:

- smart warehouse automation,
- industrial intralogistics,
- autonomous material handling,
- resilient multi-robot logistics operations,
- future deployment in connected edge-computing environments.

---

## 13. Vision

> To enable autonomous mobile robot fleets to coordinate safely, efficiently, and self-sufficiently in dynamic industrial settings without dependence on a central failure-prone controller.

PiTuPi represents a practical, simulation-backed prototype for the next generation of decentralized warehouse autonomy and is aligned with the goals of Smart India Hackathon 2026.

