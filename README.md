# PiTuPi - Self Healing Decentralised AMR Fleet

> **Smart India Hackathon 2026 Project**

## Overview

This project aims to develop a **decentralized fleet coordination system for Autonomous Mobile Robots (AMRs)** operating in smart warehouses.

Unlike traditional centralized fleet management systems, where a central server makes decisions for all robots, our approach enables AMRs to **communicate directly with each other and make local decisions** for navigation, conflict resolution, and dynamic replanning.

The primary goal is to build a system that is **safe, efficient, lightweight, and resilient** even when communication with a central system is unavailable.

---

## Key Features

* 🤖 **Multi-AMR Coordination** — Support for 3+ autonomous robots.
* 📡 **Peer-to-Peer Communication** — Robots share relevant state and path information locally.
* 🗺️ **MAPF-based Planning** — Multi-Agent Path Finding for coordinated navigation.
* ⚡ **Cost-Aware Routing** — Considers travel time, congestion, collision risk, and communication reliability.
* 🔄 **Dynamic Replanning** — Robots adapt to obstacles and changing conditions.
* 🤝 **Distributed Conflict Resolution** — Local coordination at intersections, choke points, and shared aisles.
* 🔧 **Task Reallocation** — Reassign tasks when a robot becomes unavailable.
* 🛡️ **Fault Tolerance** — Graceful behavior during communication loss and robot failures.
* 📊 **Fleet Dashboard** — Visualize robot positions, paths, tasks, obstacles, and system events.

---

## System Architecture

The project is divided into two main planes:

```text
        AUTONOMOUS COORDINATION
                  │
     ┌────────────┼────────────┐
     │            │            │
    AMR 1  ←───► AMR 2  ←───► AMR 3
     │            │            │
     └────────────┼────────────┘
                  │
        Local Planning & MAPF
                  │
             Replanning
                  │
             Task Execution


          MONITORING PLANE
                  │
              Dashboard
                  │
          Metrics & Visualization
```

The **coordination layer remains independent of the dashboard**, ensuring that the dashboard does not become a single point of failure.

---

## Planned Technology

The initial prototype will be primarily **simulation-based**, with potential deployment on lightweight edge hardware such as Raspberry Pi or Jetson-class devices.

Possible technologies include:

* Python
* ROS 2 / DDS
* MAPF algorithms
* Lightweight P2P communication
* Warehouse simulation environment
* Web-based monitoring dashboard

The final technology choices will be determined during implementation and testing.

---

## Evaluation

The proposed system will be compared against a traditional **stop-and-wait** coordination strategy.

Key metrics will include:

* Total task completion time
* Waiting time
* Number of collisions
* Number of deadlocks
* Fleet throughput
* Replanning time
* Communication overhead
* Robot failure recovery time
* CPU/RAM usage

### Target

**0 inter-robot collisions** in tested scenarios and a target of **≥20% reduction in total task completion time** compared with the stop-and-wait baseline.

These are experimental targets and will be validated during development.

---

## Vision

> **Enable a fleet of AMRs to coordinate safely and efficiently through local intelligence and peer-to-peer communication, without depending on a centralized controller.**

This project is being developed as part of **Smart India Hackathon 2026**.
