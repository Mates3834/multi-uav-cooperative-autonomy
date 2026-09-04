# Multi-UAV Cooperative Autonomy

Generic and sanitized Python framework for studying cooperative autonomy in
multi-UAV systems.

The project demonstrates:

- Multiple UAV agents
- Distributed formation control
- Cooperative waypoint/task allocation
- Collision avoidance
- Reference trajectory generation
- Communication-dropout simulation
- Multi-agent performance metrics

The framework is intended for research and educational work in autonomous
robotics, distributed control, cooperative planning, and multi-agent systems.

> The public implementation is generic and non-operational. It contains no
> weapon, payload, target-engagement, restricted mission, or platform-specific
> logic.

---

## Architecture

```text
               Mission Waypoints
                      |
                      v
               Task Allocation
                      |
                      v
         Cooperative Reference Planner
                      |
                      v
        +-------------+-------------+
        |             |             |
        v             v             v
      UAV-1         UAV-2         UAV-3
        |             |             |
        +------ Distributed State --+
                      |
                      v
              Formation Control
                      |
                      v
             Collision Avoidance
                      |
                      v
              UAV Kinematics
```

---

## Main Components

### 1. Multi-UAV Kinematics

Each vehicle is represented by a generic planar kinematic model:

```text
x_dot = V cos(psi)
y_dot = V sin(psi)
```

with bounded speed and heading-rate commands.

### 2. Task Allocation

Mission waypoints are assigned to UAVs using a lightweight distance-based
greedy allocator.

### 3. Formation Control

A leader-follower architecture is used to maintain configurable relative
offsets.

### 4. Collision Avoidance

A repulsive safety term modifies the reference direction whenever neighboring
UAVs violate a configurable separation margin.

### 5. Communication Dropout

The simulation can emulate temporary loss of neighboring-agent state
information to evaluate distributed robustness.

### 6. Evaluation

The framework reports:

- Formation RMSE
- Minimum inter-UAV distance
- Mean tracking error
- Task completion ratio
- Total path length
- Communication dropout ratio

---

## Repository Structure

```text
multi_uav_cooperative_autonomy/
├── README.md
├── requirements.txt
├── .gitignore
├── src/
│   ├── environment/
│   │   └── uav_model.py
│   ├── allocation/
│   │   └── task_allocator.py
│   ├── planning/
│   │   └── waypoint_planner.py
│   ├── coordination/
│   │   ├── formation_control.py
│   │   └── collision_avoidance.py
│   ├── control/
│   │   └── heading_controller.py
│   └── simulation/
│       └── multi_uav_simulation.py
└── examples/
    ├── run_demo.py
    └── communication_dropout_demo.py
```

---

## Installation

```bash
pip install -r requirements.txt
```

Run the nominal cooperative-autonomy demo:

```bash
python examples/run_demo.py
```

Run the communication-dropout scenario:

```bash
python examples/communication_dropout_demo.py
```

---

## Technologies

- Python
- NumPy
- Matplotlib

---

## Research Areas

- Multi-Agent Robotics
- Cooperative Autonomy
- Distributed Control
- Formation Control
- Cooperative Planning
- Task Allocation
- Collision Avoidance
- Autonomous UAV Systems

---

## Public Implementation Notice

The public code contains generic and sanitized educational implementations.
It intentionally excludes platform-specific parameters, operational mission
logic, payloads, engagement logic, restricted coordinates, and unpublished
system configurations.

## Status

Research-oriented educational implementation.
