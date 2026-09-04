# Multi-UAV Cooperative Autonomy

A modular simulation framework for **cooperative autonomy in multi-UAV systems**, focusing on distributed formation control, waypoint/task allocation, collision avoidance, communication uncertainty, and multi-agent performance evaluation.

The project investigates how multiple autonomous UAVs can coordinate their motion and complete shared mission objectives while maintaining relative formation geometry and safe inter-vehicle separation.

The public implementation is designed as a **generic research and educational framework** for multi-agent robotics, distributed control, cooperative planning, and autonomous aerial systems.

> **Note:** The public implementation is intentionally generic and sanitized. It contains no operational mission parameters, payload logic, engagement logic, restricted coordinates, or platform-specific configurations.

---

## System Architecture

The framework follows a layered cooperative-autonomy architecture:

```text
                Mission Waypoints
                       |
                       v
                Task Allocation
                       |
                       v
             Cooperative Planning
                       |
                       v
          +------------+------------+
          |            |            |
          v            v            v
        UAV-1        UAV-2        UAV-3
          |            |            |
          +------ Distributed ------+
                 State Exchange
                       |
                       v
                Formation Control
                       |
                       v
               Collision Avoidance
                       |
                       v
                Heading Control
                       |
                       v
                 UAV Kinematics
```

The architecture separates **task allocation, cooperative planning, formation control, local collision avoidance, vehicle control, and simulation**, allowing each component to be extended independently.

---

# 1. Multi-UAV System

The simulation considers multiple UAV agents operating in a common two-dimensional environment.

Each vehicle is represented by a generic planar kinematic state:

```text
x_i =
[x_i, y_i, V_i, psi_i]^T
```

where

```text
x_i, y_i = UAV position
V_i      = forward speed
psi_i    = heading angle
```

The current example uses four UAV agents.

---

# 2. UAV Kinematic Model

Each UAV follows a simple planar model:

```text
x_dot = V cos(psi)
```

```text
y_dot = V sin(psi)
```

```text
psi_dot = omega
```

where

```text
omega = commanded heading rate
```

The vehicle speed is also driven toward a commanded value using a simple first-order response.

The model includes configurable limits for:

- Minimum speed
- Maximum speed
- Maximum heading rate

This provides a lightweight platform for evaluating cooperative-autonomy algorithms without requiring a platform-specific flight-dynamics model.

---

# 3. Mission Waypoints

The cooperative mission is represented using a collection of global waypoints:

```text
W =
{w_1, w_2, ..., w_N}
```

with

```text
w_j = [x_j, y_j]^T
```

The waypoints represent generic mission tasks that must be distributed among the available UAVs.

The current simulation includes multiple spatially separated waypoints.

---

# 4. Cooperative Task Allocation

A lightweight distance-based greedy task-allocation algorithm is included.

For each task, the Euclidean distance between the task and candidate UAV positions is evaluated.

Conceptually:

```text
Mission Task
     |
     v
Distance to UAV-1
Distance to UAV-2
Distance to UAV-3
Distance to UAV-4
     |
     v
Nearest Available Agent
     |
     v
Task Assignment
```

The allocation function produces a set of task lists:

```text
UAV-1 -> Task indices
UAV-2 -> Task indices
UAV-3 -> Task indices
UAV-4 -> Task indices
```

The current method is intentionally simple and is intended to provide a clear baseline for future optimization-based task allocation.

---

# 5. Leader-Follower Formation

The project uses a generic **leader-follower formation architecture**.

One UAV acts as the reference agent:

```text
Leader = UAV-1
```

and the remaining agents maintain predefined relative offsets.

For a follower with body-frame offset

```text
r_i =
[Delta x_i, Delta y_i]^T
```

the desired world-frame position is calculated using the leader heading.

The offset is rotated using

```text
R(psi) =
[ cos(psi)  -sin(psi) ]
[ sin(psi)   cos(psi) ]
```

and the desired follower position becomes

```text
p_i,d =
p_leader + R(psi_leader) r_i
```

This allows the formation geometry to rotate naturally with the leader motion.

---

# 6. Formation Error

The formation-position error is defined as

```text
e_i =
p_i,d - p_i
```

where

```text
p_i,d = desired follower position
p_i   = actual follower position
```

The magnitude of this error is used to evaluate formation quality.

The simulation aggregates follower errors over time and calculates a formation RMSE.

---

# 7. Cooperative Reference Generation

For the leader, the active task waypoint is used as the primary reference.

For follower UAVs, the desired formation location is generated from the leader state.

The resulting concept is:

```text
Leader
  |
  +--> Mission waypoint
  |
  +--> Leader position + heading
            |
            v
      Formation Geometry
            |
            +--> Follower-1 reference
            +--> Follower-2 reference
            +--> Follower-3 reference
```

This separates global mission motion from local formation maintenance.

---

# 8. Collision Avoidance

A generic repulsive collision-avoidance term is included.

For UAV `i` and neighboring UAV `j`:

```text
Delta p_ij =
p_i - p_j
```

The inter-UAV distance is

```text
d_ij =
||Delta p_ij||
```

If

```text
d_ij < d_safe
```

a repulsive vector is generated.

Conceptually:

```text
Neighbor far away
      ↓
No repulsion

Neighbor inside safety region
      ↓
Repulsive correction
      ↓
Reference direction modified
```

The resulting command vector is formed as:

```text
command_vector =
attractive_vector
+
repulsive_vector
```

This gives each UAV a basic local collision-avoidance capability.

---

# 9. Heading Guidance

The desired motion vector is converted into a desired heading:

```text
psi_d =
atan2(v_y, v_x)
```

where

```text
[v_x, v_y]^T
```

is the commanded planar direction.

The heading error is

```text
e_psi =
wrap(psi_d - psi)
```

and the heading-rate command is generated using proportional feedback:

```text
omega_cmd =
K_psi e_psi
```

subject to heading-rate limits.

---

# 10. Distributed State Information

Formation control depends on access to neighboring-agent information.

The framework therefore conceptually includes distributed state exchange:

```text
UAV-1 State
    ↕
UAV-2 State
    ↕
UAV-3 State
    ↕
UAV-4 State
```

The current implementation uses a simplified communication-availability model to investigate the effect of information loss.

---

# 11. Communication Dropout

The project includes a configurable **communication dropout probability**.

At each simulation step, a follower can temporarily lose access to the leader state.

If communication is available:

```text
Leader State
     ↓
Formation Reference
     ↓
Follower Control
```

If communication is unavailable:

```text
No Current Leader State
          ↓
Maintain Current Motion Direction
          ↓
Temporary Open-Loop Propagation
```

This allows basic robustness studies under intermittent information exchange.

---

# 12. Nominal Scenario

The nominal simulation uses:

```text
dropout_probability = 0
```

All follower UAVs therefore receive leader information continuously.

This scenario evaluates the baseline cooperative formation behavior.

---

# 13. Communication-Loss Scenario

The second example evaluates multiple communication conditions.

For example:

```text
0% dropout
10% dropout
25% dropout
```

The same multi-UAV architecture is evaluated under each condition.

This provides a basic framework for analyzing how distributed information availability influences formation performance.

---

# 14. End-to-End Cooperative Architecture

The complete framework can be summarized as:

```text
               Global Mission
                    |
                    v
            Mission Waypoints
                    |
                    v
             Task Allocation
                    |
                    v
                UAV Team
                    |
          +---------+---------+
          |         |         |
          v         v         v
        UAV-1     UAV-2     UAV-3 ...
          |
          v
       Leader Motion
          |
          v
    Formation References
          |
          v
 Distributed Coordination
          |
          v
 Collision Avoidance
          |
          v
   Heading Controllers
          |
          v
      UAV Dynamics
          |
          v
      Updated States
          |
          +--------------------+
                               |
                               v
                       Coordination Loop
```

---

# 15. Evaluation Metrics

The simulation calculates several multi-agent performance metrics.

## Formation RMSE

The formation RMSE is calculated from the relative-position errors:

```text
RMSE_formation =
sqrt(
    1/N *
    sum(e_i^2)
)
```

This quantifies how accurately the followers maintain the desired formation geometry.

---

## Minimum Inter-UAV Distance

The minimum pairwise distance is

```text
d_min =
min ||p_i - p_j||
```

for all

```text
i != j
```

This metric provides an indication of cooperative separation safety.

---

## Task Completion Ratio

The task completion ratio is

```text
Task Completion Ratio =
Completed Tasks / Assigned Tasks
```

This provides a high-level measure of mission completion.

---

## Total Path Length

Each UAV path length is calculated as

```text
L_i =
sum ||p_i(k) - p_i(k-1)||
```

and the team path length is

```text
L_total =
sum L_i
```

This can be used in future optimization studies.

---

## Communication Dropout Ratio

The effective communication dropout ratio is calculated from the unavailable follower-state updates.

This provides a measurable link between communication quality and cooperative performance.

---

# 16. Repository Structure

```text
multi_uav_cooperative_autonomy/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── src/
│   │
│   ├── environment/
│   │   ├── __init__.py
│   │   └── uav_model.py
│   │
│   ├── allocation/
│   │   ├── __init__.py
│   │   └── task_allocator.py
│   │
│   ├── planning/
│   │   ├── __init__.py
│   │   └── waypoint_planner.py
│   │
│   ├── coordination/
│   │   ├── __init__.py
│   │   ├── formation_control.py
│   │   └── collision_avoidance.py
│   │
│   ├── control/
│   │   ├── __init__.py
│   │   └── heading_controller.py
│   │
│   └── simulation/
│       ├── __init__.py
│       └── multi_uav_simulation.py
│
└── examples/
    ├── run_demo.py
    └── communication_dropout_demo.py
```

---

# 17. Module Description

| Module | Purpose |
|---|---|
| `uav_model.py` | Generic UAV planar kinematics |
| `task_allocator.py` | Greedy cooperative task allocation |
| `waypoint_planner.py` | Active waypoint selection and completion checking |
| `formation_control.py` | Leader-follower formation reference generation |
| `collision_avoidance.py` | Inter-UAV repulsive separation logic |
| `heading_controller.py` | Heading guidance and bounded heading-rate control |
| `multi_uav_simulation.py` | Integrated cooperative-autonomy simulation |
| `run_demo.py` | Nominal multi-UAV demonstration |
| `communication_dropout_demo.py` | Communication-loss comparison |

---

# 18. Running the Project

Install the dependencies:

```bash
pip install -r requirements.txt
```

Run the nominal scenario:

```bash
python examples/run_demo.py
```

Run communication-dropout experiments:

```bash
python examples/communication_dropout_demo.py
```

---

# 19. Example Outputs

The main demonstration can visualize:

- Individual UAV trajectories
- Mission waypoint locations
- Cooperative formation motion
- Task completion
- Inter-UAV separation

Recommended repository figures:

```text
results/
├── multi_uav_trajectories.png
├── formation_error.png
├── minimum_separation.png
├── task_completion.png
└── communication_dropout_comparison.png
```

These figures would make the cooperative-autonomy behavior easier to evaluate visually.

---

# 20. Comparative Experiments

A useful experiment is to compare communication conditions:

| Scenario | Communication Dropout |
|---|---:|
| Nominal | 0% |
| Moderate Loss | 10% |
| High Loss | 25% |

The following metrics can then be compared:

```text
Formation RMSE
Minimum Separation
Task Completion Ratio
Total Path Length
```

This makes the project more than a single simulation by providing a simple robustness study.

---

# 21. Research Areas

The project is related to:

- Multi-Agent Robotics
- Multi-UAV Systems
- Cooperative Autonomy
- Formation Control
- Distributed Control
- Cooperative Path Planning
- Task Allocation
- Collision Avoidance
- Autonomous Aerial Systems
- Networked Robotics
- Guidance and Control

---

# 22. Technologies

- Python
- NumPy
- Matplotlib
- Multi-Agent Simulation
- Formation Control
- Task Allocation
- Collision Avoidance
- Distributed Coordination

---

# 23. Project Motivation

Many autonomous robotic applications require multiple agents to cooperate rather than operate independently.

A multi-UAV system introduces additional challenges:

```text
Single UAV
   ↓
Trajectory Tracking

Multi-UAV Team
   ↓
Trajectory Tracking
+
Relative Formation
+
Task Distribution
+
Collision Avoidance
+
Communication
+
Coordination
```

The purpose of this project is therefore to demonstrate how these individual problems can be integrated within a common cooperative-autonomy framework.

---

# 24. Possible Future Extensions

The current framework can be extended in several directions.

### Cooperative Planning

- A* / Hybrid A* multi-agent planning
- Conflict-Based Search
- Prioritized planning
- Dynamic replanning
- Distributed path planning

### Task Allocation

- Hungarian algorithm
- Auction-based allocation
- Consensus-based bundle algorithm
- Optimization-based assignment
- Dynamic task reassignment

### Formation Control

- Consensus control
- Virtual-structure control
- Graph-based formation control
- Distributed MPC
- Adaptive formation control

### Communication

- Communication delays
- Packet loss
- Limited communication range
- Network topology changes
- Event-triggered communication
- Consensus under communication uncertainty

### UAV Dynamics

- 3-D UAV dynamics
- Quadrotor 6-DoF model
- Attitude control
- Wind disturbances
- Actuator dynamics
- Energy constraints

### Multi-Agent Intelligence

- Multi-Agent Reinforcement Learning
- MAPPO
- MADDPG
- QMIX
- Learning-based task allocation
- Cooperative exploration

---

# 25. Future Research Architecture

A more advanced version could evolve toward:

```text
                 Mission Manager
                       |
                       v
              Cooperative Planner
                       |
              +--------+--------+
              |                 |
              v                 v
       Task Allocation      Global Planning
              |                 |
              +--------+--------+
                       |
                       v
             Distributed Coordination
                       |
        +--------------+--------------+
        |              |              |
        v              v              v
      UAV-1          UAV-2          UAV-3
        |              |              |
        +--------- State Exchange ----+
                       |
                       v
                Consensus Layer
                       |
                       v
             Formation / DMPC
                       |
                       v
             Collision Avoidance
                       |
                       v
               Vehicle Control
```

---

# 26. Public Implementation Notice

The source code provided in this repository contains **generic and sanitized implementations** of multi-agent autonomy concepts.

The public version intentionally excludes:

- Operational mission coordinates
- Platform-specific UAV parameters
- Restricted communication protocols
- Real-world operational mission logic
- Payload-control logic
- Engagement logic
- Terminal guidance
- Threat-response logic
- Classified or sensitive parameters
- Unpublished system configurations

The repository should therefore be interpreted as a **research and educational multi-agent robotics framework**, not as an operational autonomous mission system.

---

# 27. Status

**Research-oriented cooperative-autonomy prototype / active development**

The current implementation includes:

- Four-UAV cooperative simulation
- Generic UAV kinematics
- Greedy task allocation
- Leader-follower formation control
- Collision avoidance
- Heading control
- Communication-dropout simulation
- Formation and mission performance metrics

More advanced distributed optimization, multi-agent path planning, consensus control, and 3-D UAV dynamics are considered future extensions.

---

# Author

**Mehmet Ateş**

Research interests:

- Autonomous Systems
- Multi-Agent Robotics
- Cooperative Autonomy
- Guidance, Navigation and Control
- UAV Autonomy
- USV Autonomy
- Path Planning
- State Estimation
- Sensor Fusion
- Reinforcement Learning
- Distributed Control
