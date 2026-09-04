import numpy as np

from src.environment.uav_model import UAVState, position, propagate
from src.allocation.task_allocator import greedy_task_allocation
from src.planning.waypoint_planner import current_waypoint, reached_waypoint
from src.coordination.formation_control import formation_reference
from src.coordination.collision_avoidance import repulsive_vector
from src.control.heading_controller import heading_to_vector, heading_controller


def run_simulation(
    duration=120.0,
    dt=0.1,
    dropout_probability=0.0,
    seed=7,
):
    rng = np.random.default_rng(seed)

    agents = [
        UAVState(0.0,   0.0, 10.0, 0.0),
        UAVState(-25.0, -20.0, 10.0, 0.0),
        UAVState(-25.0,  20.0, 10.0, 0.0),
        UAVState(-50.0,   0.0, 10.0, 0.0),
    ]

    tasks = np.array([
        [220.0,  50.0],
        [340.0, -30.0],
        [460.0,  70.0],
        [580.0,   0.0],
    ])

    offsets = [
        np.array([0.0, 0.0]),
        np.array([-30.0, -25.0]),
        np.array([-30.0,  25.0]),
        np.array([-60.0,   0.0]),
    ]

    assignments = greedy_task_allocation(
        [position(a) for a in agents], tasks
    )
    completed = [0] * len(agents)

    trajectories = [[] for _ in agents]
    formation_errors = []
    min_distances = []
    dropout_count = 0
    path_lengths = np.zeros(len(agents))

    previous_positions = [position(a) for a in agents]

    steps = int(duration / dt)

    for _ in range(steps):
        positions = np.array([position(a) for a in agents])
        leader = agents[0]

        pairwise = []
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                pairwise.append(np.linalg.norm(positions[i] - positions[j]))
        min_distances.append(min(pairwise))

        updated = []

        for i, agent in enumerate(agents):
            communication_available = (
                rng.random() > dropout_probability or i == 0
            )

            if not communication_available:
                dropout_count += 1

            waypoint = current_waypoint(
                tasks, assignments[i], completed[i]
            )

            if waypoint is not None and reached_waypoint(
                position(agent), waypoint
            ):
                completed[i] += 1
                waypoint = current_waypoint(
                    tasks, assignments[i], completed[i]
                )

            if i == 0:
                if waypoint is None:
                    desired = position(agent) + np.array([20.0, 0.0])
                else:
                    desired = waypoint
            else:
                if communication_available:
                    desired = formation_reference(
                        position(leader),
                        leader.heading,
                        offsets[i],
                    )
                else:
                    # Continue toward the last general motion direction.
                    desired = position(agent) + np.array([
                        30.0 * np.cos(agent.heading),
                        30.0 * np.sin(agent.heading),
                    ])

            attraction = desired - position(agent)
            avoidance = repulsive_vector(i, positions)
            command_vector = attraction + avoidance

            if np.linalg.norm(command_vector) < 1e-6:
                desired_heading = agent.heading
            else:
                desired_heading = heading_to_vector(command_vector)

            heading_rate, _ = heading_controller(
                agent.heading, desired_heading
            )

            new_agent = propagate(
                agent,
                heading_rate=heading_rate,
                speed_cmd=11.0,
                dt=dt,
            )
            updated.append(new_agent)

        agents = updated

        current_positions = [position(a) for a in agents]

        for i, p in enumerate(current_positions):
            trajectories[i].append(p.copy())
            path_lengths[i] += np.linalg.norm(
                p - previous_positions[i]
            )

        previous_positions = [p.copy() for p in current_positions]

        if len(agents) > 1:
            follower_errors = []
            leader = agents[0]
            for i in range(1, len(agents)):
                desired = formation_reference(
                    position(leader),
                    leader.heading,
                    offsets[i],
                )
                follower_errors.append(
                    np.linalg.norm(position(agents[i]) - desired)
                )
            formation_errors.extend(follower_errors)

    total_tasks = sum(len(a) for a in assignments)
    completed_tasks = sum(min(completed[i], len(assignments[i]))
                          for i in range(len(agents)))

    metrics = {
        "formation_rmse_m": float(
            np.sqrt(np.mean(np.square(formation_errors)))
        ),
        "minimum_inter_uav_distance_m": float(np.min(min_distances)),
        "task_completion_ratio": float(
            completed_tasks / max(total_tasks, 1)
        ),
        "total_path_length_m": float(np.sum(path_lengths)),
        "communication_dropout_ratio": float(
            dropout_count / max(steps * (len(agents) - 1), 1)
        ),
    }

    return {
        "trajectories": [np.asarray(t) for t in trajectories],
        "tasks": tasks,
        "assignments": assignments,
        "metrics": metrics,
    }
