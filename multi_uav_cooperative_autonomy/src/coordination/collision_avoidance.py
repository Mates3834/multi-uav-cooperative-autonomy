import numpy as np


def repulsive_vector(agent_idx, positions, safety_radius=25.0, gain=180.0):
    """
    Simple repulsive field for generic collision avoidance.
    """
    positions = np.asarray(positions, dtype=float)
    p = positions[agent_idx]
    repulsion = np.zeros(2)

    for j, other in enumerate(positions):
        if j == agent_idx:
            continue

        delta = p - other
        distance = np.linalg.norm(delta)

        if 1e-6 < distance < safety_radius:
            direction = delta / distance
            magnitude = gain * (
                1.0 / distance - 1.0 / safety_radius
            )
            repulsion += magnitude * direction

    return repulsion
