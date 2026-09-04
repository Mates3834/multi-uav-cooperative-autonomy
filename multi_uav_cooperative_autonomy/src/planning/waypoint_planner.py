import numpy as np


def current_waypoint(tasks, assignment, completed_count):
    if completed_count >= len(assignment):
        return None
    return np.asarray(tasks[assignment[completed_count]], dtype=float)


def reached_waypoint(position_xy, waypoint_xy, tolerance=12.0):
    if waypoint_xy is None:
        return True
    return np.linalg.norm(
        np.asarray(position_xy) - np.asarray(waypoint_xy)
    ) <= tolerance
