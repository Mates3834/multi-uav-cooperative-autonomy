import numpy as np


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def heading_to_vector(vector_xy):
    vector_xy = np.asarray(vector_xy, dtype=float)
    return float(np.arctan2(vector_xy[1], vector_xy[0]))


def heading_controller(current_heading, desired_heading,
                       gain=2.2,
                       max_heading_rate=np.deg2rad(50.0)):
    error = wrap_angle(desired_heading - current_heading)
    command = np.clip(
        gain * error, -max_heading_rate, max_heading_rate
    )
    return float(command), float(error)
