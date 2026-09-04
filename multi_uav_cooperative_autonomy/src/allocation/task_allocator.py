import numpy as np


def greedy_task_allocation(agent_positions, tasks):
    """
    Assign each task to the currently nearest agent.

    Returns a list of task-index lists, one list per agent.
    """
    agents = np.asarray(agent_positions, dtype=float)
    tasks = np.asarray(tasks, dtype=float)

    assignments = [[] for _ in range(len(agents))]
    virtual_positions = agents.copy()

    for task_idx, task in enumerate(tasks):
        distances = np.linalg.norm(virtual_positions - task, axis=1)
        agent_idx = int(np.argmin(distances))
        assignments[agent_idx].append(task_idx)
        virtual_positions[agent_idx] = task

    return assignments
