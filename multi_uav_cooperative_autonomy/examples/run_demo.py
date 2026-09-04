import matplotlib.pyplot as plt

from src.simulation.multi_uav_simulation import run_simulation


result = run_simulation()

print("Metrics")
for key, value in result["metrics"].items():
    print(f"{key}: {value:.3f}")

plt.figure()

for i, trajectory in enumerate(result["trajectories"]):
    plt.plot(
        trajectory[:, 0],
        trajectory[:, 1],
        label=f"UAV-{i+1}",
    )

tasks = result["tasks"]
plt.scatter(
    tasks[:, 0],
    tasks[:, 1],
    marker="x",
    s=70,
    label="Mission waypoints",
)

plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Multi-UAV Cooperative Autonomy")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()
