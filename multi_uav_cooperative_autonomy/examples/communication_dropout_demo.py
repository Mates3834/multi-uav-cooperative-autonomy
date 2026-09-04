from src.simulation.multi_uav_simulation import run_simulation


for probability in (0.0, 0.10, 0.25):
    result = run_simulation(
        dropout_probability=probability,
        seed=7,
    )

    print(f"\nDropout probability: {probability:.2f}")
    for key, value in result["metrics"].items():
        print(f"{key}: {value:.3f}")
