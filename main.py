"""Run the second-order system PID simulation project."""

from pathlib import Path

import matplotlib.pyplot as plt

from src.metrics import calculate_metrics
from src.pid_controller import PIDController
from src.simulation import simulate_closed_loop, simulate_controller_comparison


SETPOINT = 1.0
SIMULATION_TIME = 10.0
DT = 0.01
WN = 2.0
ZETA = 0.3
SETTLING_TOLERANCE = 0.02
OUTPUT_DIR = Path("outputs")
README_PATH = Path("README.md")

CONTROLLER_CONFIGS = [
    {"name": "P", "kp": 8.0, "ki": 0.0, "kd": 0.0},
    {"name": "PI", "kp": 12.0, "ki": 4.0, "kd": 0.0},
    {"name": "PID", "kp": 20.0, "ki": 5.0, "kd": 2.0},
]


def build_controllers():
    """Create fresh controller instances from the default gain table."""
    return [
        PIDController(
            kp=config["kp"],
            ki=config["ki"],
            kd=config["kd"],
            name=config["name"],
        )
        for config in CONTROLLER_CONFIGS
    ]


def settling_band_limits(setpoint, tolerance=SETTLING_TOLERANCE):
    """Return lower and upper limits for the settling band."""
    band = abs(setpoint) * tolerance
    if band == 0:
        band = tolerance
    return setpoint - band, setpoint + band


def add_settling_band(setpoint):
    """Add a light +/- tolerance settling band to the current plot."""
    lower_limit, upper_limit = settling_band_limits(setpoint)
    plt.axhspan(
        lower_limit,
        upper_limit,
        color="#2ca02c",
        alpha=0.10,
        label="+/-2% settling band",
    )


def plot_pid_response(result, metrics, output_path):
    """Save a clean step-response plot for the PID controller."""
    plt.figure(figsize=(9, 5))
    add_settling_band(SETPOINT)
    plt.plot(result["time"], result["setpoint"], "k--", linewidth=1.6, label="Setpoint")
    plt.plot(
        result["time"],
        result["response"],
        color="#1f77b4",
        linewidth=2.2,
        label="System response",
    )

    settling_time = metrics["settling_time_seconds"]
    if settling_time is not None:
        plt.axvline(
            settling_time,
            color="#d62728",
            linestyle="--",
            linewidth=1.3,
            label="Settling time",
        )
        annotation_y = SETPOINT + 0.12
        plt.annotate(
            f"Settling time = {settling_time:.2f} s",
            xy=(settling_time, SETPOINT),
            xytext=(settling_time + 0.35, annotation_y),
            arrowprops={"arrowstyle": "->", "color": "#d62728", "linewidth": 1.0},
            fontsize=9,
            color="#333333",
        )

    plt.title("PID Closed-Loop Step Response")
    plt.xlabel("Time (s)")
    plt.ylabel("Output y")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def plot_controller_comparison(results, output_path):
    """Save a comparison plot for P, PI, and PID step responses."""
    colors = {
        "P": "#1f77b4",
        "PI": "#ff7f0e",
        "PID": "#2ca02c",
    }

    plt.figure(figsize=(10, 5.5))
    first_result = results[0]
    add_settling_band(SETPOINT)
    plt.plot(
        first_result["time"],
        first_result["setpoint"],
        "k--",
        linewidth=1.5,
        label="Setpoint",
    )

    for result in results:
        plt.plot(
            result["time"],
            result["response"],
            linewidth=2.0,
            color=colors.get(result["name"]),
            label=f"{result['name']} response",
        )

    plt.title("P / PI / PID Controller Comparison")
    plt.xlabel("Time (s)")
    plt.ylabel("Output y")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def format_metric_value(value):
    """Format metric values for terminal and text-file tables."""
    if value is None:
        return "Not settled"
    return f"{value:.4f}"


def build_metrics_table(results):
    """Create metric rows and a readable text table."""
    rows = []
    for result in results:
        metrics = calculate_metrics(result["time"], result["response"], SETPOINT)
        rows.append(
            {
                "controller": result["name"],
                "overshoot": metrics["overshoot_percent"],
                "final_window_error": metrics["final_window_tracking_error"],
                "settling_time": metrics["settling_time_seconds"],
            }
        )

    header = (
        f"{'Controller':<12}"
        f"{'Overshoot (%)':>16}"
        f"{'Final-window Tracking Error':>32}"
        f"{'Settling Time (s)':>20}"
    )
    separator = "-" * len(header)
    lines = [header, separator]

    for row in rows:
        lines.append(
            f"{row['controller']:<12}"
            f"{row['overshoot']:>16.4f}"
            f"{row['final_window_error']:>32.4f}"
            f"{format_metric_value(row['settling_time']):>20}"
        )

    return rows, lines


def build_readme_results_table(rows):
    """Build the Markdown results table inserted into README.md."""
    lines = [
        "| Controller | Overshoot | Final-window tracking error | Settling time |",
        "| --- | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| "
            f"{row['controller']} | "
            f"{row['overshoot']:.4f}% | "
            f"{row['final_window_error']:.4f} | "
            f"{format_metric_value(row['settling_time'])} |"
        )
    return "\n".join(lines)


def update_readme_results_table(rows, readme_path=README_PATH):
    """Update the generated results table in README.md when markers exist."""
    if not readme_path.exists():
        return

    start_marker = "<!-- RESULTS_TABLE_START -->"
    end_marker = "<!-- RESULTS_TABLE_END -->"
    readme_text = readme_path.read_text(encoding="utf-8")
    if start_marker not in readme_text or end_marker not in readme_text:
        return

    before, remainder = readme_text.split(start_marker, 1)
    _, after = remainder.split(end_marker, 1)
    table = build_readme_results_table(rows)
    updated_text = f"{before}{start_marker}\n{table}\n{end_marker}{after}"
    readme_path.write_text(updated_text, encoding="utf-8")


def write_metrics_file(lines, output_path):
    """Write the performance metrics table to a text file."""
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    """Run simulations, save plots, and print performance metrics."""
    OUTPUT_DIR.mkdir(exist_ok=True)

    pid_controller = PIDController(kp=20.0, ki=5.0, kd=2.0, name="PID")
    pid_result = simulate_closed_loop(
        controller=pid_controller,
        setpoint=SETPOINT,
        simulation_time=SIMULATION_TIME,
        dt=DT,
        wn=WN,
        zeta=ZETA,
    )
    pid_metrics = calculate_metrics(pid_result["time"], pid_result["response"], SETPOINT)
    plot_pid_response(pid_result, pid_metrics, OUTPUT_DIR / "pid_response.png")

    comparison_results = simulate_controller_comparison(
        controllers=build_controllers(),
        setpoint=SETPOINT,
        simulation_time=SIMULATION_TIME,
        dt=DT,
        wn=WN,
        zeta=ZETA,
    )
    plot_controller_comparison(
        comparison_results,
        OUTPUT_DIR / "controller_comparison.png",
    )

    metrics_rows, metrics_lines = build_metrics_table(comparison_results)
    write_metrics_file(metrics_lines, OUTPUT_DIR / "performance_metrics.txt")
    update_readme_results_table(metrics_rows)

    print("\nClosed-loop step-response performance metrics\n")
    print("\n".join(metrics_lines))
    print(f"\nGenerated outputs in: {OUTPUT_DIR.resolve()}")


if __name__ == "__main__":
    main()
