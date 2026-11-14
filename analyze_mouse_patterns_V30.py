
#!/usr/bin/env python3
"""
Compare physical mouse recordings with generated native HID output.

Usage:
    python3 analyze_mouse_patterns.py
    python3 analyze_mouse_patterns.py --pattern drawCircle

Directory structure:
    mouse_recordings/
        recording_001.csv
        generated_native/
            generated_native_0001.csv

    mouse_recordings/drawCircle/
        recording_001.csv
        generated_native/
            generated_native_0001.csv

Outputs:
    <selected recording directory>/analysis/
        position_x_histogram.png
        position_y_histogram.png
        velocity_cdf.png
        acceleration_cdf.png
        hid_report_timing_histogram.png
        summary.txt
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


BASE_DIR = Path(__file__).resolve().parent / "mouse_recordings"

POSITION_BINS = 80
TIMING_BINS = 72

MIN_KINEMATIC_DT_MS = 5.0
MAX_KINEMATIC_DT_MS = 100.0

TIMING_PLOT_MIN_MS = 4.0
TIMING_PLOT_MAX_MS = 13.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare physical and generated HID recordings."
    )

    parser.add_argument(
        "--pattern",
        type=str,
        default=None,
        help="Recording pattern directory, e.g. drawCircle",
    )

    return parser.parse_args()


def get_directories(pattern):
    if pattern:
        # Accept only a directory name, not a path.
        if not all(
            c.isalnum() or c in "_-"
            for c in pattern
        ):
            raise ValueError("Invalid pattern name")

        root = BASE_DIR / pattern
    else:
        root = BASE_DIR

    generated = root / "generated_native"
    output = root / "analysis"

    return root, generated, output


def load_sessions(directory, file_pattern):
    """
    Load CSV files independently.

    Each file is treated as a separate recording session.
    This prevents derivatives from crossing file boundaries.
    """

    files = sorted(directory.glob(file_pattern))

    if not files:
        raise FileNotFoundError(
            f"No files matching {file_pattern} in {directory}"
        )

    sessions = []

    required = {
        "time_ns",
        "dx",
        "dy",
        "x",
        "y",
    }

    for filename in files:
        df = pd.read_csv(filename)

        missing = required - set(df.columns)

        if missing:
            print(
                f"Skipping {filename.name}: "
                f"missing {sorted(missing)}"
            )
            continue

        if df.empty:
            continue

        df = df.copy()

        for column in required:
            df[column] = pd.to_numeric(
                df[column],
                errors="coerce",
            )

        df = df.dropna(subset=list(required))

        if df.empty:
            continue

        df = df.sort_values(
            "time_ns",
            kind="stable",
        ).reset_index(drop=True)

        df["source_file"] = filename.name

        # Actual time between consecutive reports.
        dt_s = df["time_ns"].diff() / 1e9

        df["dt_ms"] = dt_s * 1000.0

        # Reject intervals that are too short or too long
        # for meaningful finite-difference kinematics.
        valid_dt = (
            (dt_s >= MIN_KINEMATIC_DT_MS / 1000.0)
            & (dt_s <= MAX_KINEMATIC_DT_MS / 1000.0)
        )

        # Velocity components [counts/s].
        df["vx"] = (df["dx"] / dt_s).where(valid_dt)
        df["vy"] = (df["dy"] / dt_s).where(valid_dt)

        # Velocity magnitude.
        df["velocity"] = np.hypot(
            df["vx"],
            df["vy"],
        )

        # Acceleration is the change in the velocity vector,
        # not simply the change in speed magnitude.
        #
        # Both the current and previous velocity estimates
        # must be valid.
        valid_accel = valid_dt & valid_dt.shift(
            1,
            fill_value=False,
        )

        df["ax"] = (
            df["vx"].diff() / dt_s
        ).where(valid_accel)

        df["ay"] = (
            df["vy"].diff() / dt_s
        ).where(valid_accel)

        df["acceleration"] = np.hypot(
            df["ax"],
            df["ay"],
        )

        sessions.append(df)

    if not sessions:
        raise RuntimeError(
            f"No compatible recordings found in {directory}"
        )

    return sessions


def combine_sessions(sessions):
    return pd.concat(
        sessions,
        ignore_index=True,
    )


def clean_values(series, positive=False):
    values = series.to_numpy(dtype=float)

    values = values[np.isfinite(values)]

    if positive:
        values = values[values > 0]

    return values


def save_position_histogram(
    physical,
    generated,
    axis,
    output_dir,
):
    """
    Compare absolute X or Y positions.

    Both datasets use identical bin edges.

    Each histogram is independently normalized so
    its bin percentages sum to 100%.
    """

    p = clean_values(physical[axis])
    g = clean_values(generated[axis])

    low = min(p.min(), g.min())
    high = max(p.max(), g.max())

    if low == high:
        low -= 0.5
        high += 0.5

    edges = np.linspace(
        low,
        high,
        POSITION_BINS + 1,
    )

    p_weights = np.full(
        len(p),
        100.0 / len(p),
    )

    g_weights = np.full(
        len(g),
        100.0 / len(g),
    )

    plt.figure(figsize=(11, 6))

    plt.hist(
        p,
        bins=edges,
        weights=p_weights,
        histtype="step",
        linewidth=2,
        label="Physical",
    )

    plt.hist(
        g,
        bins=edges,
        weights=g_weights,
        histtype="step",
        linewidth=2,
        label="Generated",
    )

    plt.xlabel(
        f"Absolute {axis.upper()} position [counts]"
    )

    plt.ylabel("Reports per position bin [%]")

    plt.title(
        f"Physical vs Generated {axis.upper()} Position"
    )

    plt.legend()
    plt.grid(alpha=0.25)
    plt.tight_layout()

    plt.savefig(
        output_dir / f"position_{axis}_histogram.png",
        dpi=180,
    )

    plt.close()


def save_cdf(
    physical,
    generated,
    metric,
    xlabel,
    output_dir,
):
    """
    Empirical cumulative distribution function.

    The complete distribution is retained, including
    high-value tails.
    """

    plt.figure(figsize=(11, 6))

    for df, label in (
        (physical, "Physical"),
        (generated, "Generated"),
    ):
        values = clean_values(df[metric])

        if len(values) == 0:
            continue

        values = np.sort(values)

        cumulative = (
            np.arange(1, len(values) + 1)
            / len(values)
        )

        plt.plot(
            values,
            cumulative,
            linewidth=2,
            label=label,
        )

    plt.xlabel(xlabel)
    plt.ylabel("Cumulative probability")

    plt.title(
        f"Physical vs Generated {metric.capitalize()} CDF"
    )

    plt.ylim(0, 1.01)
    plt.grid(alpha=0.25)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        output_dir / f"{metric}_cdf.png",
        dpi=180,
    )

    plt.close()


def save_timing_histogram(
    physical,
    generated,
    output_dir,
):
    """
    Compare actual intervals between HID reports.

    Display only 4-13 ms for readability.

    Normalization uses ALL positive report intervals,
    including those outside the visible range.
    """

    edges = np.linspace(
        TIMING_PLOT_MIN_MS,
        TIMING_PLOT_MAX_MS,
        TIMING_BINS + 1,
    )

    plt.figure(figsize=(11, 6))

    for df, label in (
        (physical, "Physical"),
        (generated, "Generated"),
    ):
        dt = clean_values(
            df["dt_ms"],
            positive=True,
        )

        if len(dt) == 0:
            continue

        # Normalize using all positive intervals,
        # rather than only those inside the plot range.
        weights = np.full(
            len(dt),
            100.0 / len(dt),
        )

        plt.hist(
            dt,
            bins=edges,
            weights=weights,
            histtype="step",
            linewidth=2,
            label=label,
        )

    plt.xlabel("Time between HID reports [ms]")
    plt.ylabel("All report intervals per bin [%]")

    plt.title(
        "Physical vs Generated HID Report Timing"
    )

    plt.xlim(
        TIMING_PLOT_MIN_MS,
        TIMING_PLOT_MAX_MS,
    )

    plt.legend()
    plt.grid(alpha=0.25)
    plt.tight_layout()

    plt.savefig(
        output_dir / "hid_report_timing_histogram.png",
        dpi=180,
    )

    plt.close()


def metric_summary(df, metric):
    values = clean_values(df[metric])

    if len(values) == 0:
        return "No valid samples"

    return (
        f"N={len(values)}, "
        f"median={np.median(values):.2f}, "
        f"p95={np.percentile(values, 95):.2f}, "
        f"p99={np.percentile(values, 99):.2f}, "
        f"max={np.max(values):.2f}"
    )


def save_summary(
    physical,
    generated,
    physical_sessions,
    generated_sessions,
    output_dir,
):
    lines = [
        "PHYSICAL VS GENERATED HID ANALYSIS",
        "=" * 40,
        "",
        f"Physical sessions: {len(physical_sessions)}",
        f"Generated sessions: {len(generated_sessions)}",
        f"Physical reports: {len(physical)}",
        f"Generated reports: {len(generated)}",
        "",
    ]

    for metric, units in (
        ("velocity", "counts/s"),
        ("acceleration", "counts/s^2"),
        ("dt_ms", "ms"),
    ):
        lines.extend([
            f"{metric.upper()} [{units}]",
            f"Physical:  {metric_summary(physical, metric)}",
            f"Generated: {metric_summary(generated, metric)}",
            "",
        ])

    summary = "\n".join(lines)

    print(summary)

    (output_dir / "summary.txt").write_text(
        summary + "\n",
        encoding="utf-8",
    )


def main():
    args = parse_args()

    root, generated_dir, output_dir = get_directories(
        args.pattern
    )

    print(f"Recording directory: {root}")
    print(f"Generated directory: {generated_dir}")

    physical_sessions = load_sessions(
        root,
        "recording_*.csv",
    )

    generated_sessions = load_sessions(
        generated_dir,
        "generated_native_*.csv",
    )

    physical = combine_sessions(physical_sessions)
    generated = combine_sessions(generated_sessions)

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    save_position_histogram(
        physical,
        generated,
        "x",
        output_dir,
    )

    save_position_histogram(
        physical,
        generated,
        "y",
        output_dir,
    )

    save_cdf(
        physical,
        generated,
        "velocity",
        "Velocity magnitude [counts/s]",
        output_dir,
    )

    save_cdf(
        physical,
        generated,
        "acceleration",
        "Acceleration magnitude [counts/s²]",
        output_dir,
    )

    save_timing_histogram(
        physical,
        generated,
        output_dir,
    )

    save_summary(
        physical,
        generated,
        physical_sessions,
        generated_sessions,
        output_dir,
    )

    print(f"Plots saved to: {output_dir}")


if __name__ == "__main__":
    main()
