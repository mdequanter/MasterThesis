import pandas as pd
import scipy.stats as stats
import plotly.graph_objects as go
from pathlib import Path
import re

# === Prompt user for CSV file ===
file_path = input("Enter the path to the CSV file: ").strip()

# === Extract default title from filename ===
filename = Path(file_path).stem  # e.g., "laptop_5G_20250707_231800"

# Try to remove timestamp suffix (numbers separated by underscores)
match = re.match(r"(.+?)_\d{8}_\d{6}$", filename)
if match:
    default_title = match.group(1)
else:
    default_title = filename

# === Prompt user to confirm or enter a new title ===
title_input = input(f"Enter a title for the chart [{default_title}]: ").strip()
final_title = title_input if title_input else default_title

# === Load CSV ===
df = pd.read_csv(file_path)

# === Metrics and clean descriptions ===
metric_labels = {
    "avg_latency_ms": "End-to-End Latency (ms)",
    "inference_ms": "Inference Time (ms)",
    "processing_ms": "Processing Time (ms)",
    "avg_fps": "Average FPS",
    "avg_size_kb": "Frame size (kb)",
    "poweruse_W": "Power Use (W)",
}

# === Predefined colors per metric ===
metric_colors = {
    "End-to-End Latency (ms)": "royalblue",
    "Average FPS": "seagreen",
    "Inference Time (ms)": "darkorange",
    "Processing Time (ms)" : "darkred",
    "Power Use (W)": "indianred",
    "Frame size (kb)": "yellow"
}

metrics = list(metric_labels.keys())

# === Compute 95% confidence intervals ===
def compute_ci(series):
    n = len(series)
    mean = series.mean()
    h = stats.sem(series) * stats.t.ppf(0.975, n - 1) if n > 1 else 0
    return round(mean, 2), round(h, 2)

records = []
for metric in metrics:
    mean, ci = compute_ci(df[metric])
    records.append({
        "Metric": metric_labels[metric],
        "Mean": mean,
        "95% CI": ci
    })

ci_df = pd.DataFrame(records)

# === Prepare text labels with Mean ± CI ===
ci_df["Label"] = ci_df.apply(lambda row: f"{row['Mean']} ± {row['95% CI']}", axis=1)

# === Create a single bar chart with all metrics next to each other ===
fig = go.Figure()

for i, row in ci_df.iterrows():
    fig.add_trace(
        go.Bar(
            x=[row["Metric"]],
            y=[row["Mean"]],
            error_y=dict(
                type="data",
                array=[row["95% CI"]],
                visible=True
            ),
            marker_color=metric_colors[row["Metric"]],
            text=[row["Label"]],
            textposition="outside",
            name=row["Metric"],
            textfont=dict(size=14)
        )
    )

# === Layout ===
fig.update_layout(
    title=f"{final_title} — 95% Confidence Intervals",
    template="plotly_white",
    font=dict(size=16),
    title_font=dict(size=22),
    xaxis_title="Metric",
    yaxis_title="Value",
    barmode="group",
    showlegend=False
)

fig.update_yaxes(tickformat=".2f")

fig.show()
