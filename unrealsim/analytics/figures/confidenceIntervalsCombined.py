import pandas as pd
import scipy.stats as stats
import plotly.graph_objects as go
from pathlib import Path
import re

# === Directory with all CSV files ===
folder_path = Path("unrealsim/analytics/external")

# === Load and sort all CSV files ===
all_files = sorted(folder_path.glob("*.csv"), key=lambda f: f.name.lower())

if not all_files:
    print(f"No CSV files found in {folder_path}")
    exit(1)

# === Determine default title ===
prefixes = set()
for f in all_files:
    m = re.match(r"(.+?)_\d{8}_\d{6}\.csv$", f.name)
    if m:
        prefixes.add(m.group(1))
    else:
        prefixes.add(f.stem)

if len(prefixes) == 1:
    default_title = prefixes.pop()
else:
    default_title = folder_path.name

# === Prompt for title ===
title_input = input(f"Enter a title for the chart [{default_title}]: ").strip()
final_title = title_input if title_input else default_title

# === Metrics and clean descriptions ===
metric_labels = {
    "latency_ms": "End-to-End Latency (ms)",
    "inference_ms": "Inference Time (ms)",
    "processing_ms": "Processing Time (ms)",
    "avg_fps": "Average FPS",
    "poweruse_W" : "Power use (W)",
}

metrics = list(metric_labels.keys())

# === Compute 95% CI function ===
def compute_ci(series):
    n = len(series)
    mean = series.mean()
    h = stats.sem(series) * stats.t.ppf(0.975, n - 1) if n > 1 else 0
    return round(mean, 2), round(h, 2)

# === Prepare figure ===
fig = go.Figure()

# === Assign a unique color per file ===
color_palette = [
    "#1f77b4",  # blue
    "#ff7f0e",  # orange
    "#2ca02c",  # green
    "#d62728",  # red
    "#9467bd",  # purple
    "#8c564b",  # brown
]

# === Process each file ===
for idx, file in enumerate(all_files):
    df = pd.read_csv(file)

    records = []
    for metric in metrics:
        mean, ci = compute_ci(df[metric])
        records.append({
            "Metric": metric_labels[metric],
            "Mean": mean,
            "95% CI": ci
        })

    ci_df = pd.DataFrame(records)
    ci_df["Label"] = ci_df.apply(lambda row: f"{row['Mean']} ± {row['95% CI']}", axis=1)

    color = color_palette[idx % len(color_palette)]
    legend_name = file.stem

    # Add one bar trace per file with all metrics
    fig.add_trace(
        go.Bar(
            x=ci_df["Metric"],
            y=ci_df["Mean"],
            error_y=dict(
                type="data",
                array=ci_df["95% CI"],
                visible=True
            ),
            marker_color=color,
            text=ci_df["Label"],
            textposition="auto",
            textfont=dict(size=16, color="white"),
            name=legend_name
        )
    )

# === Layout with log scale and rotated labels ===
fig.update_layout(
    title=f"{final_title} — 95% Confidence Intervals",
    template="plotly_white",
    font=dict(size=16),
    title_font=dict(size=24),
    width=1200,
    xaxis_title="Metric",
    yaxis_title="Value (log scale)",
    barmode="group",
    bargap=0.05,
    bargroupgap=0.02,
    legend=dict(
        orientation="h",
        yanchor="top",
        y=-0.25,
        xanchor="center",
        x=0.5,
        font=dict(size=16)
    )
)

fig.update_yaxes(
    type="log",
    tickfont=dict(size=16),
)

fig.show()
