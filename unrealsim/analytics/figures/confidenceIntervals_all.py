import pandas as pd
import scipy.stats as stats
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os
from pathlib import Path

# === Folder containing CSV files ===
data_folder = Path("unrealsim/analytics")

# === Metrics you want to process ===
metrics = [
    "avg_latency_ms",
    "avg_fps",
    "avg_size_kb",
    "avg_compression_ms",
    "avg_encryption_ms",
    "poweruse_W",
    "inference_ms"
]

# === Function to compute 95% confidence intervals ===
def compute_ci(series):
    n = len(series)
    mean = series.mean()
    h = stats.sem(series) * stats.t.ppf(0.975, n - 1) if n > 1 else 0
    return round(mean, 2), round(h, 2)

# === Collect CIs for each file ===
all_records = []

for csv_file in sorted(data_folder.glob("*.csv")):
    df = pd.read_csv(csv_file)
    file_label = csv_file.stem  # filename without .csv

    for metric in metrics:
        mean, ci = compute_ci(df[metric])
        all_records.append({
            "File": file_label,
            "Metric": metric,
            "Mean": mean,
            "95% CI": ci
        })

# === Convert to DataFrame ===
ci_df = pd.DataFrame(all_records)

# === Create subplots ===
fig = make_subplots(
    rows=len(metrics), cols=1,
    shared_xaxes=True,
    vertical_spacing=0.1,
    subplot_titles=[f"{m} with 95% CI" for m in metrics]
)

# === Add traces for each metric ===
for idx, metric in enumerate(metrics, start=1):
    metric_data = ci_df[ci_df["Metric"] == metric]
    fig.add_trace(
        go.Bar(
            x=metric_data["File"],
            y=metric_data["Mean"],
            error_y=dict(type="data", array=metric_data["95% CI"], visible=True),
            name=metric
        ),
        row=idx,
        col=1
    )

# === Layout and styling ===
fig.update_layout(
    title="95% Confidence Intervals for All CSV Files",
    height=300 * len(metrics),
    template="plotly_white",
    barmode="group",
    font=dict(size=16),
    title_font=dict(size=22),
)

# === Y-axis titles ===
for idx, metric in enumerate(metrics, start=1):
    fig.update_yaxes(
        title_text=metric,
        tickformat=".2f",
        title_font=dict(size=18),
        tickfont=dict(size=14),
        row=idx,
        col=1
    )

# === X-axis formatting ===
fig.update_xaxes(
    title_text="CSV File",
    tickangle=45,
    tickfont=dict(size=14),
    row=len(metrics),
    col=1
)

fig.show()
