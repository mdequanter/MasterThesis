import pandas as pd
import scipy.stats as stats
import plotly.graph_objects as go
from pathlib import Path
import re
import os

directory = "unrealsim/analytics"

# === Prompt user for CSV file ===
user_input = input("Enter the path to the CSV file (leave blank for latest): ").strip()
if user_input:
    file_path = user_input
else:
    csv_files = [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith(".csv")]
    file_path = max(csv_files, key=os.path.getctime)
    print("Automatically selected:", file_path)

# === Extract default title from filename ===
filename = Path(file_path).stem
match = re.match(r"(.+?)_\d{8}_\d{6}$", filename)
default_title = match.group(1) if match else filename
title_input = input(f"Enter a title for the chart [{default_title}]: ").strip()
final_title = title_input if title_input else default_title

# === Load CSV ===
df = pd.read_csv(file_path)

# === Columns to use ===
mean_ci_metrics = {
    "bitrate": "Bitrate (Mbps)",
    "avg_fps": "Average FPS",
}
max_metrics = {
    "missed_frames": "Missed Frames (max /1000)",
    "successful_frames": "Successful Frames (max /1000)",
}

# === 95% CI helper ===
def mean_ci(series):
    series = pd.to_numeric(series, errors="coerce").dropna()
    n = len(series)
    if n == 0:
        return 0.0, 0.0
    m = series.mean()
    h = stats.sem(series) * stats.t.ppf(0.975, n - 1) if n > 1 else 0.0
    return round(m, 2), round(h, 2)

# === Build records ===
bars = []

# Mean ± CI metrics
for col, label in mean_ci_metrics.items():
    if col in df.columns:
        m, ci = mean_ci(df[col])
        bars.append({
            "Metric": label,
            "Value": m,
            "CI": ci,
            "Text": f"{m} ± {ci}"
        })

# Max metrics for frames (scaled down by 1000)
for col, label in max_metrics.items():
    if col in df.columns:
        val = pd.to_numeric(df[col], errors="coerce").dropna().max()
        val = 0.0 if pd.isna(val) else float(val) / 1000.0
        bars.append({
            "Metric": label,
            "Value": val,
            "CI": 0.0,
            "Text": f"max {val:.2f}k"
        })

# === Plot ===
fig = go.Figure()
for b in bars:
    fig.add_trace(go.Bar(
        x=[b["Metric"]],
        y=[b["Value"]],
        error_y=dict(type="data", array=[b["CI"]], visible=bool(b["CI"])),
        text=[b["Text"]],
        textposition="outside",
        name=b["Metric"]
    ))

fig.update_layout(
    title=f"{final_title} — Frames (/1000) and Bitrate/FPS (mean ± 95% CI)",
    template="plotly_white",
    font=dict(size=16),
    title_font=dict(size=22),
    xaxis_title="Metric",
    yaxis_title="Value",
    showlegend=False
)
fig.update_yaxes(tickformat=".2f")
fig.show()
