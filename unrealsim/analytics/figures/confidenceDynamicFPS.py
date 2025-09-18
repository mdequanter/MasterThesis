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
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {directory}")
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

# === Function to compute 95% confidence interval ===
def compute_ci(series: pd.Series):
    n = len(series)
    mean = series.mean()
    h = stats.sem(series) * stats.t.ppf(0.975, n - 1) if n > 1 else 0
    return round(mean, 2), round(h, 2)

# === Group by max_fps and compute mean + CI for latency_ms, queue size, and bitrate ===
records = []
for fps_value, group in df.groupby("max_fps"):
    latency_mean, latency_ci = compute_ci(group["latency_ms"])
    queue_mean = round(group["queuesize"].mean(), 2)
    # If your CSV 'bitrate' is in kbps, divide by 1000 to get Mbps. Remove the division if already Mbps.
    bitrate_mbps = round(group["bitrate"].mean(), 3)
    records.append({
        "max_fps": fps_value,
        "LatencyMean": latency_mean,
        "Latency95CI": latency_ci,
        "QueueMean": queue_mean,
        "BitrateMbps": bitrate_mbps,
        "Label": f"{latency_mean} ± {latency_ci} | {bitrate_mbps} Mbps"
    })

ci_df = pd.DataFrame(records).sort_values("max_fps")

# === Create figure ===
fig = go.Figure()

# --- Bar trace for Latency ---
fig.add_trace(
    go.Bar(
        x=ci_df["max_fps"].astype(str),
        y=ci_df["LatencyMean"],
        error_y=dict(type="data", array=ci_df["Latency95CI"], visible=True),
        marker_color="royalblue",
        text=ci_df["Label"],
        textposition="outside",
        name="Average Latency (ms)",
        textfont=dict(size=14)
    )
)

# --- Line trace for Queue Size (y2) ---
fig.add_trace(
    go.Scatter(
        x=ci_df["max_fps"].astype(str),
        y=ci_df["QueueMean"],
        mode="lines+markers",
        line=dict(color="firebrick", width=3),
        marker=dict(size=10),
        name="Average Queue Size",
        yaxis="y2"
    )
)

# --- Line trace for Bitrate (Mbps) on same axis (y2) ---
fig.add_trace(
    go.Scatter(
        x=ci_df["max_fps"].astype(str),
        y=ci_df["BitrateMbps"],
        mode="lines+markers",
        line=dict(color="green", width=3, dash="dot"),
        marker=dict(size=10),
        name="Average Bitrate (Mbps)",
        yaxis="y2"
    )
)

# === Layout ===
fig.update_layout(
    title=f"{final_title} — Latency, Queue Size, and Bitrate per FPS",
    template="plotly_white",
    font=dict(size=16),
    title_font=dict(size=22),
    xaxis_title="max_fps",
    yaxis=dict(title="Latency (ms)", tickformat=".2f"),
    yaxis2=dict(title="Queue Size / Bitrate (Mbps)", overlaying="y", side="right"),
    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
)

fig.show()
