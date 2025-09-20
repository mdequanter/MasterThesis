import pandas as pd
import scipy.stats as stats
import plotly.graph_objects as go
from pathlib import Path
import re

# === Directory containing all CSV files ===
folder_path = Path("unrealsim/analytics/external")

# === Load all CSV files ===
all_files = list(folder_path.glob("*.csv"))

if not all_files:
    print(f"No CSV files found in {folder_path}")
    exit(1)

# For title: if all files share the same prefix, use that, else fallback to folder name
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

# Prompt FPS range
while True:
    try:
        min_fps = int(input("Enter minimum FPS value (inclusive): ").strip())
        max_fps = int(input("Enter maximum FPS value (inclusive): ").strip())
        if min_fps > max_fps:
            print("Minimum FPS cannot be greater than maximum FPS. Please try again.")
        else:
            break
    except ValueError:
        print("Invalid input. Please enter integer values.")

# Function to compute 95% confidence interval
def compute_ci(series):
    n = len(series)
    mean = series.mean()
    h = stats.sem(series) * stats.t.ppf(0.975, n - 1) if n > 1 else 0
    return round(mean, 2), round(h, 2)

# Create figure
fig = go.Figure()

# Define colors for consistency
color_palette = [
    "#00CC96", "#B6E880", "#EF553B", "#FFA15A",
    "#636EFA", "#AB63FA", "#19D3F3", "#FF6692",
    "#FF97FF", "#FECB52",
]

# Process each file
for idx, file in enumerate(all_files):
    df = pd.read_csv(file)
    df_filtered = df[(df["max_fps"] >= min_fps) & (df["max_fps"] <= max_fps)]

    if df_filtered.empty:
        print(f"File '{file.name}' has no data in the specified FPS range.")
        continue

    records = []
    for fps_value, group in df_filtered.groupby("max_fps"):
        latency_mean, latency_ci = compute_ci(group["latency_ms"])
        records.append({
            "max_fps": fps_value,
            "LatencyMean": latency_mean,
            "Latency95CI": latency_ci,
            "Label": f"{latency_mean} ± {latency_ci}"
        })

    ci_df = pd.DataFrame(records).sort_values("max_fps")

    legend_name = file.stem
    color = color_palette[idx % len(color_palette)]

    # Bar trace for Latency
    fig.add_trace(
        go.Bar(
            x=ci_df["max_fps"].astype(str),
            y=ci_df["LatencyMean"],
            error_y=dict(
                type="data",
                array=ci_df["Latency95CI"],
                visible=True
            ),
            text=ci_df["Label"],
            textposition="inside",
            insidetextanchor="middle",
            name=f"Latency ({legend_name})",
            marker_color=color,
            textfont=dict(size=16, color="white")
        )
    )

# Add horizontal threshold line at 200 ms
fig.add_shape(
    type="line",
    x0=-0.5,
    x1=len(ci_df["max_fps"].unique()) - 0.5,
    y0=200,
    y1=200,
    line=dict(color="black", width=2, dash="dash")
)

# Add annotation label for threshold
fig.add_annotation(
    x=0,
    y=200,
    xref="x",
    yref="y",
    text="Latency Threshold (200 ms)",
    showarrow=False,
    font=dict(size=14, color="black"),
    align="center",
    bgcolor="white",
    bordercolor="black",
    borderwidth=0
)

# Layout (no second y-axis anymore)
fig.update_layout(
    template="plotly_white",
    font=dict(size=16),
    xaxis_title="max_fps",
    yaxis=dict(
        title="Latency (ms)",
        tickformat=".2f"
    ),
    legend=dict(
        orientation="h",
        yanchor="bottom",
        y=1.02,
        xanchor="right",
        x=1
    )
)

fig.show()
