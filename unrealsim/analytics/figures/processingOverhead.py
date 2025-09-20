import pandas as pd
import scipy.stats as stats
import plotly.graph_objects as go
from pathlib import Path
import re
import numpy as np

folder_path = Path("unrealsim/analytics/external")
all_files = sorted(folder_path.glob("*.csv"), key=lambda f: f.name.lower())
if not all_files:
    print(f"No CSV files found in {folder_path}")
    raise SystemExit(1)

# === Default title from filenames ===
prefixes = set()
for f in all_files:
    m = re.match(r"(.+?)_\d{8}_\d{6}\.csv$", f.name)
    prefixes.add(m.group(1) if m else f.stem)
default_title = prefixes.pop() if len(prefixes) == 1 else folder_path.name

title_input = input(f"Enter a title for the chart [{default_title}]: ").strip()
final_title = title_input if title_input else default_title

# === Metric labels (no latency, no per-kB) ===
metric_labels = {
    "compression_ms": "Compression (ms)",
    "encryption_ms": "Encryption (ms)",
    "proc_overhead_ms": "Processing Overhead (ms)",
    "proc_overhead_share_pct": "Overhead Share (% of latency)",
}

ordered_metrics = [
    "compression_ms",
    "encryption_ms",
    "proc_overhead_ms",
    "proc_overhead_share_pct",
]

def compute_ci(series: pd.Series):
    s = series.dropna().astype(float)
    n = len(s)
    if n == 0:
        return np.nan, np.nan
    mean = s.mean()
    h = stats.sem(s) * stats.t.ppf(0.975, n - 1) if n > 1 else 0.0
    return round(float(mean), 3), round(float(h), 3)

#color_palette = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
color_palette = [
    "#00CC96", "#B6E880", "#EF553B", "#FFA15A",
    "#636EFA", "#AB63FA", "#19D3F3", "#FF6692",
    "#FF97FF", "#FECB52",
]
fig = go.Figure()

for idx, file in enumerate(all_files):
    df = pd.read_csv(file)

    for col in ["latency_ms", "compression_ms", "encryption_ms"]:
        if col not in df.columns:
            raise ValueError(f"Missing column '{col}' in {file.name}")

    df = df.copy()
    df["proc_overhead_ms"] = df["compression_ms"] + df["encryption_ms"]

    lat = df["latency_ms"].replace([np.inf, -np.inf], np.nan)
    df["proc_overhead_share_pct"] = np.where(lat > 0, 100.0 * df["proc_overhead_ms"] / lat, np.nan)

    records = []
    for metric in ordered_metrics:
        mean, ci = compute_ci(df[metric])
        records.append({
            "Metric": metric_labels[metric],
            "Mean": mean,
            "95% CI": ci
        })

    ci_df = pd.DataFrame(records)
    ci_df["Label"] = ci_df.apply(
        lambda row: f"{row['Mean']} ± {row['95% CI']}" if pd.notna(row["Mean"]) else "n/a",
        axis=1
    )

    color = color_palette[idx % len(color_palette)]
    legend_name = file.stem

    fig.add_trace(
        go.Bar(
            x=ci_df["Metric"],
            y=ci_df["Mean"],
            error_y=dict(type="data", array=ci_df["95% CI"], visible=True),
            marker_color=color,
            text=ci_df["Label"],
            textposition="auto",
            textfont=dict(size=14, color="white"),
            name=legend_name,
        )
    )

fig.update_layout(
    title=f"{final_title} — Processing Overhead (mean ± 95% CI)",
    template="plotly_white",
    font=dict(size=16),
    title_font=dict(size=22),
    width=1100,
    xaxis_title="Metric",
    yaxis_title="Value",
    barmode="group",
    bargap=0.08,
    bargroupgap=0.02,
    legend=dict(
        orientation="h",
        yanchor="top",
        y=-0.25,
        xanchor="center",
        x=0.5,
        font=dict(size=14)
    )
)

fig.update_yaxes(tickfont=dict(size=14))
fig.show()
