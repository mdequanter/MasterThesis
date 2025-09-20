import pandas as pd
import scipy.stats as stats
import plotly.graph_objects as go
from pathlib import Path
import re
import numpy as np

# === Data folder ===
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
default_title = "Power usage comparison" if len(prefixes) != 1 else f"{prefixes.pop()} — Power usage"

title_input = input(f"Enter a title for the chart [{default_title}]: ").strip()
final_title = title_input if title_input else default_title

def mean_ci95(series: pd.Series):
    s = pd.to_numeric(series, errors="coerce").dropna().astype(float)
    n = len(s)
    if n == 0:
        return np.nan, np.nan
    m = s.mean()
    h = stats.sem(s) * stats.t.ppf(0.975, n - 1) if n > 1 else 0.0
    return float(m), float(h)

# === Collect per-file stats ===
rows = []
for f in all_files:
    df = pd.read_csv(f)
    if "poweruse_W" not in df.columns:
        raise ValueError(f"Missing column 'poweruse_W' in {f.name}")
    mean_w, ci_w = mean_ci95(df["poweruse_W"])
    rows.append({
        "System": f.stem,
        "Mean_W": None if np.isnan(mean_w) else round(mean_w, 3),
        "CI95_W": None if np.isnan(ci_w) else round(ci_w, 3),
    })

res = pd.DataFrame(rows)

# === Build labels ===
labels = [
    f"{row['Mean_W']} W ± {row['CI95_W']}"
    for _, row in res.iterrows()
]

# === Color palette ===
color_palette = [
    "#00CC96", "#B6E880", "#EF553B", "#FFA15A",
    "#636EFA", "#AB63FA", "#19D3F3", "#FF6692",
    "#FF97FF", "#FECB52",
]

# === Plot ===
fig = go.Figure()
for idx, row in res.iterrows():
    fig.add_trace(
        go.Bar(
            x=["Power use"],
            y=[row["Mean_W"]],
            error_y=dict(type="data", array=[row["CI95_W"]], visible=True),
            text=[labels[idx]],
            textposition="auto",
            textfont=dict(size=14, color="white"),
            marker_color=color_palette[idx % len(color_palette)],
            name=row["System"],
        )
    )

fig.update_layout(
    title=f"{final_title} — Mean poweruse_W (±95% CI)",
    template="plotly_white",
    font=dict(size=16),
    title_font=dict(size=22),
    width=1100,
    xaxis_title="Metric",
    yaxis_title="Power (W)",
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
