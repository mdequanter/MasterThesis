import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pathlib import Path
import re

folder = Path("unrealsim/analytics/inference")
files = sorted(folder.glob("*.csv"), key=lambda f: f.name.lower())
if not files:
    raise SystemExit(f"No CSV files in {folder}")

def parse_quality_from_name(name: str):
    # matches q90, quality_75, jpegq-50, etc.
    for pat in [r"q(?:uality)?[_-]?(\d{1,3})", r"jpeg[_-]?q(?:uality)?[_-]?(\d{1,3})"]:
        m = re.search(pat, name, flags=re.I)
        if m:
            try:
                return float(m.group(1))
            except:
                return np.nan
    return np.nan

def success_pct_from_df(df: pd.DataFrame) -> float:
    s = pd.to_numeric(df.get("successful_frames"), errors="coerce").max()
    m = pd.to_numeric(df.get("missed_frames"), errors="coerce").max()
    if pd.isna(s) or pd.isna(m) or (s + m) == 0:
        return np.nan
    return 100.0 * s / (s + m)

rows = []
for f in files:
    df = pd.read_csv(f)
    df = df.rename(columns={c: c.strip().lower().replace(" ", "_") for c in df.columns})

    # locate size_kb column variants
    size_col = None
    for cand in ["size_kb", "current_size_kb", "current_size__kb", "current_sizekb"]:
        if cand in df.columns:
            size_col = cand
            break
    if size_col is None:
        raise ValueError(f"{f.name} misses a 'size_kb' column (or variant)")

    needed = ["successful_frames", "missed_frames"]
    miss = [c for c in needed if c not in df.columns]
    if miss:
        raise ValueError(f"{f.name} misses columns: {miss}")

    if "jpeg_quality" not in df.columns:
        df["jpeg_quality"] = parse_quality_from_name(f.stem)

    df["jpeg_quality"] = pd.to_numeric(df["jpeg_quality"], errors="coerce")
    df = df[(df["jpeg_quality"] >= 10) & (df["jpeg_quality"] <= 100)]

    for q, g in df.groupby("jpeg_quality"):
        rows.append({
            "quality": float(q),
            "size_kb_mean": pd.to_numeric(g[size_col], errors="coerce").mean(),
            "success_pct": success_pct_from_df(g)
        })

if not rows:
    raise SystemExit("No data within 10–100% quality range found")

agg = pd.DataFrame(rows)
# average per quality across files (equal weight per file)
agg = agg.groupby("quality", as_index=False).agg(
    size_kb_mean=("size_kb_mean", "mean"),
    success_pct=("success_pct", "mean")
).sort_values("quality")

fig = make_subplots(specs=[[{"secondary_y": True}]])

fig.add_trace(
    go.Scatter(
        x=agg["quality"], y=agg["size_kb_mean"],
        mode="lines+markers", name="Avg. size (kB)",
        hovertemplate="Q=%{x:.0f}%<br>Size=%{y:.1f} kB"
    ),
    secondary_y=False
)

fig.add_trace(
    go.Scatter(
        x=agg["quality"], y=agg["success_pct"],
        mode="lines+markers", name="Avg. success (%)",
        hovertemplate="Q=%{x:.0f}%<br>Success=%{y:.1f}%"
    ),
    secondary_y=True
)

fig.update_layout(
    title="Inference time and Frame level reliability. JPEG Quality (10–100%)",
    template="plotly_white", width=950, height=480,
    legend=dict(orientation="h", yanchor="top", y=-0.15, xanchor="center", x=0.5)
)

fig.update_xaxes(title="JPEG quality (%)")
fig.update_yaxes(title_text="Avg. size (kB)", secondary_y=False, rangemode="tozero")
fig.update_yaxes(title_text="Avg. success (%)", secondary_y=True, range=[0, 100])

fig.show()
