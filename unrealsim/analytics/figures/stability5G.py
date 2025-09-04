import pandas as pd
import plotly.graph_objects as go

# === Load CSV files ===
laptop_df = pd.read_csv(r"unrealsim\\analytics\\stability\\Laptop_5G_Vlietland.csv")
rpi_df = pd.read_csv(r"unrealsim\\analytics\\stability\\Laptop_5G_Vlietland.csv")

# Convert datetime column
for df in [rpi_df]:
    if 'datetime' in df.columns:
        df['datetime'] = pd.to_datetime(df['datetime'])

# Function to create time series graphs
def create_time_graph(df, title_prefix):
    fig = go.Figure()
    metrics = ["avg_latency_ms", "inference_ms"]
    colors = ["#636EFA", "#00CC96"]

    for metric, color in zip(metrics, colors):
        if metric in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=df["datetime"],
                    y=df[metric],
                    mode="lines+markers",
                    name=metric,
                    line=dict(color=color, width=2)
                )
            )
    
    fig.update_layout(
        title=f"{title_prefix} - Time Series",
        xaxis_title="Timestamp",
        yaxis_title="Value",
        template="plotly_white",
        font=dict(size=14),
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        )
    )
    fig.show()

# === Generate plots ===
create_time_graph(laptop_df, "Laptop 5G")
#create_time_graph(rpi_df, "Raspberry Pi 5G")
