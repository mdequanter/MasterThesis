import pandas as pd
import plotly.graph_objects as go

# === Load CSV ===
csv_path = "unrealsim\\analytics\\pingAnalytics.csv"
df = pd.read_csv(csv_path)

# === Convert timestamp ===
#df['timestamp'] = pd.to_datetime(df['timestamp'])
df['timestamp'] = pd.to_datetime(df['timestamp'], format='%Y-%m-%dT%H:%M:%S.%f')


# === Create time groupings ===
df['minute'] = df['timestamp'].dt.floor('T')  # floor to minute
df['hour'] = df['timestamp'].dt.floor('H')    # floor to hour

# === Group and average ===
minute_summary = df.groupby(['minute', 'host', 'description'])['mean_latency_ms'].mean().reset_index()
hourly_summary = df.groupby(['hour', 'host', 'description'])['mean_latency_ms'].mean().reset_index()

# === Plot function ===
def plot_latency(summary_df, time_col, title):
    fig = go.Figure()
    for (host, desc), group in summary_df.groupby(['host', 'description']):
        fig.add_trace(go.Scatter(
            x=group[time_col],
            y=group['mean_latency_ms'],
            mode="lines+markers",
            name=f"{host} ({desc})"
        ))

    fig.update_layout(
        title=title,
        xaxis_title="Time",
        yaxis_title="Mean Latency (ms)",
        template="plotly_white",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        ),
        font=dict(size=14)
    )
    fig.show()

# === Plot the graphs ===
plot_latency(minute_summary, 'minute', 'Ping Latency Over Time (Minute Resolution)')
plot_latency(hourly_summary, 'hour', 'Ping Latency Over Time (Hourly Resolution)')
