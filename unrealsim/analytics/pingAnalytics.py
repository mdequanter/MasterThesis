import csv
import os
import subprocess
import time
from datetime import datetime
import platform

# === Prompt for description at startup ===
description = input("Enter a description for this ping session: ").strip()

# === Hosts to ping ===
HOSTS = [
    ("8.8.8.8", "Google_DNS"),
    ("94.111.36.87", "Signaling_Server"),
    ("192.168.0.74", "Local_Signaling"),
    ("192.168.0.78", "Inference_Server")

]

# === CSV file path ===
csv_path = r"unrealsim\\analytics\\pingAnalytics.csv"

# === Ensure directory exists ===
os.makedirs(os.path.dirname(csv_path), exist_ok=True)

# === Create file with header if it doesn't exist ===
if not os.path.isfile(csv_path):
    with open(csv_path, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp",
            "host",
            "mean_latency_ms",
            "description"
        ])

# === Detect OS for correct ping command ===
IS_WINDOWS = platform.system().lower().startswith("win")

# === Function to run a single ping ===
def ping_once(host):
    try:
        if IS_WINDOWS:
            result = subprocess.run(
                ["ping", "-n", "1", "-w", "1000", host],
                capture_output=True,
                text=True
            )
            output = result.stdout
            # Windows parsing
            for line in output.splitlines():
                if "Average =" in line:
                    parts = line.split("Average =")[-1]
                    latency_str = parts.strip().replace("ms", "").strip()
                    return float(latency_str)
                if "time=" in line:
                    idx = line.find("time=")
                    latency_str = line[idx + 5:].split("ms")[0]
                    return float(latency_str)
        else:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", host],
                capture_output=True,
                text=True
            )
            output = result.stdout
            for line in output.splitlines():
                if "time=" in line:
                    idx = line.find("time=")
                    latency_str = line[idx + 5:].split(" ")[0]
                    return float(latency_str)
    except Exception as e:
        print(f"Error pinging {host}:", e)
    return None

# === Main loop ===
while True:
    timestamp = datetime.now().isoformat()
    for host, host_label in HOSTS:
        latencies = []
        for i in range(10):
            latency = ping_once(host)
            if latency is not None:
                latencies.append(latency)
            time.sleep(1)

        if latencies:
            mean_latency = round(sum(latencies) / len(latencies), 2)
        else:
            mean_latency = -1  # indicates failure

        with open(csv_path, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                host_label,
                mean_latency,
                description
            ])

        print(f"[{timestamp}] {host_label}: {mean_latency} ms (saved)")

    # Wait 60 seconds before next measurement
    time.sleep(1)
