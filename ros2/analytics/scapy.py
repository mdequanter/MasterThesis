# Real-time packet counter with size and CSV logging
# command:  sudo python3 scapy.py --iface lo

import argparse
from scapy.all import sniff
import time
import threading
import sys
import termios
import tty
import csv
import os
import psutil  # <-- ADD THIS

packet_count = 0
byte_count = 0
running = True

# === Parse command-line arguments ===
parser = argparse.ArgumentParser(description="Real-time packet counter with size and CSV logging")
parser.add_argument('--iface', type=str, required=True, help='Network interface to monitor (e.g., wlan0, eth0)')
args = parser.parse_args()
interface = args.iface

# CSV file setup
csv_filename = f"ros2/analytics/logs/packet_log_{interface}.csv"

# === Packet handler ===
def count_packet(pkt):
    global packet_count, byte_count
    packet_count += 1
    byte_count += len(pkt)

# === Keyboard listener (press 'q' to quit) ===
def key_listener():
    global running
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while running:
            ch = sys.stdin.read(1)
            if ch.lower() == 'q':
                running = False
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# === Create CSV file with headers if needed ===
if not os.path.isfile(csv_filename):
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Interface", "PacketsPerSecond", "TotalBytes", "AvgPacketSize", "CPU_Usage_Percent"])

# === Start key listener thread ===
threading.Thread(target=key_listener, daemon=True).start()

print(f"📡 Monitoring interface: {interface}")
print("Press 'q' to stop.\n")

# === Main loop ===
while running:
    packet_count = 0
    byte_count = 0
    sniff(prn=count_packet, store=False, iface=interface, timeout=1)
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    avg_packet_size = byte_count / packet_count if packet_count > 0 else 0
    cpu_usage = psutil.cpu_percent(interval=None)  # <-- CPU usage percentage

    print(f"[{timestamp}] {interface} → Packets/sec: {packet_count}, Bytes/sec: {byte_count}, "
          f"Avg size: {avg_packet_size:.1f} B, CPU: {cpu_usage:.1f}%")

    # Append to CSV
    with open(csv_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, interface, packet_count, byte_count, round(avg_packet_size, 1), cpu_usage])
        
print(f"\n✅ Packet counting stopped. CSV saved to: {csv_filename}")
