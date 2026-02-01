import os
import time
import torch
from ultralytics import YOLO

# ---------- config ----------
ckpt_path = "models/unrealsim.pt"
imgsz = 640
runs = 50
warmup = 10
device = "cuda" if torch.cuda.is_available() else "cpu"

# ---------- load ----------
print(f"Checkpoint file size: {os.path.getsize(ckpt_path) / (1024**2):.2f} MB")

yolo = YOLO(ckpt_path)
nn_model = yolo.model.to(device).eval()

# ---------- parameter stats ----------
total_params = sum(p.numel() for p in nn_model.parameters())
param_mb = (total_params * 4) / (1024**2)  # fp32

print("\n=== Parameter stats ===")
print(f"Total parameters: {total_params:,}")
print(f"Param memory (fp32): {param_mb:.2f} MB")

# ---------- computation cost (MACs/FLOPs) ----------
x = torch.zeros(1, 3, imgsz, imgsz, device=device)

macs = flops = None
try:
    from thop import profile
    # thop returns FLOPs for many layers; for conv nets it's often "MACs*2" style dependent.
    flops, params_thop = profile(nn_model, inputs=(x,), verbose=False)
    # If you want "MACs" like in your example, treat MACs as FLOPs/2 (common convention)
    macs = flops / 2
except Exception as e:
    print("\n[Info] thop not available or failed. Install with: pip install thop")
    print(f"[Info] thop error: {e}")

print("\n=== Computation cost ===")
print(f"Input size: 1 x 3 x {imgsz} x {imgsz}")
if macs is not None and flops is not None:
    print(f"MACs: {macs/1e9:.2f} GMac")
    print(f"Approx FLOPs: {flops/1e9:.2f} GFLOPs (assuming 2 FLOPs/MAC)")
else:
    print("MACs: (not computed)")
    print("Approx FLOPs: (not computed)")

# ---------- inference benchmark ----------
# Use a realistic input; keep it fp32 unless you're explicitly using AMP
torch.backends.cudnn.benchmark = True if device == "cuda" else False

# Warmup (important for stable timings)
with torch.inference_mode():
    for _ in range(warmup):
        _ = nn_model(x)

if device == "cuda":
    starter = torch.cuda.Event(enable_timing=True)
    ender = torch.cuda.Event(enable_timing=True)

    torch.cuda.synchronize()
    times_ms = []
    with torch.inference_mode():
        for _ in range(runs):
            starter.record()
            _ = nn_model(x)
            ender.record()
            torch.cuda.synchronize()
            times_ms.append(starter.elapsed_time(ender))

    avg_ms = sum(times_ms) / len(times_ms)
else:
    # CPU fallback timing
    times_s = []
    with torch.inference_mode():
        for _ in range(runs):
            t0 = time.perf_counter()
            _ = nn_model(x)
            t1 = time.perf_counter()
            times_s.append(t1 - t0)
    avg_ms = (sum(times_s) / len(times_s)) * 1000

fps = 1000.0 / avg_ms

print("\n=== Inference benchmark ===")
print(f"Device: {device}")
print(f"Runs: {runs}")
print(f"Avg time per image: {avg_ms:.2f} ms")
print(f"Throughput: {fps:.2f} FPS")
