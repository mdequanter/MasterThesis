import os
import time
import torch
from ultralytics import YOLO

# ---------- config ----------
ckpt_path = "D:\\github\\MasterThesis\\unrealsim\\models\\unrealsimLocal.pt"
imgsz = 640
runs = 1000         # meer runs = stabieler; pas aan als je wil
warmup = 50
device = "cuda" if torch.cuda.is_available() else "cpu"

print(torch.__version__)
print(torch.version.cuda)
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu")
print(torch.backends.cudnn.version())

# ---------- load ----------
print(f"Checkpoint file size: {os.path.getsize(ckpt_path) / (1024**2):.2f} MB")

yolo = YOLO(ckpt_path)
nn_model_fp32 = yolo.model.to(device).eval()

# ---------- parameter stats ----------
total_params = sum(p.numel() for p in nn_model_fp32.parameters())
param_mb = (total_params * 4) / (1024**2)  # fp32

print("\n=== Parameter stats ===")
print(f"Total parameters: {total_params:,}")
print(f"Param memory (fp32): {param_mb:.2f} MB")

# ---------- computation cost (MACs/FLOPs) ----------
x_fp32 = torch.zeros(1, 3, imgsz, imgsz, device=device)

macs = flops = None
try:
    from thop import profile
    flops, params_thop = profile(nn_model_fp32, inputs=(x_fp32,), verbose=False)
    macs = flops / 2  # common convention
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

# ---------- benchmark helper ----------
def benchmark_model(model, x, runs=200, warmup=50, device="cuda"):
    torch.backends.cudnn.benchmark = True if device == "cuda" else False

    # warmup
    with torch.inference_mode():
        for _ in range(warmup):
            _ = model(x)

    if device == "cuda":
        starter = torch.cuda.Event(enable_timing=True)
        ender   = torch.cuda.Event(enable_timing=True)

        torch.cuda.synchronize()
        starter.record()
        with torch.inference_mode():
            for _ in range(runs):
                _ = model(x)
        ender.record()

        torch.cuda.synchronize()
        total_ms = starter.elapsed_time(ender)
        avg_ms = total_ms / runs
    else:
        times_s = []
        with torch.inference_mode():
            for _ in range(runs):
                t0 = time.perf_counter()
                _ = model(x)
                t1 = time.perf_counter()
                times_s.append(t1 - t0)
        avg_ms = (sum(times_s) / len(times_s)) * 1000

    fps = 1000.0 / avg_ms
    return avg_ms, fps

# ---------- inference benchmark (FP32) ----------
avg_ms, fps = benchmark_model(nn_model_fp32, x_fp32, runs=runs, warmup=warmup, device=device)

print("\n=== Inference benchmark (FP32) ===")
print(f"Device: {device}")
print(f"Runs: {runs}")
print(f"Avg time per image: {avg_ms:.2f} ms")
print(f"Throughput: {fps:.2f} FPS")

# ---------- inference benchmark (FP16, if CUDA) ----------
if device == "cuda":
    nn_model_fp16 = yolo.model.to(device).eval().half()
    x_fp16 = x_fp32.half()

    avg_ms16, fps16 = benchmark_model(nn_model_fp16, x_fp16, runs=runs, warmup=warmup, device=device)

    print("\n=== Inference benchmark (FP16) ===")
    print(f"Device: {device}")
    print(f"Runs: {runs}")
    print(f"Avg time per image: {avg_ms16:.2f} ms")
    print(f"Throughput: {fps16:.2f} FPS")
