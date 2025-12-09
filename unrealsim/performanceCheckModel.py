import os
import time

import torch

from ultralytics import YOLO
from thop import profile


def get_param_stats(model: torch.nn.Module):
    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)

    param_bytes = total_params * 4  # assume float32
    param_mb = param_bytes / (1024 ** 2)

    print("=== Parameter stats ===")
    print(f"Total parameters:      {total_params:,}")
    print(f"Trainable parameters:  {trainable_params:,}")
    print(f"Param memory (fp32):   {param_mb:.2f} MB")
    print()


def get_flops(model: torch.nn.Module, img_size=(640, 640), device="cpu"):
    model.eval()
    h, w = img_size
    dummy = torch.randn(1, 3, h, w).to(device)

    # thop returns MACs; many papers report MACs as "FLOPs-ish"
    macs, params = profile(model, inputs=(dummy,), verbose=False)
    macs /= 1e9  # to GMac
    flops = macs * 2  # approx, if you really want "FLOPs"

    print("=== Computation cost ===")
    print(f"Input size:            1 x 3 x {h} x {w}")
    print(f"MACs:                  {macs:.2f} GMac")
    print(f"Approx FLOPs:          {flops:.2f} GFLOPs (assuming 2 FLOPs/MAC)")
    print()


def benchmark_inference(model: torch.nn.Module, img_size=(640, 640), device="cpu", runs=50):
    model.eval()
    h, w = img_size
    x = torch.randn(1, 3, h, w).to(device)

    # Warm-up
    with torch.no_grad():
        for _ in range(5):
            _ = model(x)

    if device == "cuda":
        torch.cuda.synchronize()

    start = time.time()
    with torch.no_grad():
        for _ in range(runs):
            _ = model(x)
        if device == "cuda":
            torch.cuda.synchronize()
    end = time.time()

    total_time = end - start
    avg_time = total_time / runs
    fps = 1.0 / avg_time

    print("=== Inference benchmark ===")
    print(f"Device:                {device}")
    print(f"Runs:                  {runs}")
    print(f"Avg time per image:    {avg_time * 1000:.2f} ms")
    print(f"Throughput:            {fps:.2f} FPS")
    print()


def measure_gpu_peak_memory(model: torch.nn.Module, img_size=(640, 640), device="cuda"):
    if device != "cuda" or not torch.cuda.is_available():
        print("GPU not available; skipping GPU memory measurement.")
        return

    model.eval()
    h, w = img_size
    x = torch.randn(1, 3, h, w).to(device)

    torch.cuda.reset_peak_memory_stats(device)
    with torch.no_grad():
        _ = model(x)
        torch.cuda.synchronize()

    peak_bytes = torch.cuda.max_memory_allocated(device)
    peak_mb = peak_bytes / (1024 ** 2)

    print("=== GPU memory usage (peak during single inference) ===")
    print(f"Peak allocated:        {peak_mb:.2f} MB")
    print()


def main():
    ckpt_path = "models/unrealsim.pt"  # adjust this
    img_size = (640, 640)             # adjust if you trained with a different size
    device = "cuda" if torch.cuda.is_available() else "cpu"

    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(f"Checkpoint not found at: {ckpt_path}")

    print(f"Loading model from:    {ckpt_path}")
    model_wrapper = YOLO(ckpt_path)
    nn_model = model_wrapper.model.to(device)

    # 1) Parameter stats
    get_param_stats(nn_model)

    # 2) FLOPs / MACs
    get_flops(nn_model, img_size=img_size, device=device)

    # 3) Inference time / FPS
    benchmark_inference(nn_model, img_size=img_size, device=device, runs=50)

    # 4) Peak GPU memory during inference (if CUDA)
    measure_gpu_peak_memory(nn_model, img_size=img_size, device=device)


if __name__ == "__main__":
    main()
