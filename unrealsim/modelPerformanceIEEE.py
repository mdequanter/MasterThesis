import os
import time
from pathlib import Path

import torch
from ultralytics import YOLO
from thop import profile


def format_mb(num_bytes: int) -> str:
    return f"{num_bytes / (1024 ** 2):.2f} MB"


def get_checkpoint_size(ckpt_path: str) -> str:
    size_bytes = os.path.getsize(ckpt_path)
    return format_mb(size_bytes)


def get_software_hardware_info(ckpt_path: str, device: str):
    rows = []

    rows.append(("Software & Hardware", ""))
    rows.append(("PyTorch version", torch.__version__))
    rows.append(("CUDA version", torch.version.cuda if torch.version.cuda else "Not available"))

    if device == "cuda":
        rows.append(("GPU", torch.cuda.get_device_name(0)))
    else:
        rows.append(("GPU", "CUDA not available"))

    rows.append(("Checkpoint size", get_checkpoint_size(ckpt_path)))

    return rows


def get_param_stats(model: torch.nn.Module):
    total_params = sum(p.numel() for p in model.parameters())
    param_bytes_fp32 = total_params * 4

    rows = []
    rows.append(("Model statistics", ""))
    rows.append(("Total parameters", f"{total_params:,}"))
    rows.append(("Parameter memory (FP32)", format_mb(param_bytes_fp32)))

    return rows


def get_flops(model: torch.nn.Module, img_size=(640, 640), device="cpu"):
    model.eval()

    h, w = img_size
    dummy = torch.randn(1, 3, h, w).to(device)

    with torch.no_grad():
        macs, _ = profile(model, inputs=(dummy,), verbose=False)

    gmacs = macs / 1e9
    gflops = gmacs * 2

    rows = []
    rows.append(("Computation cost", ""))
    rows.append(("Input resolution", f"1 × 3 × {h} × {w}"))
    rows.append(("MACs", f"{gmacs:.2f} GMac"))
    rows.append(("Approx. FLOPs", f"{gflops:.2f} GFLOPs"))

    return rows


def benchmark_inference_fp32(
    model: torch.nn.Module,
    img_size=(640, 640),
    device="cpu",
    warmup_runs=20,
    benchmark_runs=200,
):
    model.eval()

    h, w = img_size
    x = torch.randn(1, 3, h, w).to(device)

    with torch.no_grad():
        for _ in range(warmup_runs):
            _ = model(x)

    if device == "cuda":
        torch.cuda.synchronize()

        starter = torch.cuda.Event(enable_timing=True)
        ender = torch.cuda.Event(enable_timing=True)

        timings = []

        with torch.no_grad():
            for _ in range(benchmark_runs):
                starter.record()
                _ = model(x)
                ender.record()

                torch.cuda.synchronize()
                timings.append(starter.elapsed_time(ender))

        latency_ms = sum(timings) / len(timings)

    else:
        start = time.perf_counter()

        with torch.no_grad():
            for _ in range(benchmark_runs):
                _ = model(x)

        end = time.perf_counter()
        latency_ms = ((end - start) / benchmark_runs) * 1000

    fps = 1000.0 / latency_ms

    return latency_ms, fps


def benchmark_inference_fp16(
    model: torch.nn.Module,
    img_size=(640, 640),
    device="cpu",
    warmup_runs=20,
    benchmark_runs=200,
):
    if device != "cuda":
        return None, None

    model.eval()
    model.half()

    h, w = img_size
    x = torch.randn(1, 3, h, w).to(device).half()

    with torch.no_grad():
        for _ in range(warmup_runs):
            _ = model(x)

    torch.cuda.synchronize()

    starter = torch.cuda.Event(enable_timing=True)
    ender = torch.cuda.Event(enable_timing=True)

    timings = []

    with torch.no_grad():
        for _ in range(benchmark_runs):
            starter.record()
            _ = model(x)
            ender.record()

            torch.cuda.synchronize()
            timings.append(starter.elapsed_time(ender))

    latency_ms = sum(timings) / len(timings)
    fps = 1000.0 / latency_ms

    return latency_ms, fps


def get_benchmark_rows(
    model: torch.nn.Module,
    img_size=(640, 640),
    device="cpu",
    warmup_runs=20,
    benchmark_runs=200,
):
    rows = []
    rows.append(("Inference benchmark (batch size = 1)", ""))

    fp32_latency, fp32_fps = benchmark_inference_fp32(
        model=model.float(),
        img_size=img_size,
        device=device,
        warmup_runs=warmup_runs,
        benchmark_runs=benchmark_runs,
    )

    rows.append(("FP32 latency", f"{fp32_latency:.2f} ms"))
    rows.append(("FP32 throughput", f"{fp32_fps:.2f} FPS"))

    fp16_latency, fp16_fps = benchmark_inference_fp16(
        model=model,
        img_size=img_size,
        device=device,
        warmup_runs=warmup_runs,
        benchmark_runs=benchmark_runs,
    )

    if fp16_latency is not None:
        rows.append(("FP16 latency", f"{fp16_latency:.2f} ms"))
        rows.append(("FP16 throughput", f"{fp16_fps:.2f} FPS"))
    else:
        rows.append(("FP16 latency", "Not available on CPU"))
        rows.append(("FP16 throughput", "Not available on CPU"))

    return rows


def print_report(rows):
    col1_width = max(len(str(row[0])) for row in rows)
    col2_width = max(len(str(row[1])) for row in rows)

    print(f"{'Category':<{col1_width}}  Value")
    print(f"{'-' * col1_width}  {'-' * max(col2_width, 5)}")

    for category, value in rows:
        if value == "":
            print(f"{category}")
        else:
            print(f"{category:<{col1_width}}  {value}")


def save_report_as_csv(rows, output_path="model_report.csv"):
    import csv

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Category", "Value"])
        writer.writerows(rows)

    print(f"\nSaved report to: {output_path}")


def main():

    from pathlib import Path

    SCRIPT_DIR = Path(__file__).resolve().parent
    ckpt_path = SCRIPT_DIR / "models" / "unrealsim.pt"


    img_size = (640, 640)

    warmup_runs = 20
    benchmark_runs = 200

    if not Path(ckpt_path).exists():
        raise FileNotFoundError(f"Checkpoint not found at: {ckpt_path}")

    device = "cuda" if torch.cuda.is_available() else "cpu"

    print(f"Loading model from: {ckpt_path}")
    print(f"Using device:       {device}")
    print()

    model_wrapper = YOLO(ckpt_path)
    nn_model = model_wrapper.model.to(device)

    rows = []

    rows.extend(get_software_hardware_info(ckpt_path, device))
    rows.extend(get_param_stats(nn_model))
    rows.extend(get_flops(nn_model.float(), img_size=img_size, device=device))
    rows.extend(
        get_benchmark_rows(
            model=nn_model,
            img_size=img_size,
            device=device,
            warmup_runs=warmup_runs,
            benchmark_runs=benchmark_runs,
        )
    )

    print_report(rows)
    save_report_as_csv(rows, output_path="model_report.csv")


if __name__ == "__main__":
    main()