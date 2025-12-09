import torch

ckpt_path = "models/unrealsim.pt"  # adapt path

ckpt = torch.load(ckpt_path, map_location="cpu")

print(type(ckpt))
if isinstance(ckpt, dict):
    print("Top-level keys:", ckpt.keys())
    for k, v in ckpt.items():
        if "metric" in k.lower() or "results" in k.lower() or "stats" in k.lower():
            print(f"\nPotential metrics key: {k}")
            print(type(v))
            try:
                print(v)
            except Exception:
                pass

import os

ckpt_path = "models/unrealsim.pt"
size_bytes = os.path.getsize(ckpt_path)
size_mb = size_bytes / (1024 ** 2)
print(f"Checkpoint file size: {size_mb:.2f} MB")

from ultralytics import YOLO

ckpt_path = "models/unrealsim.pt"

model = YOLO(ckpt_path)        # loads your trained model
nn_model = model.model         # underlying torch.nn.Module

total_params = sum(p.numel() for p in nn_model.parameters())
trainable_params = sum(p.numel() for p in nn_model.parameters() if p.requires_grad)

print(f"Total parameters:     {total_params:,}")
print(f"Trainable parameters: {trainable_params:,}")

param_bytes = total_params * 4  # 4 bytes per float32
param_mb = param_bytes / (1024 ** 2)
print(f"Parameter memory (fp32): ~{param_mb:.2f} MB")