# MasterThesis
All code used to support the Masterthesis

Ultralytics Installation on JetPack 6.1 (Jetson Orin Nano)
=========================================================

This guide will walk you through installing the **Ultralytics** package with export functionality on a Jetson device running **JetPack 6.1**. Our primary focus is enabling **TensorRT exports** to maximize performance on Jetson hardware.

---

1. Update Package List and Install pip
--------------------------------------

Update the system and install pip:

    sudo apt update
    sudo apt install python3-pip -y
    pip install -U pip

---

2. Install Ultralytics with Export Dependencies
-----------------------------------------------

Install the Ultralytics pip package with optional dependencies:

    pip install ultralytics[export]

---

3. Reboot the Device
--------------------

Reboot to apply any environment or system-level changes:

    sudo reboot

---

4. Install Compatible PyTorch and Torchvision
---------------------------------------------

The versions of PyTorch and Torchvision installed via `pip install ultralytics[export]` are not compatible with Jetson (ARM64). We need to manually install Jetson-compatible versions.

Install PyTorch 2.5.0 and Torchvision 0.20.0 (for JetPack 6.1):

    pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl
    pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl

> **Note**  
> Visit the [PyTorch for Jetson page](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048) to find compatible versions of PyTorch for various JetPack versions.  
> For a detailed compatibility chart, check the official [PyTorch & Torchvision compatibility matrix](https://pytorch.org/get-started/previous-versions/).

---

5. Install cuSPARSELt (Fixes Dependency Issue with Torch 2.5.0)
---------------------------------------------------------------

To resolve dependency issues introduced by torch 2.5.0, install cuSPARSELt:

    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
    sudo dpkg -i cuda-keyring_1.1-1_all.deb
    sudo apt-get update
    sudo apt-get -y install libcusparselt0 libcusparselt-dev

---

6. Install ONNX Runtime GPU (onnxruntime-gpu)
---------------------------------------------

The `onnxruntime-gpu` package on PyPI does not provide ARM64 builds compatible with Jetson. We'll manually install version **1.20.0** with Python 3.10 support.

    pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl

> **Note**  
> You can find the full list of ONNX Runtime GPU builds organized by JetPack and Python versions in the [Jetson Zoo ONNX Runtime compatibility matrix](https://elinux.org/Jetson_Zoo#ONNX_Runtime).

---

7. Fix Numpy Version
---------------------

Installing `onnxruntime-gpu` may upgrade `numpy` to an incompatible version. Downgrade it back to 1.23.5:

    pip install numpy==1.23.5

---

✅ **Installation Complete**

You can now use Ultralytics on JetPack 6.1 with optimized export capabilities including **TensorRT**, **ONNX**, and more.
You're now ready to use Ultralytics on Jetson with optimized export capabilities (including TensorRT)!

After this you can run: jetson@jetson-orin-nano1:~/Documents/MasterThesis$ python unrealsim/senderLocalInference.py 

<img width="643" height="576" alt="image" src="https://github.com/user-attachments/assets/846a4457-bd6e-4f23-86ec-4886d6b3fb45" />



