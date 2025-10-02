# MasterThesis
All code used to support the Masterthesis

Ultralytics Installation on JetPack 6.1 (Jetson Platform)
=========================================================

This guide will walk you through installing the **Ultralytics** package with export functionality on a Jetson device running **JetPack 6.1**. Our primary focus is enabling **TensorRT exports** to optimize performance on the Jetson platform.

Steps:

1. Update Package List and Install pip
---------------------------------------

Update the system and install Python's pip package manager:

    sudo apt update
    sudo apt install python3-pip -y
    pip install -U pip


2. Install Ultralytics with Export Dependencies
-----------------------------------------------

Install the Ultralytics package with optional export dependencies:

    pip install ultralytics[export]


3. Reboot the Device
--------------------

After installing the package, reboot the Jetson device to ensure all changes take effect:

    sudo reboot


4. Install Compatible PyTorch and Torchvision
---------------------------------------------

The versions of PyTorch and Torchvision installed via `pip install ultralytics[export]` are not compatible with Jetson's ARM64 architecture. Therefore, we must:

- Install a compatible pre-built PyTorch wheel.
- Install the corresponding Torchvision version.

Use the following commands:

    pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl
    pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl


You're now ready to use Ultralytics on Jetson with optimized export capabilities (including TensorRT)!


