# Jetson Thor Python Environment

Use these files for the Jetson Thor Python/voice stack:

- `environment.jetson-thor.yml`: conda base environment for Ubuntu 24.04 / Python 3.12 / linux-aarch64.
- `requirements.jetson-thor.txt`: pip packages from the original voice environment, excluding PyTorch and torchaudio.
- `install_jetson_thor_env.sh`: helper that creates/updates the env, verifies NVIDIA PyTorch/torchaudio, then installs the remaining packages without replacing the Jetson GPU stack.

## Expected NVIDIA Base

Jetson Thor is a JetPack 7.x target. Use a JetPack/NGC-matched PyTorch build for `arm64`; do not use the generic `https://download.pytorch.org/whl/cu128` index from `environment.yml`.

The recommended path is to start from an NVIDIA PyTorch/NGC image that matches the JetPack installed on the Thor, or install NVIDIA's matching Jetson PyTorch wheel into the conda env before installing the rest of the requirements.

## Native Conda Flow

```bash
cd docker_stuff/docker_talk

# Create/update the base conda env.
conda env create -f environment.jetson-thor.yml
conda activate unified-voice-pipeline-thor

# Install the NVIDIA Jetson Thor PyTorch and torchaudio builds here.
# Example shape only; use the wheel/container matching the installed JetPack:
# python -m pip install --no-cache-dir /path/to/torch-...-linux_aarch64.whl
# python -m pip install --no-cache-dir /path/to/torchaudio-...-linux_aarch64.whl

./install_jetson_thor_env.sh
```

You can also pass wheel paths directly:

```bash
TORCH_INSTALL=/path/to/torch-...-linux_aarch64.whl \
TORCHAUDIO_INSTALL=/path/to/torchaudio-...-linux_aarch64.whl \
./install_jetson_thor_env.sh
```

## Notes

- `onnxruntime` is left as CPU ONNX Runtime for openwakeword compatibility. If you need GPU/TensorRT ONNX Runtime on Thor, install a Thor-compatible `onnxruntime-gpu` build separately and remove the `onnxruntime` line from `requirements.jetson-thor.txt`.
- `numpy` is pinned to `<2` in the conda env to stay compatible with NVIDIA PyTorch wheels and common audio packages.
- The existing `Dockerfile` still contains x86-specific CUDA and ONNX Runtime downloads; those need separate Dockerfile changes before this whole Docker image can build natively on Thor.

