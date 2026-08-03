#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_NAME="${JETSON_THOR_CONDA_ENV:-unified-voice-pipeline-thor}"
CONDA_EXE="${CONDA_EXE:-conda}"

if ! command -v "${CONDA_EXE}" >/dev/null 2>&1; then
    echo "Could not find conda. Install Miniforge for linux-aarch64 first or set CONDA_EXE."
    exit 1
fi

eval "$("${CONDA_EXE}" shell.bash hook)"

if conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
    conda env update -n "${ENV_NAME}" -f "${SCRIPT_DIR}/environment.jetson-thor.yml" --prune
else
    conda env create -n "${ENV_NAME}" -f "${SCRIPT_DIR}/environment.jetson-thor.yml"
fi

conda activate "${ENV_NAME}"
python -m pip install --upgrade pip setuptools wheel

if [[ -n "${TORCH_INSTALL:-}" ]]; then
    python -m pip install --no-cache-dir "${TORCH_INSTALL}"
fi

if [[ -n "${TORCHAUDIO_INSTALL:-}" ]]; then
    python -m pip install --no-cache-dir "${TORCHAUDIO_INSTALL}"
fi

if ! python -c "import torch" >/dev/null 2>&1; then
    cat <<'EOF'
PyTorch is not installed in this conda environment.

Install the NVIDIA JetPack/NGC-matched Jetson Thor PyTorch build first, then
rerun this script. You can also set TORCH_INSTALL to a local or remote wheel:

  TORCH_INSTALL=/path/to/torch-...-linux_aarch64.whl ./install_jetson_thor_env.sh

Avoid the generic download.pytorch.org CUDA wheel indexes on Jetson Thor.
EOF
    exit 1
fi

if ! python -c "import torchaudio" >/dev/null 2>&1; then
    cat <<'EOF'
torchaudio is not installed in this conda environment.

Install a torchaudio build matched to the NVIDIA Jetson Thor PyTorch build,
then rerun this script. You can set TORCHAUDIO_INSTALL to a local or remote
wheel:

  TORCHAUDIO_INSTALL=/path/to/torchaudio-...-linux_aarch64.whl ./install_jetson_thor_env.sh

This script stops here to avoid pip installing an incompatible generic
torchaudio wheel over the Jetson PyTorch stack.
EOF
    exit 1
fi

CONSTRAINTS_FILE="$(mktemp)"
trap 'rm -f "${CONSTRAINTS_FILE}"' EXIT

python - <<'PY' > "${CONSTRAINTS_FILE}"
from importlib import metadata

for package in ("torch", "torchaudio", "torchvision"):
    try:
        print(f"{package}==={metadata.version(package)}")
    except metadata.PackageNotFoundError:
        pass
PY

python -m pip install \
    --constraint "${CONSTRAINTS_FILE}" \
    --requirement "${SCRIPT_DIR}/requirements.jetson-thor.txt"

python - <<'PY'
import torch
import torchaudio

print("Jetson Thor Python environment ready.")
print(f"torch={torch.__version__}")
print(f"torchaudio={torchaudio.__version__}")
print(f"torch.cuda.is_available={torch.cuda.is_available()}")
PY

