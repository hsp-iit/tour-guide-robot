# R1 Talk Docker Environment

This Docker environment provides the full conversational and AI stack for the R1 Tour Guide Robot. The current setup uses a compose-based workflow with configurable build arguments, GPU-aware settings, and dedicated services for image builds, interactive sessions, and YARP runtime execution.

## What the image includes

The container image is built to support:

- Ubuntu 24.04 with development toolchains and GPU-related packages
- YARP with CUDA-aware build support
- tour-guide-robot build and runtime setup
- Offline speech and LLM integration, including whisper.cpp, speech transcription, llama2, voicebox, openWakeWord, Azure/OpenAI, and Google Cloud support
- Optional model mounting and persistent build caches for CUDA-related components

## Prerequisites

- Docker Engine and Docker Compose v2 (or the legacy `docker-compose` binary)
- NVIDIA Container Toolkit for GPU access
- X11 forwarding capability on the host
- The following files and directories available on the host:
  - Google Cloud credentials: `~/.config/credentials/google-credentials/hsp_google.json`
  - Azure credentials: `~/.config/credentials/azure-credentials/config.env`
  - YARP configuration: `~/.config/yarp/`
  - Optional Porcupine configuration directory

## Build options

The compose file builds the image with the following defaults:

- `UB_VERSION=ub24.04`
- `GCCPP_VERSION=v2.43`
- `CUDA_VERSION=13.0`
- `BUILD_TYPE=devel`

The resulting image tag is:

```bash
ghcr.io/hsp-iit/r1_talk:${UB_VERSION:-ub24.04}_gccpp_${GCCPP_VERSION:-v2.43}_cuda_${CUDA_VERSION:-13.0}_${BUILD_TYPE:-devel}
```

### Dockerfile build arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `base_img` | `ubuntu:24.04` | Base image used for the container |
| `tour_branch` | `jazzy` | Branch of the tour-guide-robot repository to build |
| `llm_branch` | `master` | Branch used for LLM-related repositories |
| `robot_code` | `/usr/local/src/robot` | Installation root for the robot stack |
| `user_name` | `yarp-user` | User name inside the container |
| `yarp_version` | `master` | YARP branch or tag to build |
| `uid` | `1000` | User ID inside the container |
| `gid` | `1000` | Group ID inside the container |
| `cuda_version` | `13.0` | CUDA toolkit version to install |
| `ai_model_path` | `/home/yarp-user/models` | Path used for mounted AI models |

### Compose environment variables

The compose file also exposes these runtime variables:

```bash
export DISPLAY=$DISPLAY
export R1_OBR_REMOTE=origin
export R1_OBR_BRANCH=master
export TOUR_REMOTE=origin
export TOUR_BRANCH=jazzy
export ROBOT_CODE=/usr/local/src/robot
export AI_MODELS_PATH=$HOME
export CUDA_REBUILD=0
export CUDA_BUILD_JOBS=8
export CUDA_ARCHITECTURES=native
export PORCUPINE_CONFIG=$HOME/.config/porcupine
```

## Building the image

### Using Docker Compose

```bash
docker compose --profile build build r1-talk-image
```

You can also use the legacy syntax:

```bash
docker-compose --profile build build r1-talk-image
```

## Services in the compose file

The compose file defines four services:

- `r1-talk-image`: build-only service, enabled through the `build` profile
- `r1-talk-cuda-build`: helper container that runs the CUDA build entrypoint and exits after preparing the CUDA-dependent artifacts; it is used as a dependency for the other runtime services
- `r1-talk-container`: interactive container for development and testing
- `r1-talk-yarprun`: container that starts the YARP runtime command

### CUDA build helper service

The `r1-talk-cuda-build` service is meant to bootstrap the CUDA-dependent components once, so that subsequent containers can reuse the built artifacts. It uses [entrypoints/entrypoint_cuda.sh](entrypoints/entrypoint_cuda.sh) as its entrypoint and runs a one-shot build for:

- `whisper.cpp`
- `yarp-device-speechTranscription-whisper`
- `yarp-device-llama2`

The entrypoint script configures the build environment, creates the required directories under the robot workspace, sets `CMAKE_PREFIX_PATH` to include YARP and YCM if present, builds the repositories with CUDA enabled, and writes a stamp file to avoid rebuilding unless `CUDA_REBUILD=1` is set.

## Launching the container

Before running either the interactive container or the YARP runtime service, source the environment file:

```bash
source entrypoints/env_cuda.sh
```

### Interactive shell

```bash
docker compose up r1-talk-container
```

### YARP runtime container

```bash
docker compose up r1-talk-yarprun
```

### Manual run example

```bash
docker run -it --rm \
  --name r1-talk-custom \
  --env-file <(env | grep -E '^(DISPLAY|XAUTHORITY|R1_OBR_REMOTE|R1_OBR_BRANCH|TOUR_REMOTE|TOUR_BRANCH|ROBOT_CODE|AI_MODELS_PATH|CUDA_REBUILD|CUDA_BUILD_JOBS|CUDA_ARCHITECTURES)') \
  -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v "${HOME}/.config/yarp:/home/yarp-user/.config/yarp:rw" \
  -v "${HOME}/.config/credentials/google-credentials/hsp_google.json:/home/yarp-user/.config/google-credential/hsp_google.json" \
  -v "${HOME}/.config/credentials/azure-credentials/config.env:/home/yarp-user/.env/config.env" \
  --network host \
  --privileged \
  ghcr.io/hsp-iit/r1_talk:ub24.04_gccpp_v2.43_cuda_13.0_devel \
  bash
```

## Volume mounts

The compose configuration mounts the following resources into the container:

- X11 socket: `/tmp/.X11-unix:/tmp/.X11-unix:rw`
- Host `/etc/hosts`
- YARP config: `~/.config/yarp:/home/yarp-user/.config/yarp:rw`
- Google credentials: `~/.config/credentials/google-credentials/hsp_google.json`
- Azure credentials: `~/.config/credentials/azure-credentials/config.env`
- Porcupine config: `${PORCUPINE_CONFIG:-/usr/local/src}/:/usr/local/src/robot/.config/porcupine`
- AI models directory: `${AI_MODELS_PATH:-$HOME}:/home/yarp-user/models:rw`
- Entrypoint scripts and CUDA-related persistent volumes

The compose file also creates named volumes for reuse across runs:

- `cuda-user-installed`: persists the contents of `/home/yarp-user/userInstalled`
- `cuda-whisper-build`: persists the build directory for `whisper.cpp`
- `cuda-speech-transcription-build`: persists the build directory for `yarp-device-speechTranscription-whisper`
- `cuda-llama2-build`: persists the build directory for `yarp-device-llama2`

## Usage notes

- Use `r1-talk-container` for interactive development and debugging.
- Use `r1-talk-yarprun` when you want the container to start the YARP runtime directly.
- If you want to rebuild CUDA components, set `CUDA_REBUILD=1` before starting the container.
- For wake-word support, provide a valid `PORCUPINE_CONFIG` directory and ensure the required model files are available.

## Troubleshooting

### Display issues
- Ensure `xhost + local:docker` is enabled on the host.
- Check X11 forwarding with `echo $DISPLAY`.

### Credential issues
- Verify that the Google and Azure credential files exist and are readable.
- Confirm that the paths in the compose file match the files on your host.

### GPU issues
- Ensure the NVIDIA Container Toolkit is installed and configured.
- Check GPU visibility from the host with `nvidia-smi`.

### Build issues
- Increase `CUDA_BUILD_JOBS` if you want faster rebuilds on a powerful machine.
- Use a larger disk allocation if the CUDA and model-dependent build steps are failing due to space constraints.

## Ports and networking

- YARP traffic is exposed through `host` networking mode.
- The container uses port `10000` for YARP communication (TCP/UDP).
