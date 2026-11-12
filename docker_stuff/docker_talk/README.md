# R1 Talk Docker Environment

This Docker environment provides the full conversational and AI stack for the R1 Tour Guide Robot. It uses a compose-based workflow with two image flavours (**devel** and **stable**), GPU-aware settings, and dedicated services for image builds, CUDA bootstrapping, interactive sessions, and YARP runtime execution.

## What the image includes

The container image is built to support:

- Ubuntu 24.04 with development toolchains, CUDA toolkit and GPU-related packages
- YCM and YARP (branch selected by the image flavour, see [Devel vs stable](#devel-vs-stable))
- tour-guide-robot build and runtime setup
- `r1-object-retrieval` and `cer` applications (sparse checkout of their `app` folders only)
- Cloud speech and LLM integration: Azure/OpenAI (`yarp-devices-azure-openai`, `yarp-devices-llm`) and Google Cloud (`yarp-devices-google-cloud`)
- Offline speech and LLM integration: whisper.cpp, `yarp-device-speechTranscription-whisper`, `yarp-device-llama2`, `yarp-device-voicebox`, `yarp-device-kokoro`, voicebox, openWakeWord and Ollama
- Acoustic echo cancellation (`aec_baseline`) and Silero VAD
- A conda environment (`unified-voice-pipeline`) defined in [environment.yml](environment.yml)
- Persistent build caches for the CUDA-dependent components

## Prerequisites

- Docker Engine and Docker Compose v2 (or the legacy `docker-compose` binary)
- NVIDIA Container Toolkit for GPU access
- X11 forwarding capability on the host
- The following files and directories available on the host:
  - Google Cloud credentials: `~/.config/credentials/google-credentials/hsp_google.json`
  - Azure credentials: `~/.config/credentials/azure-credentials/config.env`
  - YARP configuration: `~/.config/yarp/`
  - Ollama models directory: `/usr/share/ollama/.ollama/models`
  - Optional Porcupine configuration directory

## Folder layout

```text
docker_talk/
├── Dockerfile
├── docker-compose.yml
├── environment.yml            # conda environment used inside the image
├── start_talk_devel.sh        # starts the devel flavour
├── start_talk_stable.sh       # starts the stable flavour
├── common/xorg.conf
└── env_config/                # (formerly "entrypoints/")
    ├── devel.env              # compose variables for the devel flavour
    ├── stable.env             # compose variables for the stable flavour
    ├── entrypoint_cuda.sh     # builds the CUDA-dependent repos, then runs the command
    ├── entrypoint_porcupine.sh# rebuilds tour-guide-robot with wake-word support
    └── env_porcupine.sh       # host-side variables to use the Porcupine entrypoint
```

## Devel vs stable

The image comes in two flavours, selected through the env files in [env_config/](env_config/):

| Variable | [devel.env](env_config/devel.env) | [stable.env](env_config/stable.env) |
|----------|-----------------------------------|-------------------------------------|
| `YARP_BRANCH` | `master` | `yarp-4.0` |
| `BUILD_TYPE` | `devel` | `stable` |
| `TOUR_BRANCH` | `jazzy` | `jazzy` |
| `R1_OBR_REMOTE` / `R1_OBR_BRANCH` | `origin` / `master` | `origin` / `master` |
| `TOUR_REMOTE` | `origin` | `origin` |
| `ENTRY_POINT` | `.entrypoint_cuda.sh` | `.entrypoint_cuda.sh` |

`YARP_BRANCH` is passed to the Dockerfile as `yarp_version`, and it is used not only for YARP itself but also as the branch for `cer`, `yarp-devices-azure-openai`, `yarp-devices-google-cloud`, `yarp-device-voicebox` and `yarp-device-kokoro`. Those repositories must therefore have a branch with the same name.

`BUILD_TYPE` is used both in the image tag and as a prefix for the named volumes, so the devel and stable flavours keep separate CUDA build caches.

## Build options

The compose file builds the image with the following defaults (used when no env file is passed):

- `UB_VERSION=ub24.04`
- `GCCPP_VERSION=v2.43`
- `CUDA_VERSION=13.0`
- `BUILD_TYPE=devel`
- `YARP_BRANCH=master`

The resulting image tag is:

```bash
ghcr.io/hsp-iit/r1_talk:${UB_VERSION:-ub24.04}_gccpp_${GCCPP_VERSION:-v2.43}_cuda_${CUDA_VERSION:-13.0}_${BUILD_TYPE:-devel}
```

i.e. `ghcr.io/hsp-iit/r1_talk:ub24.04_gccpp_v2.43_cuda_13.0_devel` or `ghcr.io/hsp-iit/r1_talk:ub24.04_gccpp_v2.43_cuda_13.0_stable`.

### Dockerfile build arguments

| Argument | Default | Compose variable | Description |
|----------|---------|------------------|-------------|
| `base_img` | `ubuntu:24.04` | `BASE_TALK_IMG` | Base image used for the container |
| `yarp_version` | `master` | `YARP_BRANCH` | YARP branch or tag (also used for the YARP-dependent repos listed above) |
| `tour_branch` | `jazzy` | `TOUR_BRANCH` | Branch of the tour-guide-robot repository to build |
| `robot_code` | `/usr/local/src/robot` | `ROBOT_CODE` | Installation root for the robot stack |
| `cuda_version` | `13.0` | `CUDA_VERSION` | CUDA toolkit version to install |
| `user_name` | `yarp-user` | - | User name inside the container |
| `uid` | `1000` | - | User ID inside the container |
| `gid` | `1000` | - | Group ID inside the container |
| `ai_model_path` | `/home/yarp-user/models` | - | Path used for mounted AI models |

### Runtime variables

Besides the ones set by the env files, the compose file reads these variables from the host shell:

| Variable | Default | Description |
|----------|---------|-------------|
| `DISPLAY` | - | X11 display |
| `AI_MODELS_PATH` | `$HOME` | Host directory mounted on `/home/yarp-user/models` |
| `PORCUPINE_CONFIG` | `/usr/local/src` | Host directory with the Porcupine configuration |
| `CUDA_REBUILD` | `0` | Set to `1` to force a rebuild of the CUDA-dependent repos |
| `CUDA_BUILD_JOBS` | `8` | Parallel jobs for the CUDA build |
| `CUDA_ARCHITECTURES` | `native` | Value passed to `CMAKE_CUDA_ARCHITECTURES` |

Variables exported in the shell take precedence over the values in the `--env-file`.

## Building the image

Pass the env file of the flavour you want to build:

```bash
# devel (YARP master)
docker compose --env-file env_config/devel.env --profile build build r1-talk-image

# stable (YARP yarp-4.0)
docker compose --env-file env_config/stable.env --profile build build r1-talk-image
```

You can also use the legacy `docker-compose` syntax with the same arguments.

## Services in the compose file

The compose file defines four services:

- `r1-talk-image`: build-only service, enabled through the `build` profile
- `r1-talk-cuda-build`: helper container that runs the CUDA build entrypoint and exits after preparing the CUDA-dependent artifacts; the other runtime services wait for it to complete successfully
- `r1-talk-container`: interactive container for development and testing (opens `terminator`)
- `r1-talk-yarprun`: container that configures YARP (`yarp conf 192.168.100.10 10000`) and starts `yarprun --server /console-llm`

### CUDA build helper service

The `r1-talk-cuda-build` service bootstraps the CUDA-dependent components once, so that subsequent containers can reuse the built artifacts. It uses [env_config/entrypoint_cuda.sh](env_config/entrypoint_cuda.sh) as its entrypoint and runs a one-shot build for:

- `whisper.cpp` (installed in `/home/yarp-user/userInstalled`)
- `yarp-device-speechTranscription-whisper`
- `yarp-device-llama2`

The entrypoint script configures the build environment, creates the required directories under the robot workspace, sets `CMAKE_PREFIX_PATH` to include YARP and YCM if present, builds the repositories with CUDA enabled, and writes a stamp file to avoid rebuilding unless `CUDA_REBUILD=1` is set.

## Launching the container

### Using the start scripts (recommended)

```bash
./start_talk_devel.sh    # devel flavour
./start_talk_stable.sh   # stable flavour
```

Each script enables X11 access (`xhost + local:docker`) and runs:

```bash
docker compose --env-file env_config/<flavour>.env up r1-talk-container --attach r1-talk-cuda-build
```

so the output of the CUDA build step is shown before the interactive container starts. If the image is not available locally, compose builds it first.

### Manually with Docker Compose

```bash
xhost + local:docker

# Interactive shell
docker compose --env-file env_config/devel.env up r1-talk-container

# YARP runtime container
docker compose --env-file env_config/devel.env up r1-talk-yarprun
```

Replace `devel.env` with `stable.env` to use the stable flavour.

### Using the Porcupine entrypoint

To rebuild tour-guide-robot with wake-word support at startup, source [env_config/env_porcupine.sh](env_config/env_porcupine.sh) before starting the container. It exports `ENTRY_POINT` pointing to [env_config/entrypoint_porcupine.sh](env_config/entrypoint_porcupine.sh) and `PORCUPINE_CONFIG=$HOME/.config/porcupine`, which override the env file values:

```bash
source env_config/env_porcupine.sh
./start_talk_devel.sh
```

### Manual run example

```bash
docker run -it --rm \
  --name r1-talk-custom \
  --gpus all \
  -e DISPLAY \
  -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v "${HOME}/.config/yarp:/home/yarp-user/.config/yarp:rw" \
  -v "${HOME}/.config/credentials/google-credentials/hsp_google.json:/home/yarp-user/.config/google-credential/hsp_google.json" \
  -v "${HOME}/.config/credentials/azure-credentials/config.env:/home/yarp-user/.env/config.env" \
  --network host \
  --privileged \
  ghcr.io/hsp-iit/r1_talk:ub24.04_gccpp_v2.43_cuda_13.0_devel \
  bash
```

Note that this bypasses the CUDA build entrypoint, so whisper.cpp, `yarp-device-speechTranscription-whisper` and `yarp-device-llama2` are not built.

## Volume mounts

The compose configuration mounts the following resources into the container:

- X11 socket: `/tmp/.X11-unix:/tmp/.X11-unix:rw`
- Host `/etc/hosts`
- YARP config: `~/.config/yarp:/home/yarp-user/.config/yarp:rw`
- Google credentials: `~/.config/credentials/google-credentials/hsp_google.json`
- Azure credentials: `~/.config/credentials/azure-credentials/config.env`
- Porcupine config: `${PORCUPINE_CONFIG:-/usr/local/src}/:/usr/local/src/robot/.config/porcupine`
- AI models directory: `${AI_MODELS_PATH:-$HOME}:/home/yarp-user/models:rw`
- Ollama models: `/usr/share/ollama/.ollama/models`
- Entrypoint scripts from [env_config/](env_config/), mounted in `/home/yarp-user/config/`

The compose file also creates named volumes, prefixed with `BUILD_TYPE` so that each flavour has its own:

- `<BUILD_TYPE>-cuda-user-installed`: persists `/home/yarp-user/userInstalled`
- `<BUILD_TYPE>-cuda-whisper-build`: persists the build directory of `whisper.cpp`
- `<BUILD_TYPE>-cuda-speech-transcription-build`: persists the build directory of `yarp-device-speechTranscription-whisper`
- `<BUILD_TYPE>-cuda-llama2-build`: persists the build directory of `yarp-device-llama2`

These volumes survive `docker compose down --remove-orphans`. Remove them with `docker volume rm` to start from a clean CUDA build.

## Usage notes

- Use `r1-talk-container` for interactive development and debugging.
- Use `r1-talk-yarprun` when you want the container to start the YARP runtime directly.
- If you want to rebuild CUDA components, set `CUDA_REBUILD=1` before starting the container.
- For wake-word support, provide a valid `PORCUPINE_CONFIG` directory and ensure the required model files are available.

## Troubleshooting

### Display issues
- Ensure `xhost + local:docker` is enabled on the host (the start scripts do it for you).
- Check X11 forwarding with `echo $DISPLAY`.

### Credential issues
- Verify that the Google and Azure credential files exist and are readable.
- Confirm that the paths in the compose file match the files on your host.

### GPU issues
- Ensure the NVIDIA Container Toolkit is installed and configured.
- Check GPU visibility from the host with `nvidia-smi`.

### Build issues
- If a `git clone -b <branch>` step fails, check that all the YARP-dependent repositories have a branch matching `YARP_BRANCH`.
- Increase `CUDA_BUILD_JOBS` if you want faster rebuilds on a powerful machine.
- Use a larger disk allocation if the CUDA and model-dependent build steps are failing due to space constraints.

## Ports and networking

- YARP traffic is exposed through `host` networking mode.
- The container uses port `10000` for YARP communication (TCP/UDP).
