# R1 Talk Docker Environment

This Docker environment provides a complete setup for the R1 Tour Guide Robot's conversational and AI capabilities. The image includes all necessary components for natural language processing, speech synthesis, dialogue management, and integration with Google Cloud services and Large Language Models (LLMs).

## Docker Image Contents

The Docker image (`elandini84/r1_talk:ub24.04_vcpkg_gccpp_v2.33`) includes:

- **Base System**: Ubuntu 24.04 with essential development tools
- **YARP Framework**: Complete YARP middleware installation with Python bindings
- **Robotology Stack**: YCM, iCub-main, CER, and Navigation components
- **Google Cloud Integration**: Speech-to-Text, Text-to-Speech, and Dialogflow CX services via vcpkg
- **Azure OpenAI Integration**: LLM capabilities through Azure OpenAI services
- **Tour Guide Application**: Complete tour-guide-robot application
- **R1 Object Retrieval**: Object detection and retrieval capabilities
- **Development Tools**: CMake, Git, Terminator, debugging tools, and more

## Prerequisites

- Docker and Docker Compose installed
- NVIDIA Docker runtime (for GPU support)
- X11 forwarding capability
- Required credential files:
  - Google Cloud credentials: `~/.config/credentials/google-credentials/hsp_google.json`
  - Azure credentials: `~/.config/credentials/azure-credentials/config.env`
  - YARP configuration: `~/.config/yarp/`
  - (Optional) Porcupine wake word configuration

## Building the Docker Image

### Using Docker Compose (Recommended)

```bash
# Build the image
docker-compose build r1-talk-ub24-v2.33-master
```

### Using Docker Build Directly

```bash
# Build with default arguments
docker build -t r1-talk:custom .

# Build with custom arguments
docker build \
  --build-arg base_img="ubuntu:24.04" \
  --build-arg yarp_version="yarp-3.12" \
  --build-arg tour_branch="jazzy" \
  --build-arg llm_branch="master" \
  --build-arg google_cloud_version="tags/2025.01.13" \
  --build-arg user_name="yarp-user" \
  --build-arg uid="1000" \
  --build-arg gid="1000" \
  -t r1-talk:custom .
```

### Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `base_img` | `ubuntu:24.04` | Base Docker image |
| `yarp_version` | `master` | YARP version/branch to build |
| `user_name` | `yarp-user` | User name inside container |
| `robot_code` | `/usr/local/src/robot` | Robot code installation path |
| `llm_branch` | `master` | LLM devices branch |
| `uid` | `1000` | User ID |
| `gid` | `1000` | Group ID |
| `google_cloud_version` | `tags/2025.01.13` | Google Cloud C++ library version |
| `tour_branch` | `jazzy` | Tour guide robot branch |

## Launching the Container

### Environment Setup

Choose one of the available environment configurations before launching:

#### Standard Environment (Light Build)
```bash
source entrypoints/env.sh
```

#### Porcupine Wake Word Environment
```bash
source entrypoints/env_porcupine.sh
```

#### Empty Environment (No Auto-Build)
```bash
source entrypoints/env_e.sh
```

### Launch Options

#### 1. Interactive Terminal Container
```bash
# Source environment first
source entrypoints/env.sh

# Launch with terminator
docker-compose up r1-talk-ub24-v2.33-master-container
```

#### 2. YARP Server Container
```bash
# Source environment first
source entrypoints/env.sh

# Launch YARP server
docker-compose up r1-talk-ub24-v2.33-master-yarprun
```

#### 3. Custom Container Launch
```bash
# Run with custom command
docker run -it --rm \
  --name r1-talk-custom \
  --env-file entrypoints/env.sh \
  -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v "${HOME}/.config/yarp:/home/yarp-user/.config/yarp:rw" \
  -v "${HOME}/.config/credentials/google-credentials/hsp_google.json:/home/yarp-user/.config/google-credential/hsp_google.json" \
  -v "${HOME}/.config/credentials/azure-credentials/config.env:/home/yarp-user/.env/config.env" \
  --network host \
  --privileged \
  elandini84/r1_talk:ub24.04_vcpkg_gccpp_v2.33 \
  bash
```

## Entrypoint Options

The container supports different entrypoint modes:

### 1. Light Entrypoint (`entrypoint_light.sh`)
- Updates tour-guide-robot and r1-object-retrieval repositories
- Builds with standard options
- **Default for most use cases**

### 2. Heavy Entrypoint (`entrypoint_heavy.sh`)
- Full rebuild of all components (YCM, YARP, iCub-main, Navigation, CER)
- Complete system update
- **Use for major updates or clean builds**

### 3. Porcupine Entrypoint (`entrypoint_porcupine.sh`)
- Enables wake word detection capabilities
- Builds with `WAKE_WORD=ON` flag
- **Use for voice activation features**

### 4. Empty Entrypoint (`entrypoint_empty.sh`)
- No automatic building or updates
- **Use for development or custom setups**

## Environment Variables

Key environment variables that can be customized:

```bash
# Repository Configuration
export R1_OBR_REMOTE=elandini84          # R1 object retrieval remote
export R1_OBR_BRANCH=test/sim_lobby      # R1 object retrieval branch
export TOUR_REMOTE=origin                # Tour guide remote
export TOUR_BRANCH=jazzy                 # Tour guide branch

# Container Configuration
export ENTRY_POINT=/home/yarp-user/config/.entrypoint_light.sh
export PORCUPINE_CONFIG=${HOME}/.config/porcupine  # For wake word
```

## Volume Mounts

The container requires several volume mounts for proper operation:

- **X11 Display**: `/tmp/.X11-unix:/tmp/.X11-unix:rw`
- **YARP Config**: `~/.config/yarp:/home/yarp-user/.config/yarp:rw`
- **Google Credentials**: `~/.config/credentials/google-credentials/hsp_google.json`
- **Azure Credentials**: `~/.config/credentials/azure-credentials/config.env`
- **Porcupine Config**: `${PORCUPINE_CONFIG}:/usr/local/src/robot/.config/porcupine` (optional)

## Usage Examples

### Basic Development Session
```bash
# Setup environment
source entrypoints/env.sh

# Launch interactive container
docker-compose up r1-talk-ub24-v2.33-master-container

# Inside container, you can now:
# - Build tour-guide-robot applications
# - Run YARP applications
# - Test speech and dialogue systems
```

### YARP Server Setup
```bash
# Launch YARP server
docker-compose up r1-talk-ub24-v2.33-master-yarprun

# In another terminal/container, connect applications
```

### Wake Word Detection
```bash
# Setup Porcupine environment
source entrypoints/env_porcupine.sh

# Launch container with wake word support
docker-compose up r1-talk-ub24-v2.33-master-container
```

## Troubleshooting

### Display Issues
- Ensure `xhost + local:docker` is run on the host
- Check X11 forwarding: `echo $DISPLAY`

### Credential Issues
- Verify Google Cloud credentials file exists and has correct permissions
- Check Azure configuration file format

### Build Issues
- Use heavy entrypoint for complete rebuild
- Check available disk space
- Verify network connectivity for repository updates

### GPU Issues
- Ensure NVIDIA Docker runtime is installed
- Check GPU accessibility: `nvidia-smi` on host

## Ports

- **10000**: YARP port (TCP/UDP)

## Network

The container uses `host` networking mode for seamless integration with YARP and ROS2 systems.