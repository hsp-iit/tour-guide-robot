# R1 Tour Guide Robot Simulation Docker Environment

This Docker environment provides the complete simulation stack for the R1 Tour Guide Robot. The image includes all components necessary for robot simulation, including Gazebo Harmonic, YARP-Gazebo integration, robot models, and simulation-specific plugins for testing and development in a virtual environment.

## Docker Image Contents

The Docker image (`ghcr.io/hsp-iit/r1images:tourSim2_*`) includes:

- **Base System**: Ubuntu 24.04 with development tools and X11 support
- **ROS2 Framework**: Complete ROS2 installation (Jazzy by default) with Navigation2, SLAM, and testing tools
- **YARP Middleware**: Full YARP installation with Python bindings and simulation-specific device plugins
- **Gazebo Simulation**:
  - Gazebo Harmonic, installed through the `ros-${ros_distro}-ros-gz` package
  - gz-sim-yarp-plugins for YARP-Gazebo integration (from the `elandini84/gz-sim-yarp-plugins` fork, `fix/yarp4` branch)
  - CLI11 dependency for Gazebo plugins
- **Robotology Stack**:
  - YCM (YARP CMake Modules)
  - iCub-main with cartesian controllers (simulation-focused)
  - CER (Collaborative Effort Robot) components (from the `elandini84/cer` fork)
  - Ergocub software and R1 robot models
- **Simulation Components**:
  - CER-SIM with ROS2 support
  - R1 robot models and the LIRS-HMLG environments
  - Fake device plugins for simulation (fakeBattery, fakeIMU, fakeLocalizer, etc.)
- **Navigation Stack**: Advanced navigation with ROS2 integration
- **Development Tools**: web teleop sources (configured, not built), CMake, Git, and debugging tools
- **Network Middleware**: CycloneDDS for improved ROS2 communication

## Prerequisites

- Docker Engine and Docker Compose v2 (`--env-file` must be repeatable, i.e. Compose >= 2.17, to combine [cuda.env](env_config/cuda.env) with a flavour env file)
- NVIDIA Container Toolkit (for the default GPU-enabled container and for CUDA-based images)
- X11 forwarding capability
- CycloneDDS configuration file (optional, the one in the repository is used as fallback)

## Folder layout

```text
docker_sim/
├── Dockerfile
├── docker-compose.yml
├── start_tourSim_devel.sh       # starts the devel flavour
├── start_tourSim_stable.sh      # starts the stable flavour
├── common/xorg.conf
└── env_config/
    ├── devel.env                # compose variables for the devel flavour
    ├── stable.env               # compose variables for the stable flavour
    ├── cuda.env                 # overrides the base image with the nvidia/cuda one
    └── select_dds_conf.sh       # picks the CycloneDDS configuration file to mount
```

## Devel vs stable

The image comes in two flavours, selected through the env files in [env_config/](env_config/):

- **devel**: built from the development branches (`master` for YARP and the YARP-dependent repositories)
- **stable**: built from a stable YARP release branch. The YARP branch is part of the image tag, so it is always clear which stable YARP version an image contains

| Variable | [devel.env](env_config/devel.env) | [stable.env](env_config/stable.env) |
|----------|-----------------------------------|-------------------------------------|
| `BASE_IMG` / `BASE_IMG_LABEL` | `ubuntu:24.04` / `ubuntu24.04` | `ubuntu:24.04` / `ubuntu24.04` |
| `ROS_DISTRO` | `jazzy` | `jazzy` |
| `YARP_BRANCH` | `master` | `yarp-4.0` |
| `MESH_BRANCH` | `devel` | `devel` |
| `BATTERY` | `OFF` | `OFF` |
| `BUILD_TYPE` | `devel` | `${YARP_BRANCH}` (i.e. `yarp-4.0`) |

`BATTERY` is not used by this image: the env files are kept identical to the [docker_tourCore](../docker_tourCore/) ones, so both images are always built from the same branches.

The resulting image tag is:

```bash
${IMAGE_REPO:-ghcr.io/hsp-iit/r1images}:tourSim2_${BASE_IMG_LABEL:-ubuntu24.04}_${ROS_DISTRO:-jazzy}_${BUILD_TYPE:-devel}
```

i.e. `ghcr.io/hsp-iit/r1images:tourSim2_ubuntu24.04_jazzy_devel` or `ghcr.io/hsp-iit/r1images:tourSim2_ubuntu24.04_jazzy_yarp-4.0`.

To move the stable flavour to a newer YARP release, change `YARP_BRANCH` in [stable.env](env_config/stable.env): the tag follows automatically.

### Dockerfile build arguments

| Argument | Default | Compose variable | Description |
|----------|---------|------------------|-------------|
| `base_img` | - (compose: `ubuntu:24.04`) | `BASE_IMG` | Base Docker image (ubuntu:24.04 or nvidia/cuda:*) |
| `ros_distro` | - (compose: `jazzy`) | `ROS_DISTRO` | ROS2 distribution name |
| `yarp_branch` | `master` | `YARP_BRANCH` | Branch of YARP, `navigation`, `cer` and `yarp-devices-ros2` (all of them must have a branch with this name) |
| `mesh_branch` | `master` | `MESH_BRANCH` | Branch of `icub-main` |
| `gazebo_pkg` | `ros-${ros_distro}-ros-gz` | - | ROS2-Gazebo integration package |
| `gazebo_version` | `harmonic` | - | Currently unused (only referenced by the commented-out standalone Gazebo installation) |

### Runtime variables

Besides the ones set by the env files, the compose file reads these variables from the host shell:

| Variable | Default | Description |
|----------|---------|-------------|
| `DISPLAY` | - | X11 display |
| `ROS_DOMAIN_ID` | - | Passed to the container only if exported on the host |
| `CYCLONE_DDS_CONF` | `~/.config/cyclone_dds_settings.xml` | Host CycloneDDS file mounted on `/home/user1/.config/cyclone_dds_settings.xml` |
| `IMAGE_REPO` | `ghcr.io/hsp-iit/r1images` | Repository part of the image name |
| `BASE_IMG_LABEL` | `ubuntu24.04` | Label of the base image used in the tag |

Variables exported in the shell take precedence over the values in the `--env-file`.

## Building the image

Pass the env file of the flavour you want to build:

```bash
# devel
docker compose --env-file env_config/devel.env --profile build build r1-toursim-image

# stable
docker compose --env-file env_config/stable.env --profile build build r1-toursim-image

# CUDA-based image (nvidia/cuda base): add cuda.env after the flavour env file
docker compose --env-file env_config/devel.env --env-file env_config/cuda.env --profile build build r1-toursim-image

# Custom options: export the variables before running compose
YARP_BRANCH=<branch> MESH_BRANCH=<branch> docker compose --env-file env_config/devel.env --profile build build r1-toursim-image
```

With `devel.env` the YARP branch is not part of the image tag: if you build devel images with a different YARP branch, also change `BUILD_TYPE` (or `IMAGE_REPO`) to keep them apart. With `stable.env` the tag follows `YARP_BRANCH` automatically.

## Services in the compose file

- `r1-toursim-image`: build-only service, enabled through the `build` profile
- `r1-toursim-container`: GPU-enabled container (opens `terminator`)
- `r1-toursim-container-nogpu`: same container without GPU reservation, enabled through the `nogpu` profile (only for Ubuntu-based images; CUDA-based ones require a GPU)

## Launching the container

### Using the start scripts (recommended)

```bash
./start_tourSim_devel.sh    # devel flavour
./start_tourSim_stable.sh   # stable flavour
```

Each script selects the CycloneDDS configuration file (see [DDS configuration](#dds-configuration)), enables X11 access (`xhost + local:docker`) and runs:

```bash
docker compose --env-file env_config/<flavour>.env up r1-toursim-container
```

If the image is not available locally, compose pulls it (or builds it if pulling fails).

### Manually with Docker Compose

```bash
xhost + local:docker

# GPU-enabled container
docker compose --env-file env_config/devel.env up r1-toursim-container

# Without GPU support (Ubuntu images only)
docker compose --env-file env_config/devel.env --profile nogpu up r1-toursim-container-nogpu

# CUDA-based image
docker compose --env-file env_config/devel.env --env-file env_config/cuda.env up r1-toursim-container

# Interactive bash shell in the terminal instead of terminator
docker compose --env-file env_config/devel.env run --rm r1-toursim-container bash

# Open another shell in a running container
docker exec -it r1-toursim-container bash
```

Replace `devel.env` with `stable.env` to use the stable flavour. Stop and remove the containers with `docker compose down --remove-orphans`.

### DDS configuration

The CycloneDDS file mounted in the container is chosen through `CYCLONE_DDS_CONF`. The start scripts source [env_config/select_dds_conf.sh](env_config/select_dds_conf.sh), which uses, in order:

1. the file pointed by `CYCLONE_DDS_CONF`, if exported and existing
2. `~/.config/cyclone_dds_settings.xml`, if existing
3. the fallback in the repository: `app/navigation2/conf/cyclone_dds_settings.xml`

```bash
# Use the configuration in the repository
CYCLONE_DDS_CONF=../../app/navigation2/conf/cyclone_dds_settings.xml ./start_tourSim_devel.sh

# Use a custom configuration
CYCLONE_DDS_CONF=/path/to/cyclone_dds_settings.xml ./start_tourSim_devel.sh
```

When running compose directly, make sure `CYCLONE_DDS_CONF` (or `~/.config/cyclone_dds_settings.xml`) points to an existing file: otherwise Docker creates an empty directory in its place.

### Manual Docker Run

```bash
# With GPU support
docker run --rm -it --privileged --network host --pid host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -e ROS_DOMAIN_ID \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/.config/cyclone_dds_settings.xml:/home/user1/.config/cyclone_dds_settings.xml \
  -e QT_X11_NO_MITSHM=1 --gpus all \
  ghcr.io/hsp-iit/r1images:tourSim2_ubuntu24.04_jazzy_devel
```

## Usage Examples

### Gazebo Simulation Environment

```bash
# Inside the container, start YARP server
yarpserver --write

# In another terminal inside container, launch Gazebo with one of the tour guide worlds
gz sim /usr/local/src/robot/tour-guide-robot/app/maps/SIM_GAM/GAM.world
```

### R1 Robot Simulation with ROS2

The [r1Mk3_gz_sim.launch.py](../../app/navigation2/launch/r1Mk3_gz_sim.launch.py) launch file starts Gazebo (through `ros_gz_sim`) and the ROS2-Gazebo bridges for the R1Mk3 model. The related yarpmanager applications are in [app/navigation2/scripts/](../../app/navigation2/scripts/) (e.g. `Navigation_ROS2_R1_SIM_R1Mk3.xml`).

### Tour Guide Robot Applications

```bash
# Inside container, navigate to tour guide applications
cd /usr/local/src/robot/tour-guide-robot

# Build applications
cd build && make -j11

# Run the applications (launch Gazebo first)
yarpmanager
```

## Environment Variables

Key environment variables set in the container:

```bash
ROBOT_CODE=/usr/local/src/robot              # Robot code installation path
YARP_ROOT=/usr/local/src/robot/yarp          # YARP installation
ROS_DISTRO=jazzy                             # ROS2 distribution
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp        # ROS2 middleware
CYCLONEDDS_URI=/home/user1/.config/cyclone_dds_settings.xml  # DDS config
GZ_SIM_YARP_PLUGINS_DIR=/usr/local/src/robot/gz-sim-yarp-plugins/build  # Gazebo-YARP plugins
GZ_SIM_RESOURCE_PATH=.../installed/share:... # Gazebo model resources
YARP_COLORED_OUTPUT=1                        # Colored YARP output
```

## Volume Mounts

Important volume mounts for the container:

- **X11 Display**: `/tmp/.X11-unix:/tmp/.X11-unix:rw`
- **DDS Configuration**: `~/.config/cyclone_dds_settings.xml:/home/user1/.config/cyclone_dds_settings.xml`

## Simulation Features

### Fake Device Plugins (Simulation)
The image includes various fake device plugins for simulation:
- `fakeAnalogSensor`, `fakeBattery`, `fakeDepthCamera`
- `fakeIMU`, `fakeLaser`, `fakeLocalizer`
- `fakeMicrophone`, `fakeMotionControl`, `fakeNavigation`
- `fakeSpeaker`, `fakebot`

### Gazebo Integration
- **gz-sim-yarp-plugins**: Direct integration between YARP and Gazebo
- **R1 Models**: Complete R1 robot models for simulation
- **Custom Worlds**: Simulation environments for tour guide scenarios (see `app/maps/`)

## Troubleshooting

### Display Issues
- Ensure `xhost + local:docker` is run on the host (the start scripts do it for you)
- Check X11 forwarding: `echo $DISPLAY`
- Verify X11 socket permissions

### Gazebo Issues
- Check Gazebo installation: `gz sim --version`
- Verify model paths: `echo $GZ_SIM_RESOURCE_PATH`
- Test plugin loading: Check Gazebo console for plugin errors

### GPU Issues
- Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- Test GPU access: `nvidia-smi` on host
- Add `--env-file env_config/cuda.env` for CUDA-based images
- Use the `r1-toursim-container-nogpu` service on machines without an NVIDIA GPU

### Network Issues
- Check `ROS_DOMAIN_ID` conflicts (export it on the host to pass it to the container)
- Verify DDS configuration file exists
- Set `CYCLONE_DDS_CONF` to the fallback DDS config in `app/navigation2/conf/`

### YARP-Gazebo Integration
- Check plugin paths: `echo $GZ_SIM_SYSTEM_PLUGIN_PATH`
- Verify YARP server is running: `yarp name list`
- Test plugin communication in Gazebo console

## Ports

- **10000**: YARP port (TCP/UDP)

## Default Credentials

- **User**: `user1`
- **Sudo**: Passwordless sudo enabled
- **Working Directory**: `/home/user1`

## Network Configuration

The container uses `host` networking mode for seamless integration with YARP, ROS2, and Gazebo systems. The default YARP configuration connects to `172.17.0.1:10000` for Docker network compatibility.
