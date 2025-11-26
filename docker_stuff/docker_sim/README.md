# R1 Tour Guide Robot Simulation Docker Environment

This Docker environment provides the complete simulation stack for the R1 Tour Guide Robot. The image includes all components necessary for robot simulation, including Gazebo Harmonic, YARP-Gazebo integration, robot models, and simulation-specific plugins for testing and development in a virtual environment.

## Docker Image Contents

The Docker image (`elandini84/r1images:tourSim2_*`) includes:

- **Base System**: Ubuntu 24.04 with development tools and X11 support
- **ROS2 Framework**: Complete ROS2 installation (Jazzy by default) with Navigation2, SLAM, and testing tools
- **YARP Middleware**: Full YARP installation with Python bindings and simulation-specific device plugins
- **Gazebo Simulation**:
  - Gazebo Harmonic for physics simulation
  - gz-sim-yarp-plugins for YARP-Gazebo integration
  - CLI11 dependency for Gazebo plugins
- **Robotology Stack**:
  - YCM (YARP CMake Modules)
  - iCub-main with cartesian controllers (simulation-focused)
  - CER (Collaborative Effort Robot) components
  - Ergocub software and R1 robot models
- **Simulation Components**:
  - CER-SIM with ROS2 support
  - R1 robot models and environments
  - Fake device plugins for simulation (fakeBattery, fakeIMU, fakeLocalizer, etc.)
- **Navigation Stack**: Advanced navigation with ROS2 integration
- **Development Tools**: Web teleop interface, CMake, Git, and debugging tools
- **Network Middleware**: CycloneDDS for improved ROS2 communication

## Prerequisites

- Docker installed
- NVIDIA Docker runtime (for GPU support with CUDA images)
- X11 forwarding capability
- CycloneDDS configuration file (optional, uses default if not provided)

## Building the Docker Image

### Using the Management Script (Recommended)

The `manage-docker.sh` script provides a convenient interface for building and running the Docker containers:

```bash
# Make the script executable
chmod +x manage-docker.sh

# Build with default options (Ubuntu 24.04, Jazzy, Harmonic, devel tag)
./manage-docker.sh --build

# Build with specific options
./manage-docker.sh --build --ubuntu --ros_distro jazzy --gz_version harmonic --yarp_branch yarp-3.12 --devel
```

### Using Docker Build Directly

```bash
# Build with default arguments
docker build -t r1-toursim:custom .

# Build with custom arguments
docker build \
  --build-arg base_img="ubuntu:24.04" \
  --build-arg ros_distro="jazzy" \
  --build-arg yarp_branch="yarp-3.12" \
  --build-arg gazebo_version="harmonic" \
  --build-arg ros2_dev_remote="robotology" \
  --build-arg ros2_dev_branch="master" \
  -t r1-toursim:custom .
```

### Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `base_img` | - | Base Docker image (ubuntu:24.04 or nvidia/cuda:*) |
| `ros_distro` | - | ROS2 distribution name |
| `yarp_branch` | `master` | YARP version/branch to build |
| `gazebo_version` | `harmonic` | Gazebo version to install |
| `ros2_dev_remote` | `robotology` | Remote repository for YARP-ROS2 devices |
| `ros2_dev_branch` | `${yarp_branch}` | Branch for YARP-ROS2 devices |
| `gazebo_pkg` | `ros-${ros_distro}-ros-gz` | ROS2-Gazebo integration package |

## Build and Run Options

### Management Script Options

#### Base Image Selection
```bash
# Ubuntu-based image (CPU only)
./manage-docker.sh --ubuntu [other-options]

# NVIDIA CUDA-based image (GPU support)
./manage-docker.sh --cuda [other-options]
```

#### Build Type
```bash
# Development build (latest features)
./manage-docker.sh --devel [other-options]

# Stable build
./manage-docker.sh --stable [other-options]
```

#### ROS2 Distribution
```bash
# Specify ROS2 distro
./manage-docker.sh --ros_distro humble [other-options]
./manage-docker.sh --ros_distro jazzy [other-options]
```

#### Gazebo Version
```bash
# Specify Gazebo version
./manage-docker.sh --gz_version harmonic [other-options]
./manage-docker.sh --gz_version garden [other-options]
```

#### YARP Version
```bash
# Specify YARP branch/version
./manage-docker.sh --yarp_branch yarp-3.12 [other-options]
./manage-docker.sh --yarp_branch master [other-options]
```

#### Repository Configuration
```bash
# Use custom repository
./manage-docker.sh --repo "myrepo/r1images" [other-options]
```

#### DDS Configuration
```bash
# Use internal CycloneDDS config
./manage-docker.sh --inner_cycl_dds [other-options]

# Use custom DDS config file
./manage-docker.sh --dds_conf_path "/path/to/cyclone_dds_settings.xml" [other-options]
```

#### GPU Support
```bash
# Run without GPU (Ubuntu images only)
./manage-docker.sh --nogpu [other-options]
```

### Running the Container

#### Using the Management Script

```bash
# Run with default options (GPU enabled if CUDA image)
./manage-docker.sh --ubuntu --ros_distro jazzy --gz_version harmonic --devel

# Run without GPU support
./manage-docker.sh --ubuntu --ros_distro jazzy --gz_version harmonic --devel --nogpu

# Print the command without executing
./manage-docker.sh --ubuntu --ros_distro jazzy --gz_version harmonic --devel --print
```

#### Manual Docker Run

```bash
# With GPU support (CUDA images)
sudo docker run --rm -it --privileged --network host --pid host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -e ROS_DOMAIN_ID=37 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e QT_X11_NO_MITSHM=1 --gpus all \
  elandini84/r1images:tourSim2_ubuntu24.04_jazzy_devel

# Without GPU support (Ubuntu images)
sudo docker run --rm -it --privileged --network host --pid host \
  -e DISPLAY -e ROS_DOMAIN_ID=37 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e QT_X11_NO_MITSHM=1 \
  elandini84/r1images:tourSim2_ubuntu24.04_jazzy_devel bash
```

## Usage Examples

### Basic Simulation Development
```bash
# Build and run simulation container
./manage-docker.sh --build --ubuntu --ros_distro jazzy --gz_version harmonic --devel

# Run the built container
./manage-docker.sh --ubuntu --ros_distro jazzy --gz_version harmonic --devel
```

### Gazebo Simulation Environment
```bash
# Inside the container, start YARP server
yarpserver --write

# In another terminal inside container, launch Gazebo
gz sim

# Or launch with specific world file
gz sim /usr/local/src/robot/tour-guide-robot/app/maps/SIM_GAM/GAM.world
```

### R1 Robot Simulation
```bash
# Inside container, navigate to robot models
cd /usr/local/src/robot/r1-models

# Launch R1 simulation (example)
gz sim worlds/r1_world.sdf

# In another terminal, run YARP applications
yarpmanager
```

### Tour Guide Robot Simulation
```bash
# Inside container, navigate to tour guide applications
cd /usr/local/src/robot/tour-guide-robot

# Build applications
cd build && make -j11

# Run simulation with tour guide behavior
# (Launch gazebo first, then run specific tour applications)
```

## Environment Variables

Key environment variables set in the container:

```bash
ROBOT_CODE=/usr/local/src/robot              # Robot code installation path
YARP_ROOT=/usr/local/src/robot/yarp          # YARP installation
ROS_DISTRO=jazzy                             # ROS2 distribution
RMW_IMPLEMENTATION=rmw_cyclonedx_cpp         # ROS2 middleware
GZ_SIM_YARP_PLUGINS_DIR=/usr/local/src/robot/gz-sim-yarp-plugins/build  # Gazebo-YARP plugins
GZ_SIM_RESOURCE_PATH=.../installed/share:... # Gazebo model resources
ROS_DOMAIN_ID=37                             # ROS2 domain ID
YARP_COLORED_OUTPUT=1                        # Colored YARP output
```

## Volume Mounts

Important volume mounts for the container:

- **X11 Display**: `/tmp/.X11-unix:/tmp/.X11-unix:rw`
- **DDS Configuration**: `~/.config/cyclone_dds_settings.xml:/home/user1/.config/cyclone_dds_settings.xml` (optional)

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
- **Custom Worlds**: Simulation environments for tour guide scenarios

## Troubleshooting

### Display Issues
- Ensure `sudo xhost +` is run on the host (handled automatically by management script)
- Check X11 forwarding: `echo $DISPLAY`
- Verify X11 socket permissions

### Gazebo Issues
- Check Gazebo installation: `gz sim --version`
- Verify model paths: `echo $GZ_SIM_RESOURCE_PATH`
- Test plugin loading: Check Gazebo console for plugin errors

### GPU Issues (for CUDA images)
- Install NVIDIA Docker runtime: `sudo apt-get install nvidia-docker2`
- Test GPU access: `nvidia-smi` on host
- Use `--cuda` flag for GPU-enabled images

### Network Issues
- Check ROS_DOMAIN_ID: Default is 37, ensure no conflicts
- Verify DDS configuration if using custom settings
- Use `--inner_cycl_dds` for fallback DDS config

### YARP-Gazebo Integration
- Check plugin paths: `echo $GZ_SIM_SYSTEM_PLUGIN_PATH`
- Verify YARP server is running: `yarp name list`
- Test plugin communication in Gazebo console

## Ports

- **10000**: YARP port (TCP/UDP)
- **11345**: Gazebo communication port

## Default Credentials

- **User**: `user1`
- **Sudo**: Passwordless sudo enabled
- **Working Directory**: `/usr/local/src/robot`

## Network Configuration

The container uses `host` networking mode for seamless integration with YARP, ROS2, and Gazebo systems. The default YARP configuration connects to `172.17.0.1:10000` for Docker network compatibility, and ROS_DOMAIN_ID is set to 37 to avoid conflicts with other ROS2 systems.