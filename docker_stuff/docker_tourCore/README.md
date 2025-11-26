# R1 Tour Guide Robot Core Docker Environment

This Docker environment provides the complete robotics stack for the R1 Tour Guide Robot. The image includes all core components necessary for robot navigation, control, simulation, and interaction with both YARP and ROS2 ecosystems.

## Docker Image Contents

The Docker image (`elandini84/r1images:tourCore2_*`) includes:

- **Base System**: Ubuntu 24.04 with development tools and X11 support
- **ROS2 Framework**: Complete ROS2 installation (Jazzy by default) with Navigation2, SLAM, and Gazebo
- **YARP Middleware**: Full YARP installation with Python bindings and device plugins
- **Robotology Stack**:
  - YCM (YARP CMake Modules)
  - iCub-main with cartesian controllers and sensor modules
  - CER (Collaborative Effort Robot) components
  - Ergocub software and R1 robot models
- **Navigation Components**: Advanced navigation stack with ROS2 integration
- **Simulation Environment**: CER-SIM with ROS2 support and Gazebo integration
- **Hardware Interfaces**: RPLidar support and YARP-ROS2 device bridges
- **Development Tools**: Groot (BehaviorTree visualization), web teleop, CMake, Git, and debugging tools
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

# Build with default options (Ubuntu 24.04, Jazzy, devel tag)
./manage-docker.sh --build

# Build with specific options
./manage-docker.sh --build --ubuntu --ros_distro jazzy --yarp_branch yarp-3.12 --devel
```

### Using Docker Build Directly

```bash
# Build with default arguments
docker build -t r1-tourcore:custom .

# Build with custom arguments
docker build \
  --build-arg base_img="ubuntu:24.04" \
  --build-arg ros_distro="jazzy" \
  --build-arg yarp_branch="yarp-3.12" \
  --build-arg ros2_dev_remote="robotology" \
  --build-arg ros2_dev_branch="master" \
  --build-arg battery="OFF" \
  -t r1-tourcore:custom .
```

### Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `base_img` | - | Base Docker image (ubuntu:24.04 or nvidia/cuda:*) |
| `ros_distro` | - | ROS2 distribution name |
| `yarp_branch` | `master` | YARP version/branch to build |
| `ros2_dev_remote` | `robotology` | Remote repository for YARP-ROS2 devices |
| `ros2_dev_branch` | `${yarp_branch}` | Branch for YARP-ROS2 devices |
| `gazebo_pkg` | `ros-${ros_distro}-ros-gz` | Gazebo package to install |
| `battery` | `OFF` | Enable battery-related iCub modules |

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
./manage-docker.sh --ubuntu --ros_distro jazzy --devel

# Run without GPU support
./manage-docker.sh --ubuntu --ros_distro jazzy --devel --nogpu

# Print the command without executing
./manage-docker.sh --ubuntu --ros_distro jazzy --devel --print
```

#### Manual Docker Run

```bash
# With GPU support (CUDA images)
sudo docker run --rm -it --privileged --network host --pid host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/.config/cyclone_dds_settings.xml:/home/user1/.config/cyclone_dds_settings.xml \
  -e QT_X11_NO_MITSHM=1 --gpus all \
  elandini84/r1images:tourCore2_ubuntu24.04_jazzy_devel

# Without GPU support (Ubuntu images)
sudo docker run --rm -it --privileged --network host --pid host \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/.config/cyclone_dds_settings.xml:/home/user1/.config/cyclone_dds_settings.xml \
  -e QT_X11_NO_MITSHM=1 \
  elandini84/r1images:tourCore2_ubuntu24.04_jazzy_devel
```

## Usage Examples

### Basic Development Session
```bash
# Build and run development container
./manage-docker.sh --build --ubuntu --ros_distro jazzy --devel

# Run the built container
./manage-docker.sh --ubuntu --ros_distro jazzy --devel
```

### Tour Guide Robot Applications
```bash
# Inside container, navigate to robot applications
cd /usr/local/src/robot/tour-guide-robot

# Build tour guide applications
cd build && make -j11

# Run tour manager
yarpmanager
```

## Environment Variables

Key environment variables set in the container:

```bash
ROBOT_CODE=/usr/local/src/robot              # Robot code installation path
YARP_ROOT=/usr/local/src/robot/yarp          # YARP installation
ROS_DISTRO=jazzy                             # ROS2 distribution
RMW_IMPLEMENTATION=rmw_cyclonedx_cpp         # ROS2 middleware
CYCLONEDDS_URI=/home/user1/.config/cyclone_dds_settings.xml  # DDS config
YARP_COLORED_OUTPUT=1                        # Colored YARP output
```

## Volume Mounts

Important volume mounts for the container:

- **X11 Display**: `/tmp/.X11-unix:/tmp/.X11-unix:rw`
- **DDS Configuration**: `~/.config/cyclone_dds_settings.xml:/home/user1/.config/cyclone_dds_settings.xml`
- **USB Devices**: `/dev/ttyUSB*` (for hardware sensors like RPLidar)

## Troubleshooting

### Display Issues
- Ensure `sudo xhost +` is run on the host
- Check X11 forwarding: `echo $DISPLAY`
- Verify X11 socket permissions

### GPU Issues
- Install NVIDIA Docker runtime: `sudo apt-get install nvidia-docker2`
- Test GPU access: `nvidia-smi` on host
- Use `--cuda` flag for GPU-enabled images

### Network Issues
- Check ROS_DOMAIN_ID conflicts
- Verify DDS configuration file exists
- Use `--inner_cycl_dds` for fallback DDS config

### USB Device Access
- Add user to dialout group on host: `sudo usermod -a -G dialout $USER`
- Check device permissions: `ls -la /dev/ttyUSB*`
- Container automatically adjusts permissions for `/dev/ttyUSB0` and `/dev/ttyUSB1`

## Ports

- **10000**: YARP port (TCP/UDP)
- **11311**: ROS Master (if using ROS1 bridge)

## Default Credentials

- **User**: `user1`
- **Sudo**: Passwordless sudo enabled
- **Working Directory**: `/home/user1`

## Network Configuration

The container uses `host` networking mode for seamless integration with YARP and ROS2 systems. This allows direct communication with hardware and other containers without port mapping.

The default YARP configuration connects to `192.168.100.10:10000` for R1 robot network compatibility.