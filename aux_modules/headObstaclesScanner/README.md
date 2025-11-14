# Head Obstacles Scanner Module

## Overview

The Head Obstacles Scanner module controls the robot's head movement to scan for obstacles and monitor the environment during navigation. It provides two main operation modes: a sweeping mode for general environment scanning and a trajectory mode that follows navigation waypoints. The module integrates with the robot's navigation system to provide intelligent head positioning that enhances obstacle detection and situational awareness.

## Key Features

- **Dual Operation Modes**:
  - **Sweep Mode**: Continuous left-right head sweeping for general scanning
  - **Trajectory Mode**: Intelligent head positioning based on navigation waypoints
- **Visual Feedback**: Real-time map visualization with robot position, waypoints, and camera FOV
- **Navigation Integration**: Seamless integration with YARP Navigation2D system
- **Configurable Parameters**: Adjustable head speed, rotation range, camera FOV, and detection radius
- **Head Position Broadcasting**: Publishes current head encoder values for other modules
- **Safety Features**: Automatic head reset when navigation goals are reached

## Technical Specifications

- **Update Frequency**: 2 Hz (0.5-second period)
- **Head Control**: Position and direct control modes via YARP control board interfaces
- **Camera FOV**: Default 70° field of view with configurable range
- **Detection Range**: Up to 3.5 meters (configurable)
- **Map Visualization**: Real-time OpenCV-based map rendering with navigation overlays

## Dependencies

### System Dependencies
- **CMake** (>= 3.12): Build system
- **C++ Compiler**: Supporting C++11 standard
- **OpenCV**: Computer vision library for image processing and visualization
- **YARP** (Yet Another Robot Platform): Middleware for robot communication
  - Components needed: `os`, `sig`, `cv`, `dev`, `math`

### Ubuntu/Debian Installation

#### Basic Dependencies
```bash
sudo apt-get update
sudo apt-get install cmake build-essential
sudo apt-get install libopencv-dev
```

#### YARP Installation

**Option 1: Using Conda (Recommended)**
```bash
# Install conda/miniconda if not already installed
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Create a new environment with YARP
conda create -n robotics-env
conda activate robotics-env

# Install YARP from conda-forge
conda install -c conda-forge -c robotology yarp
```

**Option 2: From Package Manager**
```bash
# Install YARP dependencies
sudo apt-get install libace-dev libeigen3-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5opengl5-dev

# Install YARP (if available in repositories)
sudo apt-get install libyarp-dev yarp
```

**Option 3: Build from Source**
```bash
# Install YARP dependencies
sudo apt-get install libace-dev libeigen3-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5opengl5-dev

# Clone and build YARP
git clone https://github.com/robotology/yarp.git
cd yarp
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

### Fedora/CentOS Installation

#### Basic Dependencies
```bash
sudo yum install cmake gcc-c++
sudo yum install opencv-devel
```

#### YARP Installation

**Option 1: Using Conda (Recommended)**
```bash
# Install conda/miniconda if not already installed
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Create a new environment with YARP
conda create -n robotics-env
conda activate robotics-env

# Install YARP from conda-forge
conda install -c conda-forge -c robotology yarp
```

**Option 2: Build from Source**
```bash
# Install YARP dependencies
sudo yum install ace-devel eigen3-devel sqlite-devel tinyxml-devel qt5-qtbase-devel qt5-qtdeclarative-devel qt5-qtmultimedia-devel

# Clone and build YARP
git clone https://github.com/robotology/yarp.git
cd yarp
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

## Build Instructions

### Building as Part of the Main Project

**Important**: This module is only built when OpenCV is available and `OPENCV_ON` flag is set.

1. Navigate to the project root directory:
```bash
cd /path/to/tour-guide-robot
```

2. Create and enter build directory:
```bash
mkdir build && cd build
```

3. Configure the build (OpenCV will be auto-detected):
```bash
cmake ..
```

4. Build the project:
```bash
make
```

The executable will be created in `build/bin/headObstaclesScanner`.

### Building Standalone
If you want to build only the headObstaclesScanner module:

1. Navigate to the headObstaclesScanner directory:
```bash
cd aux_modules/headObstaclesScanner
```

2. Create and enter build directory:
```bash
mkdir build && cd build
```

3. Configure and build:
```bash
cmake ..
make
```

**Note**: Ensure that OpenCV and YARP are properly installed and findable by CMake.

## Configuration Parameters

The module accepts configuration via INI files with the following groups and parameters:

### [GENERAL] Group
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot` | string | **required** | Robot name (e.g., "icub", "r1") |
| `head_speed` | double | 25.0 | Head rotational speed in degrees/second |
| `rotation_range` | double | 35.0 | Head rotation range in degrees (+/- range) |
| `head_mode` | string | "sweep" | Operation mode: "sweep" or "trajectory" |
| `head_pitch` | double | 10.0 | Fixed robot head pitch in degrees |
| `local` | string | "/headObstaclesScanner" | Local port name prefix |

### [HEAD] Group
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `circle_range` | double | 1.0 | Distance for waypoint detection in meters |
| `camera_fov` | double | 70.0 | Camera field of view in degrees |
| `camera_max_considered_radius` | double | 3.5 | Maximum obstacle detection distance in meters |
| `map_name` | string | "/test/manual/load.png" | Map file path (trajectory mode) |
| `map_resolution` | double | 0.05 | Map resolution in meters/pixel |

### [NAVIGATION] Group (Trajectory Mode Only)
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `remote_localization` | string | "/localization2D_nws_yarp" | Localization server port |
| `remote_navigation` | string | "/navigation2D_nws_yarp" | Navigation server port |
| `remote_map` | string | "/map2D_nws_yarp" | Map server port |

## Operation Modes

### Sweep Mode
- **Behavior**: Continuous left-right head sweeping motion
- **Range**: ±35° (configurable via `rotation_range`)
- **Use Case**: General environment scanning, idle state monitoring
- **Requirements**: Only head control board connection needed

### Trajectory Mode
- **Behavior**: Intelligent head positioning based on navigation waypoints
- **Features**: Looks ahead along planned trajectory, stops movement when goal reached
- **Visual Output**: Real-time map with robot position, waypoints, and camera FOV
- **Requirements**: Full navigation stack (localization, navigation, map services)

## Execution

### Prerequisites
Before running the headObstaclesScanner module, ensure that:

1. **YARP Server is running**:
   ```bash
   yarp server
   ```

2. **Robot Control Board**: Head control board must be accessible
   - For iCub: `/icub/head` control board
   - For R1: `/r1/head` control board

3. **Navigation Services** (Trajectory Mode Only):
   - Localization service (`/localization2D_nws_yarp`)
   - Navigation service (`/navigation2D_nws_yarp`)
   - Map service (`/map2D_nws_yarp`)

### Running the Module

#### Basic Execution
```bash
./headObstaclesScanner --robot icub
```

#### With Configuration File
Create a configuration file (e.g., `headObstaclesScanner.ini`):
```ini
[GENERAL]
robot               icub
head_speed          30.0
rotation_range      40.0
head_mode           sweep
head_pitch          15.0
local               /headObstaclesScanner

[HEAD]
circle_range                    1.5
camera_fov                     70.0
camera_max_considered_radius   4.0
map_resolution                 0.05

[NAVIGATION]
remote_localization    /localization2D_nws_yarp
remote_navigation      /navigation2D_nws_yarp
remote_map             /map2D_nws_yarp
```

Then run:
```bash
./headObstaclesScanner --from headObstaclesScanner.ini
```

#### Trajectory Mode Configuration
For trajectory mode, use:
```ini
[GENERAL]
robot               icub
head_mode           trajectory
head_speed          20.0

[HEAD]
circle_range        2.0
map_name           /path/to/your/map.png

[NAVIGATION]
remote_localization    /localization2D_nws_yarp
remote_navigation      /navigation2D_nws_yarp
remote_map             /map2D_nws_yarp
```

### YARP Port Information
The module creates the following YARP ports:

#### Output Ports
- `/headObstaclesScanner/head_position:o`: Head encoder positions (yaw, pitch)
- `/headObstaclesScanner/rgb:o`: Map visualization image (trajectory mode only)
- `/myModule`: RPC command handler

#### Client Connections (Trajectory Mode)
- `/headObstaclesScanner/localizationClient`: Localization client
- `/headObstaclesScanner/navigation2D_nwc_yarp`: Navigation client

## How It Works

### Sweep Mode Operation
1. **Initialization**: Head moves to neutral position (0°, 0°)
2. **Sweep Cycle**: Alternates between +rotation_range and -rotation_range
3. **Position Broadcasting**: Continuously publishes head encoder values
4. **Speed Control**: Configurable rotational velocity

### Trajectory Mode Operation
1. **Navigation Integration**: Connects to localization and navigation services
2. **Waypoint Analysis**: Retrieves current navigation trajectory
3. **Look-ahead Calculation**: Determines optimal head direction based on upcoming waypoints
4. **Visual Feedback**: Generates real-time map visualization showing:
   - Robot position and orientation
   - Navigation waypoints
   - Camera field of view cone
   - Planned trajectory path
5. **Goal Behavior**: Resets head to neutral when navigation goal is reached

### Visual Output (Trajectory Mode)
The module generates a real-time visualization showing:
- **Red Circle**: Robot position with orientation arrow
- **Blue Circles**: Navigation waypoints
- **Green Triangle**: Camera field of view cone
- **Colored Lines**: Planned trajectory path

## Control Interfaces

### Head Control
- **Position Control**: Standard YARP position control for sweep movements
- **Direct Control**: Direct position control for precise trajectory following
- **Encoder Feedback**: Real-time head position monitoring

### Navigation Integration
- **Waypoint Retrieval**: Gets current navigation trajectory
- **Status Monitoring**: Tracks navigation state (moving, idle, goal reached)
- **Localization**: Continuous robot pose updates

## Dependencies on Other Modules

This module requires:
- **YARP Server**: Must be running for inter-process communication
- **Robot Control Board**: Head motor control interface
- **Navigation Stack** (Trajectory Mode):
  - **Localization Service**: For robot pose information
  - **Navigation Service**: For waypoint and trajectory data
  - **Map Service**: For map data and visualization

## Troubleshooting

### Common Issues

1. **YARP Connection errors**:
   - Ensure `yarp server` is running
   - Check if YARP namespace is correctly configured: `yarp namespace`
   - Verify ports are not already in use: `yarp name list`

2. **Head Control Board errors**:
   - Verify robot control board is running and accessible
   - Check robot name parameter matches actual robot configuration
   - Test head control independently: `yarp motor --robot icub --part head`

3. **Build errors**:
   - Ensure OpenCV is properly installed: `pkg-config --modversion opencv`
   - Verify YARP is correctly installed: `yarp version`
   - Check that all YARP components are available

4. **Navigation Service errors** (Trajectory Mode):
   - Verify navigation services are running:
     ```bash
     yarp name list | grep navigation
     yarp name list | grep localization
     yarp name list | grep map
     ```
   - Check service connectivity and configuration

5. **Map Visualization issues** (Trajectory Mode):
   - Verify map file path and format
   - Check map resolution parameter
   - Ensure sufficient memory for image processing

6. **Head Movement problems**:
   - Check head speed and range parameters
   - Verify encoder feedback is working
   - Monitor head position output port

### Debug Mode
The module includes debug output levels. To enable:

1. Edit the source code to uncomment debug defines:
   ```cpp
   #define DEBUG
   #define DEBUG_LV2
   #define DEBUG_LV3
   ```

2. Recompile and run for detailed debug output

### YARP Network Diagnostics
```bash
# Check YARP server status
yarp detect --write

# List all active ports
yarp name list

# Check head control board
yarp name query /icub/head

# Monitor head position output
yarp read ... /headObstaclesScanner/head_position:o

# View map visualization (trajectory mode)
yarp view /headObstaclesScanner/rgb:o

# Test navigation services
yarp name query /navigation2D_nws_yarp
yarp name query /localization2D_nws_yarp
```

## Performance Considerations

- **Update Frequency**: 2 Hz provides good balance between responsiveness and system load
- **Image Processing**: Map visualization (trajectory mode) uses OpenCV operations
- **Memory Usage**: Map images can consume significant memory - monitor system resources
- **Network Traffic**: Image streaming adds network load in trajectory mode
- **Head Movement**: Continuous movement in sweep mode - consider power consumption

## Customization

### Extending Operation Modes
Add new head movement patterns by:
1. Adding new mode string to configuration
2. Implementing mode-specific logic in `updateModule()`
3. Creating dedicated mode function (following `sweepMode()` pattern)

### Custom Visualizations
Extend `drawImage()` function to add:
- Custom map overlays
- Additional robot information
- Different visualization styles
- Export capabilities

### Integration Examples
```bash
# Connect to head control
yarp connect /headObstaclesScanner/head_position:o /consumer/head:i

# View live map visualization
yarp view /headObstaclesScanner/rgb:o

# Control via RPC
yarp rpc /myModule
>> help

# Monitor in sweep mode
yarp read ... /headObstaclesScanner/head_position:o
```

## Safety Considerations

- **Head Movement Limits**: Respect robot's physical head movement constraints
- **Speed Limits**: Avoid excessive head speeds that could damage hardware
- **Emergency Stop**: Module responds to YARP interrupt signals
- **Navigation Coordination**: Automatic head reset during critical navigation phases