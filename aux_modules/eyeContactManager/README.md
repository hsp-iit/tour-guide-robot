# Eye Contact Manager Module

## Overview

The Eye Contact Manager module is responsible for managing eye contact behavior with humans during robot navigation. It detects when people are looking at the robot, tracks the closest person making eye contact, and controls the robot's head/gaze to maintain eye contact when the robot is not moving. The module integrates with the navigation system to avoid head movements during robot motion.

## Key Features

- **Person Detection Analysis**: Processes person detection data to identify people looking at the robot
- **Proximity-based Filtering**: Only considers people within 1.2 meters for eye contact
- **Closest Person Tracking**: Prioritizes the closest person when multiple people are looking
- **Navigation-aware Behavior**: Resets head position when robot is moving
- **Timeout Management**: Automatically resets head position after 6 seconds without eye contact
- **Depth-based Gaze Control**: Uses depth camera information for precise gaze targeting

## Dependencies

### System Dependencies
- **CMake** (>= 3.12): Build system
- **C++ Compiler**: Supporting C++11 standard
- **YARP** (Yet Another Robot Platform): Middleware for robot communication
  - Components needed: `os`, `dev`, `init`

### Ubuntu/Debian Installation

#### Basic Dependencies
```bash
sudo apt-get update
sudo apt-get install cmake build-essential
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
1. Navigate to the project root directory:
```bash
cd /path/to/tour-guide-robot
```

2. Create and enter build directory:
```bash
mkdir build && cd build
```

3. Configure the build:
```bash
cmake ..
```

4. Build the project:
```bash
make
```

The executable will be created in `build/bin/eyeContactManager`.

### Building Standalone
If you want to build only the eyeContactManager module:

1. Navigate to the eyeContactManager directory:
```bash
cd aux_modules/eyeContactManager
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

**Note**: For standalone building, you'll need to ensure that the `headSynchronizerRPC` interface is built first from the `interfaces/headSynchronizerRPC` directory.

## Configuration Parameters

The module uses hardcoded parameters but connects to specific services:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Module Name | "eyeContactManager" | Fixed name for the module |
| Update Period | 0.1 seconds | Module update frequency |
| Eye Contact Timeout | 6.0 seconds | Time before resetting head position |
| Max Detection Distance | 1.2 meters | Maximum depth for person detection |
| Min Detection Distance | 0.0 meters | Minimum depth for person detection |

### Service Connections
| Service | Default Port | Description |
|---------|--------------|-------------|
| Navigation Server | `/navigation2D_nws_yarp` | Navigation control service |
| Map Server | `/map2D_nws_yarp` | Map service for localization |
| Localization Server | `/localization2D_nws_yarp` | Robot localization service |

## Execution

### Prerequisites
Before running the eyeContactManager module, ensure that:

1. **YARP Server is running**:
   ```bash
   yarp server
   ```

2. **Navigation Services are running**:
   - Navigation2D server (`/navigation2D_nws_yarp`)
   - Map2D server (`/map2D_nws_yarp`)
   - Localization server (`/localization2D_nws_yarp`)

3. **Person Detection System**: A system that publishes person detection data with gaze information to the input port

4. **Gaze Control System**: A gaze control system that accepts gaze commands

### Running the Module

1. Make sure the necessary services are running (yarp server, navigation services, etc.)

2. Run the executable:
```bash
./eyeContactManager
```

### Running with Configuration File
Create a configuration file (e.g., `eyeContactManager.ini`):
```ini
# Currently no configurable parameters via file
# Module uses hardcoded configuration
```

Then run:
```bash
./eyeContactManager --from eyeContactManager.ini
```

### YARP Port Information
The module creates the following YARP ports:
- `/eyeContactManager/pred:i`: Input port for person detection data
- `/eyeContactManager/pred:o`: Output port for headSynchronizer RPC calls
- `/eyeContactManager/control:o`: Output port for gaze control commands
- `/eyeContactManager/navigation2D_nwc_yarp`: Local port for navigation client

## How It Works

1. **Person Detection Processing**: Receives person detection data including position, depth, gaze direction, and confidence
2. **Eye Contact Analysis**: Identifies people looking at the robot within the specified distance range
3. **Closest Person Selection**: When multiple people are looking, selects the closest person based on depth information
4. **Navigation Status Check**: Monitors robot movement status to determine appropriate behavior
5. **Gaze Control**:
   - When robot is stationary and person is looking: directs gaze to person's face
   - When robot is moving: resets head to neutral position
   - When no eye contact for 6 seconds: resets head position
6. **Head Reset Management**: Implements timeout-based head position reset to avoid getting stuck

## Input Data Format

The module expects person detection data in the following format:
```
[person_id [x_center y_center] depth looking_flag confidence]
```

Where:
- `person_id`: Unique identifier for the person
- `x_center, y_center`: Pixel coordinates of person center (will be divided by 2)
- `depth`: Distance to person in meters
- `looking_flag`: 1 if person is looking at robot, 0 otherwise
- `confidence`: Detection confidence score

## Dependencies on Other Modules

This module requires:
- **YARP Server**: Must be running for inter-process communication
- **Navigation2D Services**: Required for checking robot movement status
- **Person Detection Pipeline**: Must publish person data with gaze information
- **Gaze Control System**: Must accept and execute gaze control commands
- **headSynchronizerRPC Service**: For head synchronization functionality

## Troubleshooting

### Common Issues

1. **YARP Connection errors**:
   - Ensure `yarp server` is running
   - Check if YARP namespace is correctly configured: `yarp namespace`
   - Verify ports are not already in use: `yarp name list`

2. **Navigation Service errors**:
   - Ensure navigation2D services are running and accessible
   - Check service availability: `yarp name list | grep navigation`
   - Verify navigation server configuration

3. **No person detection**:
   - Verify that the person detection pipeline is active and publishing data
   - Check input port connection: `yarp name query /eyeContactManager/pred:i`
   - Monitor input data: `yarp read ... /eyeContactManager/pred:i`

4. **Gaze control not working**:
   - Ensure gaze control system is running and accepting commands
   - Check gaze control port connection: `yarp name query /eyeContactManager/control:o`
   - Verify gaze control system configuration

5. **Build errors**:
   - Make sure YARP is correctly installed and in the system path
   - Verify that headSynchronizerRPC interface is built
   - Check that all YARP components (os, dev, init) are available

6. **Port binding errors**:
   - Check if ports are already in use
   - Use `yarp clean` to remove stale port registrations
   - Verify network connectivity between modules

### Debug Mode
The module uses YARP logging. Enable debug output by setting the appropriate log level:

```bash
export YARP_VERBOSE=1
./eyeContactManager
```

### YARP Network Diagnostics
```bash
# Check YARP server status
yarp detect --write

# List all active ports
yarp name list

# Check specific port connections
yarp name query /eyeContactManager/pred:i
yarp name query /eyeContactManager/control:o

# Monitor port data
yarp read ... /eyeContactManager/pred:i
```

## Module States and Behavior

### EyeContactStatus States
- **NOBODY**: No people detected in range
- **NOT_LOOKING**: People detected but none looking at robot
- **LOOKING**: At least one person is looking at robot

### Behavioral Logic
- **Robot Moving**: Head is reset to neutral position regardless of eye contact
- **Robot Stationary + Eye Contact**: Head tracks closest person looking
- **No Eye Contact for 6+ seconds**: Head automatically resets to neutral
- **Multiple People Looking**: Prioritizes closest person by depth

## Performance Considerations

- Module updates at 10Hz (0.1-second period)
- Distance filtering (0-1.2m) reduces computational load
- Navigation status checking prevents unnecessary head movements
- Timeout mechanism prevents head from staying in extreme positions