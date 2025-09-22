# Crowd Detector Module

## Overview

The Crowd Detector module is responsible for detecting and counting humans in a specific area around a Point of Interest (POI). It uses frame transformation to track human positions and determines if they are within a defined radius around a POI.

## Dependencies

### System Dependencies
- **CMake** (>= 3.12): Build system
- **C++ Compiler**: Supporting C++11 standard
- **OpenCV**: Computer vision library for image processing
- **YARP** (Yet Another Robot Platform): Middleware for robot communication
  - Components needed: `os`, `sig`, `cv`, `dev`, `math`

### Option 1: Conda Installation (Recommended)
```bash
# Install Miniconda/Anaconda if not already installed
# Download from https://docs.conda.io/en/latest/miniconda.html

# Create and activate a new conda environment
conda create -n crowddetector-env
conda activate crowddetector-env

# Install dependencies from conda-forge
conda install -c conda-forge cmake compilers opencv yarp

# Note: This will install YARP, OpenCV, and build tools in the conda environment
```

### Option 2: Ubuntu/Debian Installation
```bash
sudo apt-get update
sudo apt-get install cmake build-essential
sudo apt-get install libopencv-dev

# Install YARP dependencies
sudo apt-get install libace-dev libeigen3-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5opengl5-dev libopencv-dev

# Install YARP (either from package manager or compile from source)
# Option 2a: From package manager (if available)
sudo apt-get install libyarp-dev yarp

# Option 2b: Compile from source (recommended for latest version)
git clone https://github.com/robotology/yarp.git
cd yarp
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

### Fedora/CentOS Installation
```bash
sudo yum install cmake gcc-c++
sudo yum install opencv-devel

# Install YARP dependencies
sudo yum install ace-devel eigen3-devel sqlite-devel tinyxml-devel qt5-qtbase-devel qt5-qtdeclarative-devel qt5-qtmultimedia-devel

# Compile YARP from source
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

3. Configure the build (ensure OpenCV is enabled):
```bash
cmake .. -DOPENCV_ON=ON
```

4. Build the project:
```bash
make
```

The executable will be created in `build/bin/crowdDetector`.

### Building Standalone
If you want to build only the crowdDetector module:

1. Navigate to the crowdDetector directory:
```bash
cd aux_modules/crowdDetector
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

## Configuration Parameters

The module accepts the following configuration parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | "crowdDetector" | Name of the module |
| `targetFrame` | string | "mobile_base_body_link" | Target frame for coordinate transformation |
| `m_area_radius` | double | 0.5 | Radius around the POI to count people (in meters) |
| `remoteTC` | string | "/transformServer" | Remote port for FrameTransformClient |

## Execution

### Prerequisites
Before running the crowdDetector module, ensure that:

1. **YARP Server is running**:
   ```bash
   yarp server
   ```

2. **Transform Server is available**: The module requires a transform server to be running (typically `/transformServer`)

3. **Human Detection Pipeline**: Make sure a human detection system is running that publishes human frames in the format `/human#/shoulderCenter`

### Running the Module

1. Make sure the necessary services are running (yarp server, transform server, etc.)

2. Run the executable:
```bash
./crowdDetector
```

### Running with Configuration File
Create a configuration file (e.g., `crowdDetector.ini`):
```ini
name crowdDetector
targetFrame mobile_base_body_link
m_area_radius 1.0
remoteTC /transformServer
```

Then run:
```bash
./crowdDetector --from crowdDetector.ini
```

### Running with Command Line Parameters
```bash
./crowdDetector --name crowdDetector --targetFrame mobile_base_body_link --m_area_radius 1.0 --remoteTC /transformServer
```

### YARP Port Information
The module creates the following YARP ports:
- `/myModule`: Handler port for receiving commands and queries
- `/{name}/transformClient`: Client port for frame transformations (where `{name}` is the module name parameter)

## How It Works

1. **Frame Detection**: The module scans for human frame IDs in the format `/human#/shoulderCenter`
2. **Position Calculation**: It retrieves the position of detected humans through frame transformations
3. **Area Filtering**: Only humans within the specified radius around the POI (default at position [1.5, 0]) are counted
4. **Persistence**: Implements a 3-second persistence mechanism to avoid false negatives when humans temporarily disappear from detection
5. **Continuous Monitoring**: Updates every 0.5 seconds to provide real-time crowd information

## Output

The module outputs:
- Real-time count of humans in the specified area
- Debug information about detected human positions
- Persistence-filtered crowd dimension data

## Module Lifecycle

- **Initialization**: Opens ports, connects to transform client, sets up configuration
- **Main Loop**: Continuously monitors for humans every 0.5 seconds
- **Cleanup**: Properly closes ports and connections on shutdown

## Dependencies on Other Modules

This module requires:
- **YARP Server**: Must be running for inter-process communication
- **Transform Server**: Required for coordinate transformations between frames
- **Human Detection System**: Must publish human frames (e.g., OpenPose-based detection) in the format `/human#/shoulderCenter`

## Troubleshooting

### Common Issues

1. **YARP Connection errors**:
   - Ensure `yarp server` is running
   - Check if YARP namespace is correctly configured: `yarp namespace`
   - Verify ports are not already in use: `yarp name list`

2. **Transform errors**:
   - Ensure the transform server is running and accessible at the specified `remoteTC` port
   - Verify the target frame exists and is being published
   - Check frame transformation chain: `yarp rpc /transformServer`

3. **No human detection**:
   - Verify that the human detection pipeline is active and publishing frames
   - Check available frames: use YARP tools to list active frame transforms
   - Ensure human frames follow the expected naming convention `/human#/shoulderCenter`

4. **Build errors**:
   - Make sure OpenCV is properly installed and CMake can find it
   - Verify YARP is correctly installed and in the system path
   - Check that all YARP components (os, sig, cv, dev, math) are available

5. **Port binding errors**:
   - Check if port `/myModule` is already in use
   - Use `yarp clean` to remove stale port registrations
   - Try running with a different module name: `--name crowdDetector2`

### Debug Mode
The module uses YARP logging. Enable debug output by setting the appropriate log level to see detailed frame transformation information:

```bash
export YARP_VERBOSE=1
./crowdDetector
```

### YARP Network Diagnostics
```bash
# Check YARP server status
yarp detect --write

# List all active ports
yarp name list

# Check specific port connections
yarp name query /myModule

# Monitor port data (replace with actual port name)
yarp read ... /human1/shoulderCenter
```