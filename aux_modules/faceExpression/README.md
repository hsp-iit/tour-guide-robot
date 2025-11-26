# Face Expression Module

## Overview

The Face Expression module is responsible for generating animated facial expressions for the R1 robot's face display. It creates a 80x32 pixel RGB image that simulates various facial features including eyes (with blinking animation), ears (with audio visualization bars), mouth (with talking animation), and nose. The module integrates with the robot's audio system to provide synchronized visual feedback during speech and listening activities.

## Key Features

- **Animated Eyes**: Realistic blinking animation with configurable frequency (15 blinks per minute + random variation)
- **Dynamic Ears**: Audio-reactive bars that visualize microphone input and audio recording status
- **Talking Mouth**: Animated mouth movements synchronized with speech output
- **Multi-threaded Architecture**: Separate threads for eyes, ears, mouth, and output rendering
- **RPC Interface**: Remote control via YARP RPC commands for real-time expression control
- **Resource Management**: Configurable image assets and parameters via ResourceFinder

## Technical Specifications

- **Output Resolution**: 80x32 pixels RGB
- **Update Frequency**:
  - Output: ~30 FPS (0.033s period)
  - Features: ~50 FPS (0.020s period)
- **Image Format**: OpenCV Mat (CV_8UC3)
- **Thread Safety**: Mutex-protected shared image buffer

## Dependencies

### System Dependencies
- **CMake** (>= 3.12): Build system
- **C++ Compiler**: Supporting C++11 standard
- **OpenCV**: Computer vision library for image processing and manipulation
- **YARP** (Yet Another Robot Platform): Middleware for robot communication
  - Components needed: `os`, `sig`, `dev`, `math`

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

**Important**: This module requires the `ENABLE_faceExpressionImage` option to be enabled during build.

1. Navigate to the project root directory:
```bash
cd /path/to/tour-guide-robot
```

2. Create and enter build directory:
```bash
mkdir build && cd build
```

3. Configure the build with the face expression module enabled:
```bash
cmake .. -DENABLE_faceExpressionImage=ON
```

4. Build the project:
```bash
make
```

The executable will be created in `build/bin/faceExpressionImage5GTour`.

### Building Standalone
If you want to build only the faceExpression module:

1. Navigate to the faceExpression directory:
```bash
cd aux_modules/faceExpression
```

2. Create and enter build directory:
```bash
mkdir build && cd build
```

3. Configure and build:
```bash
cmake .. -DENABLE_faceExpressionImage=ON
make
```

**Note**: For standalone building, ensure that OpenCV and YARP are properly installed and findable by CMake.

## Configuration Parameters

### Eye Configuration Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| Blink Frequency | 15 + (0-6) per minute | Base frequency plus random variation |
| Eye Width | 21 pixels | Width of each eye |
| Eye Height | 14 pixels | Height of each eye |
| Left Eye Position | (9, 9) | X,Y coordinates from top-left |
| Right Eye Position | (50, 9) | X,Y coordinates from top-left |

### Ear Bar Configuration Parameters
| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `earBar0_x` | 1 | - | Outer ear bar horizontal offset |
| `earBar0_y` | 6 | - | Outer ear bar vertical offset |
| `earBar0_minLen` | 3 | - | Minimum length of outer ear bar |
| `earBar0_maxLen` | 18 | - | Maximum length of outer ear bar |
| `earBar1_x` | 3 | - | Inner ear bar horizontal offset |
| `earBar1_y` | 6 | - | Inner ear bar vertical offset |
| `earBar1_minLen` | 4 | - | Minimum length of inner ear bar |
| `earBar1_maxLen` | 19 | - | Maximum length of inner ear bar |

### Mouth Configuration
| Parameter | Default | Description |
|-----------|---------|-------------|
| Mouth Width | 16 pixels | Width of mouth area |
| Mouth Height | 5 pixels | Height of mouth area |
| Position | Center bottom | Automatically centered horizontally |

## Required Image Assets

The module requires the following image files in the `images/` directory:

### Eye Animation Frames
- `blink_1.bmp` through `blink_6.bmp`: Eye animation sequence
- Format: BMP, color images matching eye dimensions (21x14)

### Nose Asset
- `noseBar.bmp`: Static nose image
- Format: BMP, color image

### Asset Directory Structure
```
images/
├── blink_1.bmp
├── blink_2.bmp
├── blink_3.bmp
├── blink_4.bmp
├── blink_5.bmp
├── blink_6.bmp
└── noseBar.bmp
```

## Execution

### Prerequisites
Before running the faceExpression module, ensure that:

1. **YARP Server is running**:
   ```bash
   yarp server
   ```

2. **Audio Services (optional)**: If using audio-reactive features:
   - Audio recorder for ear visualization
   - Audio player for mouth synchronization

3. **Image Assets**: Ensure all required image files are available in the images directory

### Running the Module

1. Make sure YARP server is running and image assets are available

2. Run the executable:
```bash
./faceExpressionImage5GTour
```

### Running with Configuration File
Create a configuration file (e.g., `faceExpressionImage.ini`):
```ini
# Image assets path (optional, auto-detected if not specified)
path /path/to/images

# Ear bar configuration
earBar0_x 1
earBar0_y 6
earBar0_minLen 3
earBar0_maxLen 18
earBar1_x 3
earBar1_y 6
earBar1_minLen 4
earBar1_maxLen 19

# Default context and config
robot r1
```

Then run:
```bash
./faceExpressionImage5GTour --from faceExpressionImage.ini
```

### YARP Port Information
The module creates the following YARP ports:

#### Output Ports
- `/faceExpressionImage/image:o`: RGB image output (80x32 pixels)
- `/faceExpressionImage/rpc`: RPC server for commands

#### Input Ports (optional)
- `/faceExpressionImage/earsAudioData:i`: Audio data for ear visualization
- `/faceExpressionImage/earsAudioStatus:i`: Audio recorder status
- `/faceExpressionImage/mouthAudioData:i`: Audio player status for mouth sync

## RPC Commands

The module supports both string-based and vocabulary-based RPC commands:

### String Commands
```bash
# Basic control
yarp rpc /faceExpressionImage/rpc
>> help                    # Show available commands
>> start_blinking          # Enable eye blinking
>> stop_blinking          # Disable eye blinking
>> start_talking          # Enable mouth animation
>> stop_talking           # Disable mouth animation
>> start_listening        # Enable ear bars
>> stop_listening         # Disable ear bars
>> reset_default          # Reset to default expression
>> black                  # Clear to black screen
```

### Vocabulary Commands
```bash
# Audio control
yarp write ... /faceExpressionImage/rpc
[asta]      # Start audio visualization (ears)
[asto]      # Stop audio visualization
[tsta]      # Start talking animation (mouth)
[tsto]      # Stop talking animation
[blin]      # Trigger blink
[rst]       # Reset to default
[blck]      # Black reset
```

## How It Works

### Multi-threaded Architecture
1. **DrawingThread**: Manages image output at 30 FPS
2. **EyesThread**: Handles blinking animation at 50 FPS
3. **EarsThread**: Processes audio data and draws bars at 50 FPS
4. **MouthThread**: Manages mouth animation based on audio at 50 FPS

### Rendering Pipeline
1. Shared 80x32 RGB image buffer (thread-safe with mutex)
2. Each feature thread updates its portion of the image
3. Drawing thread publishes the complete image via YARP
4. Real-time composition of all facial features

### Animation Systems
- **Eyes**: State machine with 11 animation frames and configurable delays
- **Ears**: Real-time audio level visualization with configurable bar lengths
- **Mouth**: Dynamic mouth movements during speech output
- **Synchronization**: Mutex-protected access to shared image buffer

## Integration with R1 Robot

This module is specifically designed for the R1 robot's face display system:

- **Display Resolution**: Matches R1's 80x32 pixel face display
- **Color Format**: RGB format compatible with R1's display hardware
- **Real-time Performance**: Optimized for smooth animation on embedded systems
- **Audio Integration**: Synchronized with R1's speech and audio systems

## Dependencies on Other Modules

This module can integrate with:
- **Audio Recording System**: For ear bar visualization
- **Text-to-Speech System**: For mouth animation synchronization
- **Display Hardware Interface**: For rendering on R1's face display
- **Robot Control System**: For coordinated facial expressions during interaction

## Troubleshooting

### Common Issues

1. **Build Errors**:
   - Ensure `ENABLE_faceExpressionImage=ON` is set during cmake configuration
   - Verify OpenCV is properly installed: `pkg-config --modversion opencv`
   - Check YARP installation: `yarp version`

2. **Missing Image Assets**:
   - Verify all .bmp files are present in the images directory
   - Check file permissions and formats
   - Use `--path /custom/path` to specify custom image directory

3. **YARP Connection Issues**:
   - Ensure `yarp server` is running
   - Check port conflicts: `yarp name list`
   - Verify network connectivity

4. **Performance Issues**:
   - Monitor CPU usage - multiple threads can be resource intensive
   - Adjust thread periods if needed
   - Check for mutex contention in system logs

5. **Audio Integration Problems**:
   - Verify audio ports are properly connected
   - Check audio device permissions
   - Test audio system independently

### Debug Mode
Enable YARP verbose logging:
```bash
export YARP_VERBOSE=1
./faceExpressionImage5GTour
```

### YARP Network Diagnostics
```bash
# Check YARP server status
yarp detect --write

# List all active ports
yarp name list

# Monitor image output
yarp read ... /faceExpressionImage/image:o

# Test RPC interface
yarp rpc /faceExpressionImage/rpc
```

## Performance Considerations

- **Memory Usage**: ~7.5KB for main image buffer plus asset storage
- **CPU Usage**: 4 concurrent threads with high-frequency updates
- **Network Bandwidth**: ~92KB/s for 30 FPS RGB image streaming
- **Real-time Constraints**: Designed for embedded robot applications
- **Thread Priority**: Consider setting appropriate thread priorities for real-time performance

## Customization

### Creating Custom Expressions
1. Modify image assets in the images directory
2. Adjust timing parameters via configuration file
3. Extend RPC commands for new expressions
4. Add custom color schemes and animation patterns

### Integration Examples
```cpp
// Example: Trigger expression via RPC
yarp::os::Bottle cmd, reply;
cmd.addString("start_blinking");
rpcPort.write(cmd, reply);

// Example: Custom color via vocabulary
yarp::os::Bottle cmd;
cmd.addVocab(VOCAB_BLINK);
rpcPort.write(cmd);
```