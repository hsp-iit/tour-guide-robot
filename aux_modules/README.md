# Auxiliary Modules

## Overview

The Auxiliary Modules provide specialized functionality for the tour-guide robot system, including human interaction management, speech processing, visual feedback, navigation assistance, and logging capabilities. These modules work together to create engaging and intelligent robot-human interactions during guided tours.

## Architecture

The auxiliary modules integrate with the core tour-guide robot system through YARP middleware, providing modular and configurable functionality:

```
Tour Manager ← → Speech Processing ← → Wake Word Detection
     ↓                  ↓                      ↓
Head Synchronizer ← Face Expression    Audio Processing
     ↓                  ↓                      ↓
Eye Contact Manager → Visual Feedback  → Crowd Detection
     ↓                  ↓                      ↓
Navigation Support ← Head Scanner    → Google Logger
```

## Core Modules

### 1. [Tour Manager](./tourManager/) - Central Orchestration
Main coordination system for guided tours, managing Points of Interest, speech, navigation, and multi-language support.

[📖 View Tour Manager Documentation](./tourManager/README.md)

### 2. [Head Synchronizer](./headSynchronizer/) - Audio-Visual Coordination
Central system for synchronized speech synthesis, audio playbook, microphone control, and facial expressions.

[📖 View Head Synchronizer Documentation](./headSynchronizer/README.md)

### 3. [Speech Processing](./speechProcessing/) - Voice Interaction Suite
Comprehensive speech processing pipeline including wake word detection, voice activity detection, and audio filtering.

[📖 View Speech Processing Documentation](./speechProcessing/README.md)

## Human Interaction Modules

### 4. [Eye Contact Manager](./eyeContactManager/) - Gaze Control
Manages eye contact behavior during navigation, tracking the closest person and controlling robot gaze when stationary.

[📖 View Eye Contact Manager Documentation](./eyeContactManager/README.md)

### 5. [Face Expression](./faceExpression/) - Visual Feedback *(OpenCV Required)*
Generates animated facial expressions with blinking eyes, audio-reactive ears, and talking mouth animations.

[📖 View Face Expression Documentation](./faceExpression/README.md)

### 6. [Crowd Detector](./crowdDetector/) - Human Tracking *(OpenCV Required)*
Detects and counts humans around Points of Interest using frame transformation and spatial filtering.

[📖 View Crowd Detector Documentation](./crowdDetector/README.md)

## Navigation Support Modules

### 7. [Head Obstacles Scanner](./headObstaclesScanner/) - Environmental Scanning *(OpenCV Required)*
Controls head movement for obstacle detection with sweep and trajectory modes, plus real-time map visualization.

[📖 View Head Obstacles Scanner Documentation](./headObstaclesScanner/README.md)

## Utility Modules

### 8. [Google Logger](./googleLogger/) - Interaction Logging
Logs user interactions with Google Dialog services, creating timestamped CSV files for tour analysis.

[📖 View Google Logger Documentation](./googleLogger/README.md)

### 9. [Text to Face](./textToFace/) - Audio Channel Filtering
YARP port monitor plugin for extracting specific audio channels from multi-channel microphone inputs.

[📖 View Text to Face Documentation](./textToFace/README.md)

## Quick Start

### Installation
```bash
# Install YARP (recommended: conda)
conda create -n robotics-env
conda activate robotics-env
conda install -c conda-forge -c robotology yarp

# For OpenCV-dependent modules
conda install -c conda-forge opencv
```

### Build
```bash
cd /path/to/tour-guide-robot
mkdir build && cd build

# Build all modules
cmake ..
make -j$(nproc)

# Build with OpenCV modules enabled
cmake .. -DOPENCV_ON=ON
make -j$(nproc)
```

### Basic Tour Setup
```bash
# Start YARP server
yarpserver &

# Start core modules
./tourManager --from tour_config.ini &
./headSynchronizer &

# Start interaction modules
./eyeContactManager &
./googleLogger &
```

## Build Configuration

Modules are conditionally compiled based on available dependencies:

### Always Built
- `headSynchronizer` - Core audio-visual coordination
- `googleLogger` - Interaction logging
- `tourManager` - Tour orchestration
- `eyeContactManager` - Gaze control
- `speechProcessing` - Voice processing suite

### Conditionally Built (OpenCV Required)
- `headObstaclesScanner` - Environmental scanning
- `faceExpression` - Animated facial expressions
- `crowdDetector` - Human detection and counting

### Build Options
```bash
# Enable OpenCV-dependent modules
cmake .. -DOPENCV_ON=ON

# Check OpenCV status
cmake .. -DOPENCV_ON=ON --verbose
```

## Integration Patterns

### Complete Tour System
```bash
# Start all core services
yarpserver &
./tourManager --from config.ini &
./headSynchronizer &

# Add human interaction
./eyeContactManager &
./faceExpression &
./crowdDetector &

# Add navigation support
./headObstaclesScanner &

# Add logging
./googleLogger &
```

### Speech-Focused Setup
```bash
# Speech processing pipeline
cd speechProcessing
./wakeWordDetection &
./sileroVAD &

# Coordinate with head synchronizer
./headSynchronizer &
```

### Navigation-Focused Setup
```bash
# Navigation support modules
./headObstaclesScanner &
./eyeContactManager &
./crowdDetector &
```

## Module Dependencies

### Core Dependencies (All Modules)
- **YARP**: Robot communication middleware
- **CMake** (≥ 3.12): Build system
- **C++11**: Standard language support

### OpenCV Modules
- **OpenCV**: Computer vision library
- **CMake** (≥ 3.12): With OpenCV support

### Specialized Dependencies
- **nlohmann/json** (≥ 3.10.5): Tour Manager configuration
- **Apache Thrift**: Speech Processing message interfaces
- **Python 3.11+**: OpenWakeWord module
- **ONNX Runtime**: Neural network inference

## Configuration Management

### Central Configuration
Most modules support configuration via INI files:

```ini
# tour_config.ini
[GENERAL]
name tourManager
period 1.0

[TOURS]
tour_file tours.json
movements_file movements.json
```

### Module-Specific Configuration
Each module provides detailed configuration options in their respective documentation.

## Troubleshooting

### Common Issues

1. **Build Failures**:
```bash
# Check OpenCV availability
cmake .. -DOPENCV_ON=ON --verbose

# Check YARP installation
yarp detect
```

2. **Module Connection Issues**:
```bash
# Check YARP network
yarp name list

# Test module ports
yarp info /moduleName/port
```

3. **OpenCV Module Issues**:
```bash
# Verify OpenCV installation
pkg-config --modversion opencv4

# Check build flags
cmake .. -DOPENCV_ON=ON -DCMAKE_VERBOSE_MAKEFILE=ON
```

### Module-Specific Support
- [Tour Manager troubleshooting](./tourManager/README.md#troubleshooting)
- [Speech Processing troubleshooting](./speechProcessing/README.md#support)
- [Face Expression troubleshooting](./faceExpression/README.md#troubleshooting)
- [Head Synchronizer troubleshooting](./headSynchronizer/README.md#troubleshooting)

## Development Guidelines

### Adding New Modules

1. **Create Module Structure**:
```bash
mkdir newModule
cd newModule
# Add CMakeLists.txt, source files, README.md
```

2. **Update CMakeLists.txt**:
```cmake
# In aux_modules/CMakeLists.txt
add_subdirectory(newModule)

# Or conditionally:
if(${DEPENDENCY_FOUND})
    add_subdirectory(newModule)
endif()
```

3. **Follow Documentation Standards**:
   - Create comprehensive README.md
   - Include installation instructions with conda/apt options
   - Add configuration parameters table
   - Provide usage examples

### Best Practices
- **YARP Integration**: Use standard port naming conventions
- **Configuration**: Support both INI files and command-line parameters
- **Error Handling**: Provide clear error messages and recovery
- **Threading**: Use YARP's threading models appropriately
- **Documentation**: Keep README files updated with features

## Performance Considerations

### Resource Management
- **Memory Usage**: Monitor for modules with image processing
- **CPU Usage**: Profile real-time modules like speech processing
- **Network Load**: Optimize YARP port communications

### System Integration
- **Module Coordination**: Avoid conflicts between head control modules
- **State Synchronization**: Use appropriate locking for shared resources
- **Service Dependencies**: Handle graceful startup/shutdown

## Support

For general issues:
1. Check individual module documentation
2. Verify YARP network connectivity
3. Test modules individually before integration
4. Review build configuration and dependencies

Each module provides detailed troubleshooting in its respective README file.