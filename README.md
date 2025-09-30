# Tour Guide Robot

[![Docker build](https://github.com/hsp-iit/tour-guide-robot/actions/workflows/main.yml/badge.svg)](https://github.com/hsp-iit/tour-guide-robot/actions/workflows/main.yml)

[![Docker build secondary](https://github.com/hsp-iit/tour-guide-robot/actions/workflows/secondary.yml/badge.svg)](https://github.com/hsp-iit/tour-guide-robot/actions/workflows/secondary.yml)

[![Docker build - Old](https://github.com/hsp-iit/tour-guide-robot/actions/workflows/old.yml/badge.svg)](https://github.com/hsp-iit/tour-guide-robot/actions/workflows/old.yml)

## Overview

The Tour Guide Robot is a comprehensive robotics system designed for autonomous guided tours in various environments. Built on the R1 humanoid robot platform, this system combines advanced navigation, natural language processing, computer vision, and human-robot interaction capabilities to create engaging and intelligent touring experiences.

The system integrates YARP (Yet Another Robot Platform) and ROS2 frameworks to provide a robust foundation for real-time robotics applications, featuring modular architecture that supports both physical robot deployment and simulation environments.

## Key Features

### 🤖 **Complete Robotics Stack**
- **Navigation & SLAM**: Advanced autonomous navigation with simultaneous localization and mapping
- **Hardware Integration**: Support for various sensors including RPLidar, cameras, and IMUs
- **Real-time Control**: Precise motor control and sensor fusion for smooth robot operation

### 🗣️ **Conversational AI & Speech Processing**
- **Natural Language Processing**: Integration with Google Cloud Speech services and Azure OpenAI
- **Wake Word Detection**: Voice activation using Porcupine wake word technology
- **Multi-language Support**: Tour content and interactions in multiple languages
- **Speech Synthesis**: High-quality text-to-speech for engaging tour narration

### 👁️ **Computer Vision & Human Interaction**
- **Facial Expression Control**: Animated expressions with synchronized audio-visual feedback
- **Eye Contact Management**: Intelligent gaze control for natural human interaction
- **Crowd Detection**: Real-time human detection and tracking around Points of Interest
- **Visual Feedback**: Dynamic visual responses to enhance user engagement

### 🌐 **Simulation & Development Environment**
- **Gazebo Integration**: Complete simulation environment with physics-based robot models
- **Docker Containerization**: Multiple specialized environments for different development needs
- **Cross-platform Support**: Consistent development experience across different systems

## Architecture

The system is built with a modular architecture consisting of:

- **Core Applications** (`app/`): Main robot control, navigation, and tour management applications
- **Auxiliary Modules** (`aux_modules/`): Specialized modules for human interaction, speech processing, and environmental sensing
- **Docker Environments** (`docker_stuff/`): Containerized development and deployment environments
- **Hardware Interfaces** (`interfaces/`): YARP and ROS2 communication protocols
- **Behavior Trees** (`skills/`): Modular robot behaviors and decision-making logic

## Quick Start

### Using Docker (Recommended)

The fastest way to get started is using our pre-built Docker environments:

```bash
# Clone the repository
git clone https://github.com/hsp-iit/tour-guide-robot.git
cd tour-guide-robot

# Choose your environment:
cd docker_stuff/docker_tourCore    # For complete robotics development
cd docker_stuff/docker_sim         # For simulation and testing
cd docker_stuff/docker_talk        # For conversational AI development

# Build and run
chmod +x manage-docker.sh
./manage-docker.sh --build --ubuntu --ros_distro jazzy --devel
./manage-docker.sh --ubuntu --ros_distro jazzy --devel
```

### Native Installation

For native development, install the required dependencies:

```bash
# Install YARP and ROS2
conda create -n robotics-env
conda activate robotics-env
conda install -c conda-forge -c robotology yarp
# Follow ROS2 Jazzy installation instructions

# Build the project
mkdir build && cd build
cmake .. -DOPENCV_ON=ON
make -j$(nproc)
```

## Available Docker Environments

We provide four specialized Docker environments for different use cases:

| Environment | Use Case | Key Features |
|-------------|----------|--------------|
| **[docker_tourCore](docker_stuff/docker_tourCore/)** | Real robot development | Complete YARP+ROS2 stack, hardware interfaces |
| **[docker_sim](docker_stuff/docker_sim/)** | Simulation & testing | Gazebo integration, virtual environments |
| **[docker_talk](docker_stuff/docker_talk/)** | Conversational AI | Speech services, LLM integration, wake word |
| **[docker_tourCore_cuda_custom](docker_stuff/docker_tourCore_cuda_custom/)** | GPU development | Custom CUDA versions, accelerated computing |

📖 **[View Complete Docker Documentation](docker_stuff/README.md)**

## Auxiliary Modules

The system includes specialized auxiliary modules for enhanced robot capabilities:

### Core Interaction Modules
- **[Tour Manager](aux_modules/tourManager/)**: Central orchestration for guided tours with Points of Interest management
- **[Head Synchronizer](aux_modules/headSynchronizer/)**: Audio-visual coordination for speech and facial expressions
- **[Speech Processing](aux_modules/speechProcessing/)**: Complete voice interaction pipeline with wake word detection

### Human Interaction Enhancement
- **[Eye Contact Manager](aux_modules/eyeContactManager/)**: Intelligent gaze control during navigation and interaction
- **[Face Expression](aux_modules/faceExpression/)**: Animated facial expressions with real-time audio synchronization
- **[Crowd Detector](aux_modules/crowdDetector/)**: Human detection and tracking around tour locations

### Navigation & Environment
- **[Head Obstacles Scanner](aux_modules/headObstaclesScanner/)**: Environmental scanning with head movement control
- **[Google Logger](aux_modules/googleLogger/)**: Interaction logging and analytics

📖 **[View Complete Auxiliary Modules Documentation](aux_modules/README.md)**

## System Requirements

### Minimum Requirements
- **OS**: Ubuntu 20.04+ or Docker support
- **Memory**: 8GB RAM
- **Storage**: 20GB available space
- **Network**: Internet connection for cloud services

### Recommended for Full Features
- **OS**: Ubuntu 24.04
- **Memory**: 16GB+ RAM
- **GPU**: NVIDIA GPU with CUDA support (for vision modules)
- **Network**: Stable internet for speech and AI services

### Hardware Robot Requirements
- R1 humanoid robot platform
- RPLidar sensor
- RGB-D cameras
- Microphone array
- Speaker system

## Configuration

The system supports flexible configuration through:

- **INI Configuration Files**: Module-specific settings and parameters
- **JSON Tour Definitions**: Points of Interest, routes, and tour content
- **Docker Environment Variables**: Container-specific configuration
- **CMake Build Options**: Feature selection and dependency management

Example basic tour configuration:
```ini
[GENERAL]
name tourManager
period 1.0
robot_name r1

[TOURS]
tour_file tours.json
movements_file movements.json
language en-US
```

## Development

### Building from Source
```bash
# Standard build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# With OpenCV modules (vision features)
cmake .. -DCMAKE_BUILD_TYPE=Release -DOPENCV_ON=ON
make -j$(nproc)

# With additional features
cmake .. -DENABLE_faceExpressionImage=ON -DSILERO_VAD=ON
make -j$(nproc)
```

### Module Development
The modular architecture allows for easy extension:

1. **Add new auxiliary modules** in `aux_modules/`
2. **Extend behavior trees** in `skills/`
3. **Create custom applications** in `app/`
4. **Define new interfaces** in `interfaces/`

## Deployment

### Physical Robot Deployment
```bash
# Start core services
yarpserver &
./tourManager --from config.ini &
./headSynchronizer &

# Start interaction modules
./eyeContactManager &
./faceExpression &
./speechProcessing/wakeWordDetection &
```

### Simulation Environment
```bash
# Launch Gazebo simulation
gz sim world.sdf &

# Start robot applications
./tourManager --simulation &
yarpmanager
```

## Support & Documentation

- **[Wiki](https://github.com/hsp-iit/tour-guide-robot/wiki)**: Comprehensive documentation and tutorials
- **[Docker Environments](docker_stuff/README.md)**: Container setup and usage
- **[Auxiliary Modules](aux_modules/README.md)**: Module-specific documentation
- **Issues**: Report bugs and request features via GitHub Issues

## Applications Note

Some applications don't behave nicely on yarpmanager (ex gazebo due to clock), so they are run separately with a .sh file.

## Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch
3. Follow existing code style and documentation standards
4. Test your changes thoroughly
5. Submit a pull request

## License

This project is developed by the Human-Robot Interaction Lab at the Italian Institute of Technology (IIT). Please refer to the repository for licensing information.
