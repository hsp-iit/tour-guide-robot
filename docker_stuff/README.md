# R1 Tour Guide Robot Docker Environments

This directory contains multiple Docker environments for the R1 Tour Guide Robot system. Each environment is tailored for specific use cases and provides different capabilities for robotics development, simulation, and deployment.

## Available Docker Images

### 1. **R1 Tour Guide Robot Core** (`docker_tourCore/`)
Complete robotics development environment with full YARP and ROS2 integration.

**Key Features:**
- Complete robotics stack (YARP + ROS2)
- Navigation and SLAM capabilities
- Hardware interfaces (RPLidar, sensors)
- iCub and CER robot components
- Real robot deployment ready

**Use Cases:** Real robot development, hardware testing, complete robotics applications

📖 **[View Documentation](docker_tourCore/README.md)**

---

### 2. **R1 Tour Guide Robot Simulation** (`docker_sim/`)
Specialized environment for robot simulation with Gazebo integration.

**Key Features:**
- Gazebo Harmonic simulation environment
- YARP-Gazebo integration plugins
- Fake device plugins for simulation
- R1 robot models and simulation worlds
- GPU acceleration support

**Use Cases:** Robot simulation, algorithm testing, virtual environments

📖 **[View Documentation](docker_sim/README.md)**

---

### 3. **R1 Talk - Conversational AI** (`docker_talk/`)
Specialized environment for conversational AI and natural language processing.

**Key Features:**
- Google Cloud Speech services integration
- Azure OpenAI and LLM capabilities
- Wake word detection (Porcupine)
- Speech synthesis and dialogue management
- Natural language processing tools

**Use Cases:** Voice interaction, conversational AI, speech processing

📖 **[View Documentation](docker_talk/README.md)**

---

### 4. **R1 Core with Custom CUDA** (`docker_tourCore_cuda_custom/`)
Extension system for adding specific CUDA versions to existing tour guide images.

**Key Features:**
- Custom CUDA Toolkit installation
- Multi-Ubuntu repository support
- Flexible CUDA version selection
- GPU-accelerated robotics applications
- Base image extension approach

**Use Cases:** GPU acceleration, specific CUDA compatibility, custom builds

📖 **[View Documentation](docker_tourCore_cuda_custom/README.md)**

---

## Quick Start Guide

### Prerequisites
- Docker installed
- NVIDIA Docker runtime (for GPU support)
- X11 forwarding capability
- Required credential files (for specific environments)

### General Build Pattern
Each environment includes a management script for easy building and running:

```bash
# Navigate to the desired environment
cd docker_<environment_name>/

# Make the management script executable
chmod +x manage-docker.sh

# Build the image
./manage-docker.sh --build [options]

# Run the container
./manage-docker.sh [options]
```

### Environment Selection Guide

| **Use Case** | **Recommended Environment** | **Key Benefits** |
|--------------|---------------------------|------------------|
| Real robot development | `docker_tourCore` | Complete stack, hardware support |
| Robot simulation | `docker_sim` | Gazebo integration, virtual testing |
| Voice/AI features | `docker_talk` | Speech services, LLM integration |
| GPU development | `docker_tourCore_cuda_custom` | Custom CUDA versions |
| Learning/tutorials | `docker_sim` | Risk-free simulation environment |
| Hardware deployment | `docker_tourCore` | Real-world robot compatibility |

## Common Configuration

All environments share common configuration through `docker_mng_vars.sh`:

- **Ubuntu Version**: 24.04 (default)
- **ROS2 Distribution**: Jazzy (default)
- **YARP Version**: yarp-3.12 (default)
- **Repository**: elandini84/r1images

## Network Configuration

All Docker environments use:
- **Network Mode**: Host (for seamless YARP/ROS2 integration)
- **YARP Port**: 10000 (TCP/UDP)

## Support and Troubleshooting

Each environment includes comprehensive troubleshooting sections in their respective documentation. Common issues include:

- **Display Issues**: X11 forwarding and xhost configuration
- **GPU Issues**: NVIDIA Docker runtime and driver compatibility
- **Network Issues**: ROS Domain ID conflicts and DDS configuration
- **Credential Issues**: Service account and API key configuration

For environment-specific help, refer to the individual README files linked above.

## Contributing

When contributing to the Docker environments:
1. Follow the existing naming conventions
2. Update the management scripts accordingly
3. Include comprehensive documentation
4. Test both build and run scenarios
5. Update this summary when adding new environments