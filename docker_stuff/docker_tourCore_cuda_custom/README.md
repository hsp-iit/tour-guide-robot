# R1 Tour Guide Robot Core with Custom CUDA Docker Environment

This Docker environment provides a specialized build system for the R1 Tour Guide Robot Core environment with custom CUDA Toolkit versions. This image allows you to install specific CUDA versions on top of existing tour guide robot images, enabling GPU-accelerated robotics applications with precise CUDA version control for compatibility requirements.

## Docker Image Contents

The Docker image extends existing R1 Tour Guide Robot Core images with:

- **Custom CUDA Toolkit**: Automated installation of specific CUDA versions (default 12.1)
- **Multi-Ubuntu Support**: Automatically searches across Ubuntu 24.04, 22.04, and 20.04 repositories
- **CUDA Version Flexibility**: Supports major.minor version specification with automatic latest patch selection
- **Base Image Extension**: Builds on top of existing `elandini84/r1images:tourCore2_*` images
- **GPU Libraries**: Complete CUDA development environment with proper environment variables
- **Compatibility Layer**: Includes legacy library support (libtinfo5) for older CUDA applications

## Prerequisites

- Docker installed
- NVIDIA Docker runtime (required for GPU support)
- X11 forwarding capability
- Existing R1 Tour Guide Robot Core base image

## Building the Docker Image

### Using the Management Script (Recommended)

The `manage-docker.sh` script provides a convenient interface for building and running custom CUDA containers:

```bash
# Make the script executable
chmod +x manage-docker.sh

# Build with default CUDA version (12.1)
./manage-docker.sh --build --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable"

# Build with specific CUDA version
./manage-docker.sh --build --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" --cuda-version "12.0"

# Build with different base image and CUDA version
./manage-docker.sh --build --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_devel" --cuda-version "11.8"
```

### Using Docker Build Directly

```bash
# Build with default CUDA version
docker build \
  --build-arg base_img="elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" \
  --build-arg cuda_version="12.1" \
  -t r1-tourcore-cuda:custom .

# Build with specific CUDA version
docker build \
  --build-arg base_img="elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" \
  --build-arg cuda_version="11.8" \
  -t r1-tourcore-cuda11.8:custom .
```

### Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `base_img` | - | Base R1 Tour Guide Robot Core image to extend |
| `cuda_version` | `12.1` | CUDA Toolkit version (major.minor format) |

## Build and Run Options

### Management Script Options

#### Base Image Selection
```bash
# Use specific base image
./manage-docker.sh --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" [other-options]

# Different base image variations
./manage-docker.sh --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_devel" [other-options]
./manage-docker.sh --base-image "elandini84/r1images:tourCore2_cuda.12.8.1-cudnn_jazzy_stable" [other-options]
```

#### CUDA Version Selection
```bash
# CUDA 12.x versions
./manage-docker.sh --cuda-version "12.0" [other-options]
./manage-docker.sh --cuda-version "12.1" [other-options]
./manage-docker.sh --cuda-version "12.2" [other-options]

# CUDA 11.x versions
./manage-docker.sh --cuda-version "11.8" [other-options]
./manage-docker.sh --cuda-version "11.7" [other-options]
```

#### Build Type
```bash
# Development build
./manage-docker.sh --devel [other-options]

# Stable build
./manage-docker.sh --stable [other-options]
```

#### Repository Configuration
```bash
# Use custom repository
./manage-docker.sh --repo "myrepo/r1images" [other-options]
```

#### GPU Support
```bash
# Run without GPU (not recommended for this image)
./manage-docker.sh --nogpu [other-options]
```

### Running the Container

#### Using the Management Script

```bash
# Run with GPU support (recommended)
./manage-docker.sh --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" --cuda-version "12.1"

# Run without GPU support (limited functionality)
./manage-docker.sh --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" --cuda-version "12.1" --nogpu
```

#### Manual Docker Run

```bash
# With GPU support (recommended)
sudo docker run --rm -it --privileged --network host --pid host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e QT_X11_NO_MITSHM=1 --gpus all \
  elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable_cuda12.1

# Without GPU support
sudo docker run --rm -it --privileged --network host --pid host \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e QT_X11_NO_MITSHM=1 \
  elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable_cuda12.1
```

## Usage Examples

### Basic CUDA Development
```bash
# Build container with specific CUDA version
./manage-docker.sh --build --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" --cuda-version "12.0"

# Run the built container
./manage-docker.sh --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" --cuda-version "12.0"

# Inside container, verify CUDA installation
nvcc --version
nvidia-smi
```

### GPU-Accelerated Robotics Applications
```bash
# Build with CUDA 11.8 for specific library compatibility
./manage-docker.sh --build --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_devel" --cuda-version "11.8"

# Run and test GPU applications
./manage-docker.sh --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_devel" --cuda-version "11.8"

# Inside container, run GPU-enabled tour guide applications
cd /usr/local/src/robot/tour-guide-robot
# ... run CUDA-accelerated vision or AI components
```

### Version Compatibility Testing
```bash
# Test different CUDA versions for compatibility
./manage-docker.sh --build --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" --cuda-version "12.2"
./manage-docker.sh --build --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" --cuda-version "11.7"

# Compare performance or compatibility between versions
./manage-docker.sh --base-image "elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable" --cuda-version "12.2"
# ... test your applications
```

## Environment Variables

Additional environment variables set for CUDA:

```bash
PATH=/usr/local/cuda-${cuda_version}/bin:${PATH}              # CUDA binaries
LD_LIBRARY_PATH=/usr/local/cuda-${cuda_version}/lib64:${LD_LIBRARY_PATH}  # CUDA libraries
CUDA_VERSION=${cuda_version}                                  # Installed CUDA version
```

All base image environment variables are preserved from the underlying R1 Tour Guide Robot Core image.

## CUDA Version Support

### Automatic Version Resolution
The build system automatically:
1. Searches Ubuntu 24.04, 22.04, and 20.04 repositories
2. Finds the latest patch version for the specified major.minor version
3. Downloads and installs the appropriate CUDA toolkit
4. Configures environment variables for the installed version

### Supported CUDA Versions
- **CUDA 12.x**: 12.0, 12.1, 12.2, 12.3, 12.4, 12.5, 12.6
- **CUDA 11.x**: 11.0, 11.1, 11.2, 11.3, 11.4, 11.5, 11.6, 11.7, 11.8
- **Older versions**: May be available depending on Ubuntu repository support

## Volume Mounts

Same as base R1 Tour Guide Robot Core image:

- **X11 Display**: `/tmp/.X11-unix:/tmp/.X11-unix:rw`
- **Additional mounts**: As required by your GPU applications

## Troubleshooting

### CUDA Installation Issues
- Check CUDA version availability: The build will fail if the specified version is not found
- Try different CUDA versions if one is not available
- Check build logs for specific Ubuntu repository search results

### GPU Issues
- Ensure NVIDIA Docker runtime is installed: `sudo apt-get install nvidia-docker2`
- Verify GPU access: `nvidia-smi` on host
- Check CUDA installation inside container: `nvcc --version`

### Driver Compatibility
- Ensure host NVIDIA drivers support the requested CUDA version
- Check CUDA-driver compatibility matrix on NVIDIA documentation
- Use `nvidia-smi` to check driver version and supported CUDA versions

### Library Conflicts
- The image includes libtinfo5 for legacy compatibility
- If you encounter library issues, try different CUDA versions
- Check application requirements for specific CUDA version dependencies

### Performance Issues
- Ensure container is run with `--gpus all` flag
- Verify GPU utilization with `nvidia-smi` inside container
- Check for proper CUDA library linking in your applications

## Default Configuration

- **Base Image**: Must be specified (no default)
- **CUDA Version**: 12.1 (if not specified)
- **User**: Inherited from base image (`user1`)
- **Working Directory**: Inherited from base image
- **Network**: Host mode for seamless integration

## Image Naming Convention

Built images use the naming pattern:
```
${BASE_IMAGE_NAME}_cuda${CUDA_VERSION}
```

Examples:
- `elandini84/r1images:tourCore2_ubuntu24.04_jazzy_stable_cuda12.1`
- `elandini84/r1images:tourCore2_ubuntu24.04_jazzy_devel_cuda11.8`

This specialized Docker environment enables precise CUDA version control for GPU-accelerated robotics development while maintaining all the capabilities of the underlying R1 Tour Guide Robot Core system.