# Speech Processing Messages

## Overview

The Speech Processing Messages module provides YARP IDL (Interface Definition Language) message interfaces for communication between speech processing components in the tour-guide robot system. This module defines RPC (Remote Procedure Call) interfaces using Apache Thrift that enable standardized communication protocols between wake word detection, voice activity detection, and other speech processing modules.

## Key Features

- **Standardized RPC Interfaces**: Thrift-based IDL definitions for consistent inter-module communication
- **Wake Word Control**: RPC interface for controlling wake word detection behavior
- **VAD Configuration**: Interface for runtime configuration of Voice Activity Detection parameters
- **YARP Integration**: Seamless integration with YARP middleware and communication framework
- **Static Library Generation**: Automated generation of C++ client/server stubs from IDL definitions
- **Type Safety**: Strongly typed interfaces ensuring communication reliability
- **Cross-Platform**: Platform-independent message definitions compatible with YARP deployment targets

## Architecture

### Message Interfaces

The module contains two main message interface definitions:

#### 1. WakeMsgs (`wakeWordMsgs/`)
- **Purpose**: Control interface for wake word detection modules
- **Services**: Start/stop wake word streaming functionality
- **Usage**: Allows external modules to control wake word detection state

#### 2. SileroVADMsgs (`sileroVADMsgs/`)
- **Purpose**: Configuration interface for Silero Voice Activity Detection
- **Services**: Runtime adjustment of VAD parameters
- **Usage**: Dynamic tuning of voice detection sensitivity and gap handling

### Generated Components

Each message interface generates:
- **C++ Header Files**: Interface declarations and data structures
- **C++ Source Files**: Implementation stubs for client/server communication
- **YARP Integration**: Automatic port binding and message serialization
- **Static Libraries**: Linkable libraries for integration with other modules

## Technical Specifications

- **IDL Language**: Apache Thrift syntax for interface definitions
- **Build System**: CMake with YARP IDL tools integration
- **Target Languages**: C++ (primary), with YARP bindings for other languages
- **Communication**: YARP RPC ports with automatic serialization
- **Library Type**: Static libraries for efficient linking
- **CMake Version**: Requires CMake 3.16+ for modern IDL processing

## Dependencies

### System Dependencies
- **CMake** (>= 3.16): Build system with YARP IDL tools support
- **Apache Thrift**: IDL compiler and runtime libraries
- **C++ Compiler**: Supporting C++11 standard with STL
- **YARP** (Yet Another Robot Platform): Robot middleware and IDL tools
  - Components: `os` (operating system abstraction), `idl_tools` (IDL processing)

### Ubuntu/Debian Installation

#### Basic Dependencies
```bash
sudo apt-get update
sudo apt-get install cmake build-essential git pkg-config
```

#### Apache Thrift Installation
```bash
# Install Thrift compiler and development libraries
sudo apt-get install thrift-compiler libthrift-dev

# Alternative: Install from source for latest version
wget https://archive.apache.org/dist/thrift/0.17.0/thrift-0.17.0.tar.gz
tar -xzf thrift-0.17.0.tar.gz
cd thrift-0.17.0
./configure --prefix=/usr/local
make -j$(nproc)
sudo make install
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

# Install YARP from conda-forge (includes IDL tools)
conda install -c conda-forge -c robotology yarp
```

**Option 2: From Package Manager**
```bash
# Install YARP dependencies
sudo apt-get install libace-dev libeigen3-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5opengl5-dev

# Install YARP with IDL tools (if available in repositories)
sudo apt-get install libyarp-dev yarp yarp-idl-tools
```

**Option 3: Build from Source**
```bash
# Install YARP dependencies
sudo apt-get install libace-dev libeigen3-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5opengl5-dev

# Clone and build YARP with IDL support
git clone https://github.com/robotology/yarp.git
cd yarp
mkdir build && cd build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DYARP_COMPILE_IDLS=ON \
  -DYARP_COMPILE_BINDINGS=ON
make -j$(nproc)
sudo make install

# Set environment variables
echo 'export YARP_ROOT=/usr/local' >> ~/.bashrc
echo 'export PATH=$PATH:$YARP_ROOT/bin' >> ~/.bashrc
source ~/.bashrc
```

### Fedora/CentOS Installation

#### Basic Dependencies
```bash
sudo yum install cmake gcc-c++ git pkgconfig
```

#### Apache Thrift Installation
```bash
# Install Thrift (may need EPEL repository)
sudo yum install epel-release
sudo yum install thrift thrift-devel

# Or build from source
wget https://archive.apache.org/dist/thrift/0.17.0/thrift-0.17.0.tar.gz
tar -xzf thrift-0.17.0.tar.gz
cd thrift-0.17.0
./configure --prefix=/usr/local
make -j$(nproc)
sudo make install
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
cmake .. \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DYARP_COMPILE_IDLS=ON \
  -DYARP_COMPILE_BINDINGS=ON
make -j$(nproc)
sudo make install

# Set environment variables
echo 'export YARP_ROOT=/usr/local' >> ~/.bashrc
echo 'export PATH=$PATH:$YARP_ROOT/bin' >> ~/.bashrc
source ~/.bashrc
```

## Message Interface Definitions

### WakeMsgs Interface

**File**: `wakeWordMsgs/WakeMsgs.thrift`

```thrift
service WakeMsgs
{
  void stop();
}
```

**Generated Methods**:
- `stop()`: Stops wake word audio streaming and returns to monitoring mode

**Usage Example**:
```cpp
#include "WakeMsgs.h"

// Client usage
yarp::os::RpcClient client;
client.open("/client/rpc:o");
yarp::Network::connect("/client/rpc:o", "/wake/rpc:i");

WakeMsgs wake_client;
wake_client.yarp().attachAsClient(client);

// Stop wake word streaming
wake_client.stop();
```

### SileroVADMsgs Interface

**File**: `sileroVADMsgs/SileroVADMsgs.thrift`

```thrift
service SileroVADMsgs
{
  void setThreshold(1:double threshold);
  void setGapAllowance(1:i32 gapAllowance);
}
```

**Generated Methods**:
- `setThreshold(double threshold)`: Adjusts VAD detection threshold (0.0-1.0)
- `setGapAllowance(int gapAllowance)`: Sets allowed silence gap in milliseconds

**Usage Example**:
```cpp
#include "SileroVADMsgs.h"

// Client usage
yarp::os::RpcClient client;
client.open("/client/vad:o");
yarp::Network::connect("/client/vad:o", "/vad/rpc:i");

SileroVADMsgs vad_client;
vad_client.yarp().attachAsClient(client);

// Configure VAD parameters
vad_client.setThreshold(0.7);        // Set detection threshold
vad_client.setGapAllowance(300);     // Allow 300ms gaps
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

4. Build the message libraries:
```bash
make -j$(nproc)
```

The message libraries will be built automatically as dependencies of other modules.

### Building Standalone

If you want to build only the message interfaces:

1. Navigate to the messages directory:
```bash
cd /path/to/tour-guide-robot/aux_modules/speechProcessing/messages
```

2. Create and enter build directory:
```bash
mkdir build && cd build
```

3. Configure and build:
```bash
cmake ..
make -j$(nproc)
```

4. Install the libraries (optional):
```bash
make install
```

### Build Output

Each message interface generates:
- **Static Library**: `lib<MessageName>.a` (e.g., `libWakeMsgs.a`)
- **Header Files**: Interface declarations in build directory
- **CMake Targets**: Importable targets for linking

## Integration with Other Modules

### CMakeLists.txt Integration

To use these message interfaces in other modules:

```cmake
# Find the message libraries
find_package(WakeMsgs REQUIRED)
find_package(SileroVADMsgs REQUIRED)

# Add to your target
target_link_libraries(your_module
  PRIVATE
    WakeMsgs
    SileroVADMsgs
    ${YARP_LIBRARIES}
)

# Include directories (handled automatically by modern CMake)
target_include_directories(your_module
  PRIVATE
    $<TARGET_PROPERTY:WakeMsgs,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:SileroVADMsgs,INTERFACE_INCLUDE_DIRECTORIES>
)
```

### Server Implementation Example

```cpp
#include "WakeMsgs.h"
#include <yarp/os/RpcServer.h>

class WakeWordServer : public WakeMsgs {
public:
    void stop() override {
        // Implementation: stop audio streaming
        m_streaming = false;
        yInfo() << "Wake word streaming stopped";
    }

private:
    bool m_streaming = false;
};

// Usage in main module
WakeWordServer server;
yarp::os::RpcServer rpc_port;
rpc_port.open("/wake/rpc:i");
server.yarp().attachAsServer(rpc_port);
```

### Client Implementation Example

```cpp
#include "SileroVADMsgs.h"
#include <yarp/os/RpcClient.h>

class VADController {
public:
    bool configure(const std::string& server_port) {
        m_client.open("/vad_controller/rpc:o");
        if (!yarp::Network::connect("/vad_controller/rpc:o", server_port)) {
            return false;
        }
        m_vad_msgs.yarp().attachAsClient(m_client);
        return true;
    }

    void adjustSensitivity(double threshold) {
        m_vad_msgs.setThreshold(threshold);
    }

    void setGapTolerance(int milliseconds) {
        m_vad_msgs.setGapAllowance(milliseconds);
    }

private:
    yarp::os::RpcClient m_client;
    SileroVADMsgs m_vad_msgs;
};
```

## Usage Examples

### Complete Wake Word Control Example

```cpp
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include "WakeMsgs.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        std::cerr << "YARP network not available" << std::endl;
        return -1;
    }

    // Connect to wake word detection module
    yarp::os::RpcClient client;
    client.open("/wake_controller:o");

    if (!yarp::Network::connect("/wake_controller:o", "/wake/rpc:i")) {
        std::cerr << "Cannot connect to wake word module" << std::endl;
        return -1;
    }

    WakeMsgs wake_control;
    wake_control.yarp().attachAsClient(client);

    // Wait for some time, then stop streaming
    std::cout << "Waiting 10 seconds before stopping wake word streaming..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "Stopping wake word streaming" << std::endl;
    wake_control.stop();

    client.close();
    return 0;
}
```

### VAD Parameter Tuning Example

```cpp
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include "SileroVADMsgs.h"
#include <iostream>

class VADTuner {
public:
    bool initialize(const std::string& vad_port) {
        if (!m_client.open("/vad_tuner:o")) {
            return false;
        }

        if (!yarp::Network::connect("/vad_tuner:o", vad_port)) {
            m_client.close();
            return false;
        }

        m_vad.yarp().attachAsClient(m_client);
        return true;
    }

    void tuneForEnvironment(const std::string& environment) {
        if (environment == "quiet") {
            m_vad.setThreshold(0.3);      // More sensitive
            m_vad.setGapAllowance(100);   // Shorter gaps
        } else if (environment == "noisy") {
            m_vad.setThreshold(0.8);      // Less sensitive
            m_vad.setGapAllowance(500);   // Longer gaps
        } else { // balanced
            m_vad.setThreshold(0.6);      // Default sensitivity
            m_vad.setGapAllowance(300);   // Default gaps
        }

        std::cout << "VAD tuned for " << environment << " environment" << std::endl;
    }

    ~VADTuner() {
        m_client.close();
    }

private:
    yarp::os::RpcClient m_client;
    SileroVADMsgs m_vad;
};

int main() {
    yarp::os::Network yarp;

    VADTuner tuner;
    if (!tuner.initialize("/vad/rpc:i")) {
        std::cerr << "Failed to connect to VAD module" << std::endl;
        return -1;
    }

    // Tune for different environments
    tuner.tuneForEnvironment("quiet");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    tuner.tuneForEnvironment("noisy");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    tuner.tuneForEnvironment("balanced");

    return 0;
}
```

## YARP IDL Code Generation

### Understanding the Build Process

The CMake configuration automatically generates C++ code from Thrift IDL files:

```cmake
yarp_idl_to_dir(
  INPUT_FILES SileroVADMsgs.thrift
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/SileroVADMsgs
  SOURCES_VAR IDL_GEN_SRCS
  HEADERS_VAR IDL_GEN_HDRS
  INCLUDE_DIRS_VAR IDL_INCLUDE_DIRS
  PLACEMENT MERGED
)
```

### Generated Files Structure

For each `.thrift` file, YARP generates:
```
build/
├── <MessageName>/
│   ├── <MessageName>.h          # Main interface header
│   ├── <MessageName>_common.h   # Common definitions
│   └── <MessageName>.cpp        # Implementation
└── lib<MessageName>.a           # Static library
```

### Manual Code Generation (Development)

For development and debugging, you can manually generate code:

```bash
# Generate code for WakeMsgs
yarp_idl_to_dir --input WakeMsgs.thrift --output-dir ./generated

# Generate code for SileroVADMsgs
yarp_idl_to_dir --input SileroVADMsgs.thrift --output-dir ./generated
```

## Advanced Usage

### Custom Message Types

To add new message interfaces:

1. **Create Thrift File**:
```thrift
// CustomMsgs.thrift
service CustomMsgs
{
  bool processAudio(1:string audio_data);
  void setParameter(1:string key, 2:string value);
}
```

2. **Create CMakeLists.txt**:
```cmake
cmake_minimum_required(VERSION 3.16)
project(custom_msgs)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(YARP COMPONENTS os idl_tools REQUIRED)

add_library(CustomMsgs STATIC)

yarp_idl_to_dir(
  INPUT_FILES CustomMsgs.thrift
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/CustomMsgs
  SOURCES_VAR IDL_GEN_SRCS
  HEADERS_VAR IDL_GEN_HDRS
  INCLUDE_DIRS_VAR IDL_INCLUDE_DIRS
  PLACEMENT MERGED
)

target_sources(CustomMsgs
  PRIVATE ${IDL_GEN_SRCS} ${IDL_GEN_HDRS}
)

target_include_directories(CustomMsgs
  PUBLIC $<BUILD_INTERFACE:${IDL_INCLUDE_DIRS}>
)

target_link_libraries(CustomMsgs
  PRIVATE YARP::YARP_os
)
```

3. **Add to Parent CMakeLists.txt**:
```cmake
add_subdirectory(customMsgs)
```

### Complex Data Types

Thrift supports complex data structures:

```thrift
struct AudioSegment {
  1: list<double> samples;
  2: i32 sample_rate;
  3: i64 timestamp;
}

struct DetectionResult {
  1: bool voice_detected;
  2: double confidence;
  3: AudioSegment segment;
}

service AdvancedVAD {
  DetectionResult processAudio(1: AudioSegment audio);
  void configureDetection(1: map<string, string> parameters);
}
```

### Asynchronous Communication

For non-blocking communication:

```cpp
#include <yarp/os/RpcClient.h>
#include <future>

class AsyncVADController {
public:
    std::future<void> setThresholdAsync(double threshold) {
        return std::async(std::launch::async, [this, threshold]() {
            m_vad.setThreshold(threshold);
        });
    }

private:
    SileroVADMsgs m_vad;
};
```

## Troubleshooting

### Common Issues

1. **IDL Generation Fails**:
```bash
# Check YARP IDL tools installation
yarp_idl_to_dir --help

# Verify Thrift syntax
thrift --gen cpp YourMessages.thrift

# Check CMake YARP components
cmake .. -DYARP_DIR=/path/to/yarp/build
```

2. **Linking Errors**:
```bash
# Ensure static libraries are found
find /usr/local -name "libWakeMsgs.a" 2>/dev/null

# Check library dependencies
ldd your_executable

# Verify CMake target linking
cmake --build . --target your_module --verbose
```

3. **Header Not Found**:
```bash
# Check include directories
find build/ -name "*.h" | grep -E "(Wake|VAD)"

# Verify CMake include paths
cmake .. -DCMAKE_VERBOSE_MAKEFILE=ON
```

4. **RPC Connection Issues**:
```bash
# Check YARP network
yarp detect

# List active ports
yarp name list

# Test RPC connection manually
yarp rpc /target/port/name
```

5. **Build Configuration Problems**:
```bash
# Clean and reconfigure
rm -rf CMakeCache.txt CMakeFiles/
cmake ..

# Check YARP components
cmake .. -DYARP_COMPILE_IDLS=ON

# Verify generator availability
which yarp_idl_to_dir
```

### Debug Mode

Enable verbose IDL generation:

```cmake
set(YARP_IDL_VERBOSE ON)
yarp_idl_to_dir(
  INPUT_FILES YourMessages.thrift
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/YourMessages
  SOURCES_VAR IDL_GEN_SRCS
  HEADERS_VAR IDL_GEN_HDRS
  INCLUDE_DIRS_VAR IDL_INCLUDE_DIRS
  PLACEMENT MERGED
  VERBOSE
)
```

### Testing Message Interfaces

Create simple test clients:

```cpp
// test_messages.cpp
#include "WakeMsgs.h"
#include "SileroVADMsgs.h"
#include <yarp/os/Network.h>
#include <iostream>

int main() {
    yarp::os::Network yarp;

    // Test WakeMsgs compilation
    std::cout << "WakeMsgs interface compiled successfully" << std::endl;

    // Test SileroVADMsgs compilation
    std::cout << "SileroVADMsgs interface compiled successfully" << std::endl;

    return 0;
}
```

## Integration Testing

### Complete Speech Pipeline Test

```bash
#!/bin/bash
# test_speech_messages.sh

# Start YARP server
yarpserver &
sleep 2

# Start mock servers for testing
./mock_wake_server &
./mock_vad_server &
sleep 2

# Test message interfaces
./test_wake_messages
./test_vad_messages

# Cleanup
pkill -f "mock_.*_server"
pkill yarpserver

echo "Message interface tests completed"
```

### Mock Server Implementation

```cpp
// mock_vad_server.cpp
#include "SileroVADMsgs.h"
#include <yarp/os/RpcServer.h>
#include <iostream>

class MockVADServer : public SileroVADMsgs {
public:
    void setThreshold(double threshold) override {
        std::cout << "Mock VAD: threshold set to " << threshold << std::endl;
        m_threshold = threshold;
    }

    void setGapAllowance(int gapAllowance) override {
        std::cout << "Mock VAD: gap allowance set to " << gapAllowance << "ms" << std::endl;
        m_gap_allowance = gapAllowance;
    }

private:
    double m_threshold = 0.6;
    int m_gap_allowance = 300;
};

int main() {
    yarp::os::Network yarp;

    MockVADServer server;
    yarp::os::RpcServer port;
    port.open("/mock_vad/rpc:i");
    server.yarp().attachAsServer(port);

    std::cout << "Mock VAD server running..." << std::endl;

    while (true) {
        yarp::os::Time::delay(1.0);
    }

    return 0;
}
```

## Performance Considerations

- **Static Linking**: Static libraries reduce runtime dependencies but increase binary size
- **Code Generation**: IDL compilation adds to build time but provides type safety
- **RPC Overhead**: YARP RPC has minimal overhead for local communication
- **Memory Usage**: Generated code is lightweight with minimal memory footprint
- **Network Efficiency**: YARP optimizes message serialization for network communication

## Extension Guidelines

### Adding New Interfaces

1. **Define Interface**: Create `.thrift` file with service definition
2. **Create Build**: Add CMakeLists.txt with YARP IDL integration
3. **Update Parent**: Add subdirectory to parent CMakeLists.txt
4. **Implement Server**: Create server class implementing interface
5. **Test Integration**: Verify with client/server test programs

### Best Practices

- **Naming Convention**: Use descriptive service and method names
- **Parameter Types**: Use appropriate Thrift types for data
- **Error Handling**: Consider error return types for fallible operations
- **Documentation**: Document IDL files with comments
- **Versioning**: Plan for interface evolution and backward compatibility

This comprehensive documentation provides complete guidance for building, using, and extending the speech processing message interfaces in the tour-guide robot system.