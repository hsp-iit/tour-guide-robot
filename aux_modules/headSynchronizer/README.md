# Head Synchronizer Module

## Overview

The Head Synchronizer module is a central coordination system that manages synchronized audio-visual interactions for tour guide robots. It orchestrates speech synthesis, audio playback, microphone control, and facial expressions to create natural human-robot interactions. The module acts as a middleware between various subsystems, ensuring proper timing and state management during conversations.

## Key Features

- **Speech Queue Management**: Buffered text-to-speech with automatic queue processing
- **Audio-Visual Synchronization**: Coordinates speech output with facial expressions
- **Microphone State Control**: Intelligent microphone activation/deactivation during speech
- **Facial Expression Control**: Dynamic emotion and color management for robot face
- **Error State Handling**: Visual feedback for system errors and warnings
- **RPC Thrift Interface**: Remote procedure call interface for external control
- **Thread-Safe Operations**: Mutex-protected shared resources and state management

## Technical Specifications

- **Update Frequency**: 5 Hz (0.2-second period)
- **Thread Safety**: Mutex-protected text buffer and state variables
- **Interface Protocol**: YARP Thrift RPC for remote control
- **Audio Integration**: Real-time audio player and recorder status monitoring
- **State Machine**: Comprehensive state management for speech and listening modes

## Dependencies

### System Dependencies
- **CMake** (>= 3.16): Build system (higher version required than other modules)
- **C++ Compiler**: Supporting C++11 standard
- **YARP** (Yet Another Robot Platform): Middleware for robot communication
  - Components needed: `os`, `dev`, `sig`

### Interface Dependencies
- **headSynchronizerRPC**: Thrift-generated RPC interface
- **google_speech**: Google Speech Recognition interface
- **google_synthesis**: Google Text-to-Speech interface

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

The executable will be created in `build/bin/headSynchronizer`.

### Building Standalone
If you want to build only the headSynchronizer module:

1. Navigate to the headSynchronizer directory:
```bash
cd aux_modules/headSynchronizer
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

**Note**: For standalone building, you'll need to ensure that all interface dependencies are built first:
- `interfaces/headSynchronizerRPC`
- `interfaces/google_speech`
- `interfaces/google_synthesis`

## Configuration Parameters

The module accepts the following configuration parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | "HeadSynchronizer" | Module name (used for port naming) |
| `period` | double | 0.2 | Module update period in seconds |

## Port Configuration

### Input Ports
| Port Name | Format | Description |
|-----------|--------|-------------|
| `/{name}/googleStatus:i` | `yarp::os::Bottle` | Google service status updates |
| `/{name}/microphoneStatus:i` | `yarp::sig::AudioRecorderStatus` | Microphone recorder status |
| `/{name}/playerStatus:i` | `yarp::sig::AudioPlayerStatus` | Audio player status |

### Output Ports
| Port Name | Format | Description |
|-----------|--------|-------------|
| `/{name}/result:o` | `yarp::os::Bottle` | Text synthesis requests |
| `/{name}/microphone:o` | `yarp::os::Bottle` | Microphone control commands |
| `/{name}/player:o` | `yarp::os::Bottle` | Audio player control commands |
| `/{name}/face:o` | `yarp::os::Bottle` | Facial expression control |

### RPC Interface
| Port Name | Format | Description |
|-----------|--------|-------------|
| `/{name}/thrift:s` | Thrift RPC | Remote procedure call interface |

## RPC Thrift Interface

The module provides a comprehensive RPC interface for remote control:

### Speech Control
```bash
# Add text to speech queue
yarp rpc /{name}/thrift:s
>> say "Hello, welcome to the tour!"

# Check if currently speaking
>> isSpeaking

# Pause/resume speech
>> pauseSpeaking
>> continueSpeaking

# Clear speech buffer and reset
>> reset
```

### Microphone Control
```bash
# Start listening mode
>> startHearing

# Stop listening mode
>> stopHearing

# Check listening status
>> isHearing
```

### Facial Expression Control
```bash
# Change to happy face
>> happyFace

# Change to sad face with warning
>> sadFaceWarning

# Change to error face
>> sadFaceError

# Change to busy face
>> busyFace

# Change to busy error face
>> busyFaceError
```

## Execution

### Prerequisites
Before running the headSynchronizer module, ensure that:

1. **YARP Server is running**:
   ```bash
   yarp server
   ```

2. **Google Services**:
   - Google Speech Recognition service
   - Google Text-to-Speech synthesis service

3. **Audio System**:
   - Audio recorder (microphone input)
   - Audio player (speech output)

4. **Face Expression System**: Robot face display control system

### Running the Module

#### Basic Execution
```bash
./headSynchronizer
```

#### With Custom Name
```bash
./headSynchronizer --name MyHeadSynchronizer
```

#### With Configuration File
Create a configuration file (e.g., `headSynchronizer.ini`):
```ini
name HeadSynchronizer
period 0.1
```

Then run:
```bash
./headSynchronizer --from headSynchronizer.ini
```

### YARP Port Information
The module creates the following YARP ports:
- `/{name}/googleStatus:i`: Input port for Google service status
- `/{name}/microphoneStatus:i`: Input port for microphone status
- `/{name}/playerStatus:i`: Input port for audio player status
- `/{name}/result:o`: Output port for synthesis requests
- `/{name}/microphone:o`: Output port for microphone control
- `/{name}/player:o`: Output port for player control
- `/{name}/face:o`: Output port for facial expressions
- `/{name}/thrift:s`: RPC server for remote control

## How It Works

### Speech Processing Pipeline
1. **Text Queuing**: Incoming text is added to thread-safe buffer
2. **Audio State Check**: Waits for audio player to become available
3. **Microphone Management**: Automatically stops listening during speech
4. **Synthesis Request**: Sends text to Google TTS service
5. **Playback Monitoring**: Tracks audio playback status
6. **Queue Processing**: Automatically processes next item in buffer
7. **State Cleanup**: Removes completed items from buffer

### State Management
```cpp
// Simplified state machine
if (!isAudioPlaying() && !textBuffer.empty()) {
    stopHearing();              // Disable microphone
    sendToSynthesis(text);      // Request TTS
    waitForAudioStart();        // Wait for playback
    waitForAudioEnd();          // Wait for completion
    removeFromBuffer(text);     // Clean up
    enableListeningIfNeeded();  // Re-enable microphone
}
```

### Facial Expression System
The module controls robot facial expressions with:
- **Emotion States**: Happy, sad, busy states
- **Color Control**: Dynamic ear and mouth coloring
- **Context Awareness**: Expression changes based on system state
- **Error Visualization**: Visual feedback for system errors

#### Color Scheme
| State | Ears Color | Mouth Color | Description |
|-------|------------|-------------|-------------|
| Listening | Green (0,128,0) | Green (0,128,0) | Ready to hear user |
| Not Listening | Blue (0,0,255) | Green (0,128,0) | Normal idle state |
| Warning | Default | Yellow (238,210,2) | Warning condition |
| Error | Red (255,0,0) | Red (255,0,0) | Error state |
| Busy | Default | White (255,255,255) | Processing state |

### Google Service Integration
The module responds to Google service status updates:
- **"Busy"**: Sets busy face, maintains current error state
- **"Empty"**: Returns to happy face, enables listening
- **"Done"**: Completes interaction, returns to appropriate state
- **"Failure"**: Sets error face, enables listening for retry

## Integration with Robot Systems

### Required Services
- **Google Speech Recognition**: Converts user speech to text
- **Google Text-to-Speech**: Converts robot responses to audio
- **Audio Recorder**: Captures user voice input
- **Audio Player**: Outputs robot speech
- **Face Expression System**: Controls robot facial display

### Typical Interaction Flow
1. Robot enables microphone (green ears)
2. User speaks → Google Speech Recognition
3. System processes → sets busy face
4. Response generated → queued for speech
5. Microphone disabled → speech output begins
6. Speech completes → microphone re-enabled
7. System ready for next interaction

## Dependencies on Other Modules

This module requires:
- **YARP Server**: Must be running for inter-process communication
- **Google Speech/Synthesis Services**: For voice interaction capabilities
- **Audio System**: Microphone and speaker hardware interfaces
- **Face Expression Module**: For visual feedback during interactions
- **Interface Libraries**: headSynchronizerRPC, google_speech, google_synthesis

## Troubleshooting

### Common Issues

1. **YARP Connection errors**:
   - Ensure `yarp server` is running
   - Check if YARP namespace is correctly configured: `yarp namespace`
   - Verify ports are not already in use: `yarp name list`

2. **Interface Build errors**:
   - Ensure all Thrift interface libraries are built first
   - Check CMake can find headSynchronizerRPC, google_speech, google_synthesis
   - Verify Thrift compiler is available

3. **Audio System errors**:
   - Check audio recorder/player connections
   - Verify audio hardware is accessible
   - Monitor audio status ports for proper updates

4. **Google Service errors**:
   - Ensure Google services are running and accessible
   - Check authentication and API credentials
   - Monitor Google status port for proper updates

5. **Speech Buffer issues**:
   - Use `reset` RPC command to clear stuck buffers
   - Monitor thread-safe buffer operations
   - Check for memory leaks in text queuing

6. **Facial Expression errors**:
   - Verify face expression system is running
   - Check face output port connectivity
   - Test expression changes independently

### Debug Mode
The module uses YARP logging. Enable debug output:

```bash
export YARP_VERBOSE=1
./headSynchronizer
```

### YARP Network Diagnostics
```bash
# Check YARP server status
yarp detect --write

# List all active ports
yarp name list

# Check specific port connections
yarp name query /HeadSynchronizer/thrift:s
yarp name query /HeadSynchronizer/result:o

# Test RPC interface
yarp rpc /HeadSynchronizer/thrift:s
>> help

# Monitor audio status
yarp read ... /HeadSynchronizer/playerStatus:i
yarp read ... /HeadSynchronizer/microphoneStatus:i
```

## Performance Considerations

- **Update Frequency**: 5 Hz provides responsive state management
- **Thread Safety**: Mutex protection may cause brief blocking during buffer operations
- **Memory Management**: Text buffer automatically managed, but monitor for memory leaks
- **Audio Latency**: Speech synthesis and playback introduce interaction delays
- **Network Overhead**: Multiple YARP connections require adequate network bandwidth

## Advanced Usage

### Custom Integration Example
```cpp
// C++ example using RPC client
yarp::os::RpcClient client;
client.open("/myApp/headSync:c");
yarp::os::Network::connect("/myApp/headSync:c", "/HeadSynchronizer/thrift:s");

// Send speech request
yarp::os::Bottle cmd, reply;
cmd.addString("say");
cmd.addString("Welcome to our museum!");
client.write(cmd, reply);

// Check status
cmd.clear();
cmd.addString("isSpeaking");
client.write(cmd, reply);
bool speaking = reply.get(0).asBool();
```

### Batch Operations
```bash
# Queue multiple speech items
yarp rpc /HeadSynchronizer/thrift:s
>> say "First message"
>> say "Second message"
>> say "Third message"

# They will be processed sequentially
```

## Safety Considerations

- **Audio Feedback Prevention**: Automatic microphone disabling during speech
- **Buffer Overflow Protection**: Monitor text buffer size for memory safety
- **Error State Recovery**: Use `reset` command to recover from error states
- **Service Dependencies**: Ensure all required services are healthy before operation