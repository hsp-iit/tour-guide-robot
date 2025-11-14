# Silero VAD (Voice Activity Detection) Module

## Overview

The Silero VAD module provides real-time voice activity detection for the tour-guide robot using the Silero Voice Activity Detection model. It processes incoming audio streams to detect speech segments, filters out silence and background noise, and forwards only the audio containing speech to downstream processing modules. The module integrates seamlessly with YARP for audio streaming and provides RPC interfaces for runtime configuration of detection parameters.

## Key Features

- **Real-time Voice Activity Detection**: Uses the state-of-the-art Silero VAD model for accurate speech detection
- **ONNX Runtime Integration**: High-performance inference using ONNX runtime
- **YARP Audio Streaming**: Seamless integration with YARP audio ports and pipelines
- **Configurable Detection Parameters**: Runtime adjustment of threshold and gap allowance
- **Audio Buffering**: Intelligent buffering with pre-speech padding for complete capture
- **Gap Handling**: Configurable gap allowance to handle pauses in continuous speech
- **RPC Control Interface**: Remote parameter adjustment during runtime
- **Wake Word Integration**: Integration with wake word detection systems
- **Multiple Sample Rate Support**: Support for 8kHz and 16kHz audio sampling rates

## Technical Specifications

- **Audio Formats**: 16-bit PCM audio at 8kHz or 16kHz sampling rates
- **Model**: Silero VAD ONNX model for neural network-based voice detection
- **Buffer Size**: 512 samples (16kHz) or 256 samples (8kHz) per processing packet
- **Processing**: Real-time processing with 32ms packets (configurable)
- **Memory**: Efficient streaming with configurable buffer management
- **Latency**: Low-latency processing suitable for real-time applications

## Dependencies

### System Dependencies
- **CMake** (>= 3.1): Build system for C++ compilation
- **C++ Compiler**: Supporting C++11 standard with STL
- **YARP** (Yet Another Robot Platform): Robot middleware and communication
  - Components: `os`, `sig` for audio processing
- **ONNX Runtime**: High-performance inference engine for ONNX models

### Ubuntu/Debian Installation

#### Basic Dependencies
```bash
sudo apt-get update
sudo apt-get install cmake build-essential pkg-config wget
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

# Set environment variables
echo 'export YARP_ROOT=/usr/local' >> ~/.bashrc
echo 'export PATH=$PATH:$YARP_ROOT/bin' >> ~/.bashrc
source ~/.bashrc
```

#### ONNX Runtime Installation
```bash
# Download and extract ONNX Runtime (version 1.20.1 or later)
cd /usr/local/src/robot
wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
tar -xzf onnxruntime-linux-x64-1.20.1.tgz

# Set environment variable
export ONNX_PATH=/usr/local/src/robot/onnxruntime-linux-x64-1.20.1
echo 'export ONNX_PATH=/usr/local/src/robot/onnxruntime-linux-x64-1.20.1' >> ~/.bashrc
source ~/.bashrc
```

### Fedora/CentOS Installation

#### Basic Dependencies
```bash
sudo yum install cmake gcc-c++ pkgconfig wget
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

# Set environment variables
echo 'export YARP_ROOT=/usr/local' >> ~/.bashrc
echo 'export PATH=$PATH:$YARP_ROOT/bin' >> ~/.bashrc
source ~/.bashrc
```

#### ONNX Runtime Installation
```bash
# Download and extract ONNX Runtime
sudo mkdir -p /usr/local/src/robot
cd /usr/local/src/robot
sudo wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
sudo tar -xzf onnxruntime-linux-x64-1.20.1.tgz

# Set environment variable
export ONNX_PATH=/usr/local/src/robot/onnxruntime-linux-x64-1.20.1
echo 'export ONNX_PATH=/usr/local/src/robot/onnxruntime-linux-x64-1.20.1' >> ~/.bashrc
source ~/.bashrc
```

## Model Setup

### Download Silero VAD Model
Download the Silero ONNX model file from their repository:

```bash
# Create model directory
sudo mkdir -p /usr/local/src/robot/silero-vad/src/silero_vad/data

# Download the model
cd /usr/local/src/robot/silero-vad/src/silero_vad/data
sudo wget https://github.com/snakers4/silero-models/raw/master/models/silero_vad.onnx

# Or download from alternative source
sudo wget https://models.silero.ai/models/vad_models/silero_vad.onnx
```

### Model File Verification
```bash
# Verify the model file exists and is readable
ls -la /usr/local/src/robot/silero-vad/src/silero_vad/data/silero_vad.onnx

# Check file size (should be around 1.4MB)
du -h /usr/local/src/robot/silero-vad/src/silero_vad/data/silero_vad.onnx
```

## Interface Dependencies

The Silero VAD module requires several custom YARP interfaces (IDL files):

### Required Interfaces
- **SileroVADMsgs**: RPC interface for VAD parameter control
- **WakeMsgs**: Interface for wake word detection integration

### Building Interface Dependencies
These interfaces should be built as part of the main project:

```bash
# Navigate to the interfaces directory
cd /path/to/tour-guide-robot/interfaces

# Build all interface libraries
mkdir build && cd build
cmake ..
make -j$(nproc)
make install
```

## Build Instructions

### Building as Part of the Main Project
1. Ensure ONNX Runtime is properly installed and ONNX_PATH is set:
```bash
export ONNX_PATH=/usr/local/src/robot/onnxruntime-linux-x64-1.20.1
```

2. Navigate to the project root directory:
```bash
cd /path/to/tour-guide-robot
```

3. Create and enter build directory:
```bash
mkdir build && cd build
```

4. Configure the build:
```bash
cmake ..
```

5. Build the project:
```bash
make -j$(nproc)
```

6. Install (optional):
```bash
make install
```

### Building Standalone
If you want to build only the sileroVAD module:

1. Ensure all interface dependencies are built and installed first:
```bash
cd /path/to/tour-guide-robot/interfaces
mkdir build && cd build
cmake ..
make -j$(nproc)
make install
```

2. Set environment variables:
```bash
export ONNX_PATH=/usr/local/src/robot/onnxruntime-linux-x64-1.20.1
```

3. Navigate to the sileroVAD directory:
```bash
cd /path/to/tour-guide-robot/aux_modules/speechProcessing/sileroVAD
```

4. Create and enter build directory:
```bash
mkdir build && cd build
```

5. Configure and build:
```bash
cmake ..
make -j$(nproc)
```

## Configuration

### Configuration Parameters
The module accepts various configuration parameters via YARP ResourceFinder:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `audio_input_port_name` | `/vad/audio:i` | Input port for audio stream |
| `filtered_audio_output_port_name` | `/vad/audio:o` | Output port for filtered audio |
| `wake_word_client_port_name` | `/vad/rpc:o` | RPC client port for wake word notifications |
| `vad_server_port_name` | `/vad/rpc:i` | RPC server port for parameter control |
| `vad_frequency` | `16000` | Audio sampling frequency (8000 or 16000 Hz) |
| `vad_threshold` | `0.9` | Voice detection threshold (0.0-1.0) |
| `vad_gap_allowance` | `18` | Number of silence packets to allow in speech |
| `vad_save_gap` | `true` | Whether to save gap periods in output audio |
| `vad_save_prior_to_detection` | `15` | Packets to save before speech detection |
| `vad_reenable_keyword` | `0` | Re-enable keyword detection after each clip |
| `model_path` | `/usr/local/src/robot/silero-vad/src/silero_vad/data/silero_vad.onnx` | Path to Silero VAD model |

### Configuration File Example
Create a configuration file (e.g., `vad_config.ini`):

```ini
[GENERAL]
audio_input_port_name        /vad/audio:i
filtered_audio_output_port_name  /vad/audio:o
vad_server_port_name         /vad/rpc:i
wake_word_client_port_name   /vad/rpc:o

[VAD_PARAMETERS]
vad_frequency                16000
vad_threshold               0.85
vad_gap_allowance           20
vad_save_gap                true
vad_save_prior_to_detection  18
vad_reenable_keyword        1

[MODEL]
model_path                  /usr/local/src/robot/silero-vad/src/silero_vad/data/silero_vad.onnx
```

## Usage

### Basic Execution
```bash
# Start YARP server
yarpserver &

# Run Silero VAD with default configuration
./sileroVAD

# Run with custom configuration file
./sileroVAD --from vad_config.ini

# Run with command line parameters
./sileroVAD --vad_threshold 0.8 --vad_gap_allowance 25
```

### YARP Port Connections
The module creates and uses these YARP ports:

#### Input Ports
- `/vad/audio:i`: Audio input from microphone or audio source
- `/vad/rpc:i`: RPC server for parameter control

#### Output Ports
- `/vad/audio:o`: Filtered audio output (speech segments only)
- `/vad/rpc:o`: RPC client for wake word detector communication

### Connection Examples
```bash
# Connect audio source to VAD input
yarp connect /audioSource:o /vad/audio:i

# Connect VAD output to speech recognition
yarp connect /vad/audio:o /speechRecognition/audio:i

# Connect VAD to wake word detector
yarp connect /vad/rpc:o /wake/rpc:i
```

### RPC Control Interface
Control the module parameters via RPC:

```bash
# Connect to RPC interface
yarp rpc /vad/rpc:i

# Available commands:
>> setThreshold 0.85          # Set detection threshold
>> setGapAllowance 22         # Set gap allowance in packets
```

## Integration with Speech Pipeline

### Typical Audio Processing Pipeline
```
Microphone → Silero VAD → Speech Recognition → Tour Manager
     ↓
Wake Word Detector ←─────────┘
```

### Connection Setup Script
```bash
#!/bin/bash
# vad_setup.sh - Setup VAD in speech pipeline

# Start YARP server
yarpserver &
sleep 2

# Start Silero VAD
./sileroVAD --vad_threshold 0.8 &
sleep 2

# Connect microphone to VAD
yarp connect /microphone:o /vad/audio:i

# Connect VAD to speech recognition
yarp connect /vad/audio:o /speechRecognition/audio:i

# Connect VAD to wake word system
yarp connect /vad/rpc:o /wake/rpc:i

echo "Silero VAD pipeline configured"
```

### Wake Word Integration
The module can work with wake word detection systems:

```cpp
// Example integration with wake word detection
void onAudioClipComplete() {
    if (m_vadReenableKeyword) {
        // Re-enable wake word detection after processing audio clip
        m_rpcClient.stop();  // Stop wake word detector to resume listening
    }
}
```

## Voice Activity Detection Algorithm

### Detection Process
1. **Audio Buffering**: Incoming audio is buffered in 32ms packets
2. **Feature Extraction**: Audio is normalized and prepared for the neural network
3. **Neural Network Inference**: Silero VAD model processes audio features
4. **Threshold Comparison**: Output probability compared against threshold
5. **Gap Handling**: Silence gaps within speech are managed using gap allowance
6. **Buffer Management**: Audio buffers are maintained with pre-speech padding

### Detection States
- **Idle**: No speech detected, audio discarded
- **Pre-Speech**: Speech detected, collecting buffered audio including pre-speech padding
- **Active Speech**: Continuous speech detection, forwarding audio
- **Gap Period**: Brief silence within speech, maintaining buffer based on gap allowance
- **Post-Speech**: End of speech detected, finalizing and sending audio clip

### Parameter Tuning Guidelines

#### Threshold (`vad_threshold`)
- **0.3-0.5**: Very sensitive, may include background noise
- **0.6-0.8**: Balanced sensitivity for normal environments
- **0.85-0.95**: Conservative, requires clear speech (default: 0.9)

#### Gap Allowance (`vad_gap_allowance`)
- **5-10 packets**: Short pauses only, good for commands
- **15-25 packets**: Natural speech pauses (default: 18)
- **30+ packets**: Long pauses, good for presentations

#### Pre-Detection Saving (`vad_save_prior_to_detection`)
- **5-10 packets**: Minimal pre-speech capture
- **15-20 packets**: Standard pre-speech padding (default: 15)
- **25+ packets**: Extended pre-speech capture for complete words

## Performance Optimization

### CPU Usage Optimization
```bash
# Lower frequency for reduced CPU usage (if acceptable)
./sileroVAD --vad_frequency 8000

# Adjust ONNX runtime threads (set in code)
# Single-threaded for lower resource usage
init_engine_threads(1, 1);
```

### Memory Usage
- **Buffer Management**: Configurable buffer sizes for memory efficiency
- **Model Loading**: One-time model loading at startup
- **State Management**: Minimal memory footprint for recurrent states

### Latency Considerations
- **Packet Size**: 32ms processing packets (512 samples at 16kHz)
- **Pre-buffering**: Adds latency but ensures complete speech capture
- **Neural Network**: ~1-2ms inference time per packet on modern CPUs

## Troubleshooting

### Common Issues

1. **Module Won't Start**:
   ```bash
   # Check ONNX Runtime installation
   echo $ONNX_PATH
   ls -la $ONNX_PATH/lib/libonnxruntime.so

   # Check YARP server
   yarp detect

   # Verify model file
   ls -la /usr/local/src/robot/silero-vad/src/silero_vad/data/silero_vad.onnx
   ```

2. **ONNX Runtime Not Found**:
   ```bash
   # Set environment variable
   export ONNX_PATH=/usr/local/src/robot/onnxruntime-linux-x64-1.20.1

   # Check library path
   export LD_LIBRARY_PATH=$ONNX_PATH/lib:$LD_LIBRARY_PATH

   # Verify library loading
   ldd ./sileroVAD | grep onnx
   ```

3. **Model File Missing**:
   ```bash
   # Download Silero VAD model
   sudo mkdir -p /usr/local/src/robot/silero-vad/src/silero_vad/data
   cd /usr/local/src/robot/silero-vad/src/silero_vad/data
   sudo wget https://models.silero.ai/models/vad_models/silero_vad.onnx
   ```

4. **No Audio Detection**:
   ```bash
   # Lower threshold for testing
   yarp rpc /vad/rpc:i
   >> setThreshold 0.3

   # Check audio input
   yarp read ... /vad/audio:i

   # Verify audio format (16-bit PCM, 8kHz or 16kHz)
   ```

5. **False Positives**:
   ```bash
   # Increase threshold
   yarp rpc /vad/rpc:i
   >> setThreshold 0.95

   # Reduce gap allowance
   >> setGapAllowance 10
   ```

6. **Build Errors**:
   ```bash
   # Check interface dependencies
   find /usr/local -name "*SileroVADMsgs*" 2>/dev/null
   find /usr/local -name "*WakeMsgs*" 2>/dev/null

   # Rebuild interfaces if missing
   cd /path/to/tour-guide-robot/interfaces
   mkdir build && cd build
   cmake .. && make -j$(nproc) && make install
   ```

### Debug Mode
Enable debug logging for troubleshooting:

```bash
export YARP_VERBOSE=1
./sileroVAD
```

### Audio Format Debugging
```bash
# Check audio properties
yarp read ... /vad/audio:i | head -20

# Monitor detection probabilities (requires code modification)
# Add debug prints in predict() function to see speech probabilities
```

### Performance Monitoring
```bash
# Monitor CPU usage
top -p `pgrep sileroVAD`

# Monitor memory usage
ps -o pid,vsz,rss,comm -p `pgrep sileroVAD`

# Monitor YARP port statistics
yarp info /vad/audio:i
yarp info /vad/audio:o
```

## Advanced Usage

### Custom Model Integration
```cpp
// Use custom Silero VAD model
std::string custom_model_path = "/path/to/custom_silero_model.onnx";
Detector detector(16000, 18, true, 0.9, 15, custom_model_path, ...);
```

### Parameter Optimization Script
```python
#!/usr/bin/env python3
# vad_tuning.py - Optimize VAD parameters for your environment

import yarp
import time

def test_vad_parameters(threshold, gap_allowance):
    yarp.Network.init()
    rpc_port = yarp.RpcClient()
    rpc_port.open("/vad_tuner")
    yarp.Network.connect("/vad_tuner", "/vad/rpc:i")

    # Set parameters
    cmd = yarp.Bottle()
    cmd.addString("setThreshold")
    cmd.addFloat64(threshold)
    rpc_port.write(cmd)

    cmd.clear()
    cmd.addString("setGapAllowance")
    cmd.addInt32(gap_allowance)
    rpc_port.write(cmd)

    print(f"Testing threshold: {threshold}, gap_allowance: {gap_allowance}")
    time.sleep(10)  # Test for 10 seconds

    rpc_port.close()
    yarp.Network.fini()

# Test different parameter combinations
thresholds = [0.7, 0.8, 0.85, 0.9, 0.95]
gap_allowances = [10, 15, 18, 20, 25]

for threshold in thresholds:
    for gap_allowance in gap_allowances:
        test_vad_parameters(threshold, gap_allowance)
```

### Integration with Audio Recording
```bash
# Record VAD-filtered audio for analysis
yarp connect /vad/audio:o /audioRecorder/audio:i

# Record original audio for comparison
yarp connect /microphone:o /audioRecorder/original:i
```

## Safety and Performance Considerations

- **Real-time Constraints**: Ensure system can handle 16kHz audio processing
- **Memory Management**: Monitor buffer usage for long-running applications
- **Model Security**: Verify Silero VAD model integrity before deployment
- **Audio Quality**: Ensure proper audio preprocessing for optimal detection
- **Parameter Validation**: Validate threshold and gap allowance ranges

## Extension and Customization

### Custom Detection Logic
```cpp
// Extend Detector class for custom behavior
class CustomDetector : public Detector {
public:
    void onSpeechDetected(float probability) override {
        // Custom logic when speech is detected
        if (probability > 0.95) {
            // High confidence speech
            notifyHighConfidenceDetection();
        }
    }
};
```

### Multi-Channel Audio Support
```cpp
// Process multiple audio channels
void processMultiChannelAudio(const yarp::sig::Sound& sound) {
    int channels = sound.getChannels();
    for (int ch = 0; ch < channels; ch++) {
        // Process each channel separately
        processChannel(sound, ch);
    }
}
```

### Integration with Cloud Services
```cpp
// Send VAD results to cloud for analysis
void sendToCloud(const std::vector<float>& audio_segment) {
    // Upload audio segment to cloud speech service
    cloudSpeechAPI.transcribe(audio_segment);
}
```