# Wake Word Detection Module

## Overview

The Wake Word Detection module provides real-time wake word detection for the tour-guide robot using Picovoice's Porcupine engine. It continuously monitors audio input for specific wake words and activates audio streaming when detected. The module integrates with YARP for audio processing, face expression control, and notification systems, making it ideal for voice-activated robot interactions. Once a wake word is detected, the module streams subsequent audio to downstream processing modules until instructed to stop.

## Key Features

- **Real-time Wake Word Detection**: Uses Picovoice Porcupine engine for accurate wake word recognition
- **Custom Wake Word Support**: Supports custom wake word models (.ppn files) from Picovoice Console
- **Audio Streaming Control**: Automatically starts audio streaming after wake word detection
- **YARP Integration**: Seamless integration with YARP audio ports and communication systems
- **Visual Feedback**: Eye color changes and facial expressions on wake word detection
- **Audio Buffering**: Includes pre-detection audio samples to capture complete phrases
- **RPC Control Interface**: Remote control for stopping audio streaming
- **Notification System**: Plays audio notifications when wake words are detected
- **Configurable Sensitivity**: Adjustable detection sensitivity for different environments

## Technical Specifications

- **Audio Format**: 16kHz, 16-bit PCM audio (required by Porcupine engine)
- **Wake Word Engine**: Picovoice Porcupine v3.0
- **Detection Method**: Deep learning-based keyword spotting
- **Latency**: Low-latency processing with audio buffering compensation
- **Framework**: C++ with YARP middleware integration
- **Licensing**: Requires Picovoice access key and custom keyword files

## Dependencies

### System Dependencies
- **CMake** (>= 3.1): Build system with FetchContent support
- **C++ Compiler**: Supporting C++11 standard with STL
- **Git**: For FetchContent downloading of Porcupine library
- **YARP** (Yet Another Robot Platform): Robot middleware and communication
  - Components: `os`, `sig` for audio processing
- **Picovoice Porcupine**: Wake word detection engine (auto-downloaded via CMake)

### Ubuntu/Debian Installation

#### Basic Dependencies
```bash
sudo apt-get update
sudo apt-get install cmake build-essential git pkg-config
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

### Fedora/CentOS Installation

#### Basic Dependencies
```bash
sudo yum install cmake gcc-c++ git pkgconfig
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

## Picovoice Setup

### Account Setup
You will need your own access key and keyword file from Picovoice:

1. **Create Account**: Go to https://picovoice.ai/ and create a free account
2. **Get Access Key**: Navigate to your console and copy your access key
3. **Create Custom Wake Word**: Use the Picovoice Console to create custom wake words
4. **Download Model Files**: Download the `.ppn` keyword files for your platform

### Model Files Setup
```bash
# Create directory for wake word models
sudo mkdir -p /usr/local/src/robot/tour-guide-robot/aux_modules/speechProcessing/wakeWordDetection/demo

# Copy your custom wake word file (example)
sudo cp ~/Downloads/Hey-R-one_en_linux_v3_0_0.ppn \
   /usr/local/src/robot/tour-guide-robot/aux_modules/speechProcessing/wakeWordDetection/demo/

# Set appropriate permissions
sudo chmod 644 /usr/local/src/robot/tour-guide-robot/aux_modules/speechProcessing/wakeWordDetection/demo/*.ppn
```

### Access Key Configuration
Store your Picovoice access key securely:

```bash
# Option 1: Environment variable (recommended for development)
export PICOVOICE_ACCESS_KEY="your_access_key_here"
echo 'export PICOVOICE_ACCESS_KEY="your_access_key_here"' >> ~/.bashrc

# Option 2: Configuration file (recommended for production)
echo "accessKey your_access_key_here" > /etc/robot/picovoice.conf
```

## Interface Dependencies

The Wake Word Detection module requires custom YARP interfaces (IDL files):

### Required Interfaces
- **WakeMsgs**: RPC interface for wake word detection control

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
1. Navigate to the project root directory:
```bash
cd /path/to/tour-guide-robot
```

2. Create and enter build directory:
```bash
mkdir build && cd build
```

3. Configure the build (Porcupine will be automatically downloaded):
```bash
cmake ..
```

4. Build the project:
```bash
make -j$(nproc)
```

The Porcupine library will be automatically downloaded and configured via CMake FetchContent.

### Building Standalone
If you want to build only the wakeWordDetection module:

1. Ensure all interface dependencies are built and installed first:
```bash
cd /path/to/tour-guide-robot/interfaces
mkdir build && cd build
cmake ..
make -j$(nproc)
make install
```

2. Navigate to the wakeWordDetection directory:
```bash
cd /path/to/tour-guide-robot/aux_modules/speechProcessing/wakeWordDetection
```

3. Create and enter build directory:
```bash
mkdir build && cd build
```

4. Configure and build (Porcupine auto-download will occur):
```bash
cmake ..
make -j$(nproc)
```

### Porcupine Auto-Download
The build system automatically downloads Porcupine v3.0 from GitHub:

```cmake
FetchContent_Declare(
    porcupine
    GIT_REPOSITORY https://github.com/Picovoice/porcupine.git
    GIT_TAG v3.0
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/porcupine
)
```

## Configuration

### Configuration Parameters
The module accepts various configuration parameters via YARP ResourceFinder:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `audio_input_port_name` | `/wake/audio:i` | Input port for audio stream |
| `filtered_audio_output_port_name` | `/wake/audio:o` | Output port for filtered audio |
| `vad_server_port_name` | `/wake/rpc:i` | RPC server port for control commands |
| `face_server_port_name` | `/wake/face:o` | Port for facial expression control |
| `notification_port_name` | `/wake/notification:o` | Port for wake word notifications |
| `accessKey` | `""` | Picovoice access key (required) |
| `model_path` | `/usr/local/src/robot/.../porcupine_params.pv` | Path to Porcupine model |
| `keyword_path` | `/usr/local/src/robot/.../Hey-R-one_en_linux_v3_0_0.ppn` | Path to keyword file |
| `detector_sensitivity` | `0.6` | Detection sensitivity (0.0-1.0) |

### Configuration File Example
Create a configuration file (e.g., `wake_config.ini`):

```ini
[GENERAL]
audio_input_port_name           /wake/audio:i
filtered_audio_output_port_name /wake/audio:o
vad_server_port_name           /wake/rpc:i
face_server_port_name          /wake/face:o
notification_port_name         /wake/notification:o

[PICOVOICE]
accessKey                      YOUR_ACCESS_KEY_HERE
detector_sensitivity           0.7

[MODEL_PATHS]
model_path                     /usr/local/src/robot/tour-guide-robot/aux_modules/speechProcessing/wakeWordDetection/porcupine/lib/common/porcupine_params.pv
keyword_path                   /usr/local/src/robot/tour-guide-robot/aux_modules/speechProcessing/wakeWordDetection/demo/Hey-R-one_en_linux_v3_0_0.ppn
```

### Custom Wake Word Setup
1. **Create Wake Word**: Use Picovoice Console to create custom wake words
2. **Download Model**: Download the `.ppn` file for Linux x86_64
3. **Configure Path**: Update `keyword_path` parameter to point to your model
4. **Test Sensitivity**: Adjust `detector_sensitivity` based on your environment

## Usage

### Basic Execution
```bash
# Start YARP server
yarpserver &

# Run Wake Word Detection with configuration file
./wakeWordDetection --from wake_config.ini

# Run with command line parameters
./wakeWordDetection \
  --accessKey "YOUR_ACCESS_KEY" \
  --keyword_path "/path/to/your_wake_word.ppn" \
  --detector_sensitivity 0.7
```

### YARP Port Connections
The module creates and uses these YARP ports:

#### Input Ports
- `/wake/audio:i`: Audio input from microphone or audio source
- `/wake/rpc:i`: RPC server for control commands

#### Output Ports
- `/wake/audio:o`: Audio output (streams after wake word detection)
- `/wake/face:o`: Facial expression control commands
- `/wake/notification:o`: Wake word detection notifications

### Connection Examples
```bash
# Connect audio source to wake word detector
yarp connect /audioSource:o /wake/audio:i

# Connect wake word audio output to speech recognition
yarp connect /wake/audio:o /speechRecognition/audio:i

# Connect face control to face expression module
yarp connect /wake/face:o /faceExpression/notify:i

# Connect notifications to sound player
yarp connect /wake/notification:o /wake_notification:i
```

### RPC Control Interface
Control the module via RPC commands:

```bash
# Connect to RPC interface
yarp rpc /wake/rpc:i

# Available commands:
>> stop    # Stop audio streaming and return to wake word detection mode
```

## Audio Notification System

### Python Notification Player
The module includes a Python script for playing audio notifications:

```bash
# Run the notification player
python3 play_notification.py --file notification.wav

# The script listens on /wake_notification:i for "play_sound" commands
```

### Custom Notification Sounds
```bash
# Create custom notification sound (16kHz WAV recommended)
sox input.wav -r 16000 -c 1 notification.wav

# Test notification playback
echo "play_sound" | yarp write ... /wake_notification:i
```

## Integration with Speech Pipeline

### Typical Audio Processing Pipeline
```
Microphone → Wake Word Detector → Speech Recognition → Tour Manager
     ↓              ↓
Face Expression ← Visual Feedback
     ↓
Notification Sound
```

### Complete Setup Script
```bash
#!/bin/bash
# wake_word_setup.sh - Complete wake word detection setup

# Start YARP server
yarpserver &
sleep 2

# Start wake word detection
./wakeWordDetection --from wake_config.ini &
sleep 2

# Start notification player
python3 play_notification.py --file notification.wav &
sleep 1

# Connect audio pipeline
yarp connect /microphone:o /wake/audio:i
yarp connect /wake/audio:o /speechRecognition/audio:i

# Connect visual feedback
yarp connect /wake/face:o /faceExpression/notify:i

# Connect notifications
yarp connect /wake/notification:o /wake_notification:i

echo "Wake word detection system started"
echo "Say your wake word to activate audio streaming"
```

### Integration with VAD
For optimal performance, integrate with Voice Activity Detection:

```bash
# Chain wake word detection with VAD
yarp connect /wake/audio:o /vad/audio:i
yarp connect /vad/audio:o /speechRecognition/audio:i

# VAD controls wake word streaming
yarp connect /vad/rpc:o /wake/rpc:i
```

## Wake Word Detection Algorithm

### Detection Process
1. **Audio Buffering**: Incoming 16kHz audio is buffered into frames
2. **Frame Processing**: Audio frames are processed by Porcupine engine
3. **Keyword Detection**: Neural network analyzes frames for wake word patterns
4. **Threshold Evaluation**: Detection confidence compared against sensitivity setting
5. **State Change**: On detection, switches from monitoring to streaming mode
6. **Buffer Playback**: Plays back pre-detection audio samples to capture complete phrase
7. **Continuous Streaming**: Streams all subsequent audio until stop command received

### Detection States
- **Monitoring Mode**: Continuously analyzing audio for wake word, audio not forwarded
- **Detection Event**: Wake word detected, visual/audio feedback triggered
- **Streaming Mode**: All audio forwarded to output port
- **Stop Command**: Return to monitoring mode via RPC command

### Audio Buffer Management
- **Pre-detection Buffer**: Stores last 5 audio frames (~320ms at 16kHz)
- **Frame Size**: Porcupine-specific frame size (typically 512 samples)
- **Sample Rate**: Fixed 16kHz requirement for Porcupine engine
- **Buffer Compensation**: Includes pre-detection audio to capture start of speech

## Performance Optimization

### Sensitivity Tuning
```bash
# More sensitive (more false positives, fewer misses)
--detector_sensitivity 0.3

# Balanced (recommended starting point)
--detector_sensitivity 0.6

# Less sensitive (fewer false positives, more misses)
--detector_sensitivity 0.9
```

### CPU Usage Optimization
- **Single Threading**: Porcupine runs in single thread for low resource usage
- **Frame-based Processing**: Efficient frame-by-frame audio processing
- **Minimal Buffering**: Small audio buffers reduce memory footprint

### Network Bandwidth
- **Selective Streaming**: Only streams audio after wake word detection
- **YARP Efficiency**: Uses YARP's efficient audio transport
- **Buffer Optimization**: Minimal pre-detection buffering

## Troubleshooting

### Common Issues

1. **Module Won't Start**:
   ```bash
   # Check Picovoice access key
   echo $PICOVOICE_ACCESS_KEY

   # Verify keyword file exists
   ls -la /path/to/your_wake_word.ppn

   # Check YARP server
   yarp detect
   ```

2. **Porcupine Initialization Failed**:
   ```bash
   # Check access key validity
   # Visit https://console.picovoice.ai/ to verify your key

   # Check keyword file format
   file /path/to/your_wake_word.ppn

   # Verify model file exists
   ls -la porcupine/lib/common/porcupine_params.pv
   ```

3. **Wake Word Not Detected**:
   ```bash
   # Lower sensitivity threshold
   ./wakeWordDetection --detector_sensitivity 0.3

   # Check audio input format (must be 16kHz)
   yarp read ... /wake/audio:i

   # Verify wake word pronunciation matches training
   ```

4. **Build Errors - Porcupine Download Failed**:
   ```bash
   # Check internet connection
   wget https://github.com/Picovoice/porcupine.git

   # Clear CMake cache and retry
   rm -rf CMakeCache.txt CMakeFiles/
   cmake ..
   ```

5. **Audio Streaming Not Working**:
   ```bash
   # Check if module is in streaming mode
   yarp info /wake/audio:o

   # Send stop command to reset
   echo "stop" | yarp rpc /wake/rpc:i

   # Verify audio connections
   yarp info /wake/audio:i
   ```

6. **Interface Errors**:
   ```bash
   # Check WakeMsgs interface is built
   find /usr/local -name "*WakeMsgs*" 2>/dev/null

   # Rebuild interfaces if missing
   cd /path/to/tour-guide-robot/interfaces
   mkdir build && cd build && cmake .. && make && make install
   ```

### Debug Mode
Enable debug output for troubleshooting:

```bash
export YARP_VERBOSE=1
./wakeWordDetection --from wake_config.ini
```

### Audio Format Verification
```bash
# Check input audio properties
yarp read ... /wake/audio:i | head -10

# Verify 16kHz sampling rate
# Audio must be exactly 16kHz for Porcupine to work
```

### Model Validation
```bash
# Test with different wake word models
./wakeWordDetection --keyword_path /path/to/alternative_model.ppn

# Verify model file integrity
md5sum /path/to/your_wake_word.ppn
```

## Advanced Usage

### Multiple Wake Words
Currently supports single wake word. For multiple wake words:

```cpp
// Modify AudioCallback constructor to accept multiple keywords
const char* keywords[] = {"wake_word_1.ppn", "wake_word_2.ppn"};
float sensitivities[] = {0.6f, 0.7f};
pv_porcupine_init(accessKey.c_str(), modelPath.c_str(), 2, keywords, sensitivities, &m_porcupine);
```

### Custom Visual Feedback
```cpp
// Customize eye color changes in AudioCallback::colorEyes()
bool AudioCallback::colorEyes(int r, int g, int b) {
    // Default: Cyan (0, 255, 255) for wake word detection
    // Customize colors for different wake words or states
}
```

### Integration Scripts
```python
#!/usr/bin/env python3
# wake_word_monitor.py - Monitor wake word detection events

import yarp
import time

def monitor_wake_word_events():
    yarp.Network.init()

    # Monitor notification port
    port = yarp.BufferedPortBottle()
    port.open("/monitor")
    yarp.Network.connect("/wake/notification:o", "/monitor")

    print("Monitoring wake word detections...")
    while True:
        bottle = port.read(False)
        if bottle:
            command = bottle.get(0).asString()
            if command == "play_sound":
                print(f"Wake word detected at {time.strftime('%H:%M:%S')}")
        time.sleep(0.1)

if __name__ == "__main__":
    monitor_wake_word_events()
```

### Performance Monitoring
```bash
# Monitor CPU usage
top -p `pgrep wakeWordDetection`

# Monitor memory usage
ps -o pid,vsz,rss,comm -p `pgrep wakeWordDetection`

# Monitor YARP port activity
yarp info /wake/audio:i
yarp info /wake/audio:o
```

## Safety and Security Considerations

- **Access Key Security**: Store Picovoice access keys securely, avoid hardcoding
- **Model Validation**: Verify wake word model files are legitimate and unmodified
- **Audio Privacy**: Wake word detection runs locally, no audio sent to cloud
- **Resource Limits**: Monitor CPU/memory usage for long-running deployments
- **Network Security**: Secure YARP network communications in production environments

## Extension and Customization

### Custom Wake Word Models
1. **Design Wake Word**: Choose unique, easily pronounceable phrases
2. **Create via Console**: Use Picovoice Console to generate custom models
3. **Test Different Sensitivities**: Optimize for your acoustic environment
4. **Multiple Model Support**: Extend code to support multiple concurrent wake words

### Integration with Cloud Services
```cpp
// Send wake word events to cloud logging
void AudioCallback::sendNotification() {
    // Local notification
    yarp::os::Bottle bot;
    bot.addString("play_sound");
    m_notification_out.write(bot);

    // Cloud logging (example)
    logWakeWordEvent(getCurrentTimestamp());
}
```

### Custom Audio Processing
```cpp
// Pre-process audio before wake word detection
void AudioCallback::preprocessAudio(yarp::sig::Sound& sound) {
    // Apply noise reduction, gain control, etc.
    applyNoiseReduction(sound);
    normalizeGain(sound);
}
```
