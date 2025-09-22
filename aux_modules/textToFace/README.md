# Text to Face Module (Microphone Filter Plugin)

## Overview

The Text to Face module is a YARP port monitor plugin that provides audio channel filtering functionality for microphone input processing. It acts as a middleware component that can extract specific audio channels from multi-channel audio streams, making it particularly useful for processing stereo or multi-channel microphone inputs in robot applications where only specific channels need to be processed or forwarded.

## Key Features

- **Audio Channel Extraction**: Extracts single channel from multi-channel audio streams
- **YARP Port Monitor Plugin**: Integrates seamlessly with YARP's port monitoring system
- **Real-time Processing**: Low-latency audio filtering for live audio streams
- **Configurable Channel Selection**: Runtime configuration of target audio channel
- **Dynamic Plugin Loading**: Loaded automatically by YARP when needed
- **Memory Efficient**: In-place audio processing with minimal memory overhead

## Technical Specifications

- **Plugin Type**: YARP Port Monitor Plugin
- **Audio Format**: YARP Sound objects
- **Channel Support**: Multi-channel to single-channel extraction
- **Runtime Configuration**: Dynamic channel selection via YARP properties
- **Installation**: Dynamic plugin loading via YARP plugin system

## Dependencies

### System Dependencies
- **CMake** (>= 3.12): Build system
- **C++ Compiler**: Supporting C++11 standard
- **YARP** (Yet Another Robot Platform): Middleware for robot communication
  - Components needed: `os`, `sig`, `dev`

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

The plugin will be installed to the YARP plugin directory automatically.

### Building Standalone
If you want to build only the textToFace plugin:

1. Navigate to the textToFace directory:
```bash
cd aux_modules/textToFace
```

2. Create and enter build directory:
```bash
mkdir build && cd build
```

3. Configure and build:
```bash
cmake ..
make
make install
```

**Note**: The plugin must be installed to be discoverable by YARP. The installation path is automatically determined by YARP's plugin system.

## Plugin Configuration

### Runtime Parameters
The plugin accepts the following configuration parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `channel` | int8 | 0 | Audio channel to extract (0-based indexing) |
| `sender_side` | bool | false | Whether plugin is attached to sender or receiver side |

### YARP Plugin Properties
The plugin is registered with the following properties:
- **Type**: `MicrophoneFilter`
- **Category**: `portmonitor`
- **Library**: `yarp_pm_textToFace`

## Usage

### Port Monitor Setup
The plugin is used as a YARP port monitor. It can be applied to audio connections to filter specific channels:

#### Basic Usage
```bash
# Create connection with channel filter
yarp connect /audioSource:o /audioSink:i tcp+recv.portmonitor+type.textToFace+channel.0

# Or use parameters file
yarp connect /audioSource:o /audioSink:i tcp+recv.portmonitor+type.textToFace+file.filter.ini
```

#### Configuration File Example
Create a configuration file (e.g., `filter.ini`):
```ini
channel 1
```

Then use it in the connection:
```bash
yarp connect /audioSource:o /audioSink:i tcp+recv.portmonitor+type.textToFace+file.filter.ini
```

### Audio Channel Extraction
The plugin extracts a single channel from multi-channel audio:

```bash
# Extract left channel (channel 0) from stereo input
yarp connect /stereoMic:o /monoOutput:i tcp+recv.portmonitor+type.textToFace+channel.0

# Extract right channel (channel 1) from stereo input
yarp connect /stereoMic:o /monoOutput:i tcp+recv.portmonitor+type.textToFace+channel.1

# Extract specific channel from multi-channel array microphone
yarp connect /arrayMic:o /processedAudio:i tcp+recv.portmonitor+type.textToFace+channel.3
```

### Runtime Parameter Changes
You can change parameters at runtime using YARP's port monitor interface:

```bash
# Connect to the monitor port
yarp rpc /audioSink:i/monitor/rpc:o

# Change channel selection
>> set channel 2

# Get current parameters
>> get
```

## How It Works

### Audio Processing Pipeline
1. **Input Reception**: Receives multi-channel YARP Sound objects
2. **Channel Validation**: Verifies input is valid Sound data
3. **Channel Extraction**: Extracts specified channel using `extractChannelAsSound()`
4. **Output Preparation**: Packages single-channel audio for output
5. **Memory Management**: Efficient handling of audio buffers

### Plugin Lifecycle
```cpp
// Simplified workflow
create() -> setparam() -> accept() -> update() -> destroy()
```

#### Key Methods
- **`create()`**: Initialize plugin with configuration options
- **`accept()`**: Validate incoming audio data format
- **`update()`**: Perform channel extraction and return filtered audio
- **`setparam()`/`getparam()`**: Runtime parameter management
- **`destroy()`**: Cleanup resources

### Memory Management
- **Zero-copy Operation**: Extracts channels without full audio buffer duplication
- **Automatic Cleanup**: YARP handles memory management for audio buffers
- **Efficient Processing**: Minimal CPU overhead for channel extraction

## Integration Examples

### Stereo to Mono Conversion
```bash
# Convert stereo microphone input to mono for speech recognition
yarp connect /robot/microphone:o /speechRecognition/audio:i tcp+recv.portmonitor+type.textToFace+channel.0
```

### Multi-channel Array Processing
```bash
# Select front-facing microphone from 4-channel array
yarp connect /micArray:o /frontMic:i tcp+recv.portmonitor+type.textToFace+channel.2

# Process multiple channels in parallel
yarp connect /micArray:o /channel0:i tcp+recv.portmonitor+type.textToFace+channel.0
yarp connect /micArray:o /channel1:i tcp+recv.portmonitor+type.textToFace+channel.1
```

### Audio Recording Setup
```bash
# Record specific channel from multi-channel input
yarp connect /liveAudio:o /recorder/audio:i tcp+recv.portmonitor+type.textToFace+channel.1
```

## Dependencies on Other Modules

This plugin can be used with:
- **Audio Capture Devices**: Microphone arrays, stereo microphones
- **Speech Recognition Systems**: For mono audio input requirements
- **Audio Processing Modules**: As preprocessing step for audio analysis
- **Recording Systems**: For selective channel recording
- **Communication Systems**: For audio transmission optimization

## Troubleshooting

### Common Issues

1. **Plugin Not Found**:
   - Ensure plugin is properly installed: `yarp plugin list | grep textToFace`
   - Check YARP plugin path: `yarp config plugins`
   - Verify plugin installation directory

2. **Build Errors**:
   - Ensure YARP development packages are installed
   - Verify CMake can find YARP: `find_package(YARP ...)`
   - Check compiler C++11 support

3. **Audio Format Errors**:
   - Verify input is YARP Sound object, not Bottle or other type
   - Check audio source is producing valid multi-channel audio
   - Monitor input with: `yarp read ... /audioSource:o`

4. **Channel Index Errors**:
   - Ensure channel index is within audio stream channel count
   - Use 0-based indexing (first channel = 0)
   - Check input audio format and channel count

5. **Runtime Parameter Errors**:
   - Verify parameter names match plugin interface
   - Check parameter data types (channel should be int8)
   - Use YARP monitor RPC interface for runtime changes

### Debug Mode
Enable YARP logging to see plugin debug output:

```bash
export YARP_VERBOSE=1
# Run your application with audio connections
```

### Plugin Verification
```bash
# List available plugins
yarp plugin list

# Check specific plugin details
yarp plugin show textToFace

# Verify plugin installation
find /usr/local -name "*textToFace*" 2>/dev/null
```

### Connection Debugging
```bash
# Monitor plugin activity
yarp exists /audioSink:i/monitor/rpc:o

# Check connection details
yarp exists /audioSink:i
yarp info /audioSink:i

# Test audio flow
yarp read ... /audioSink:i
```

## Performance Considerations

- **CPU Overhead**: Minimal processing overhead for channel extraction
- **Memory Usage**: Single-channel output requires less memory than multi-channel
- **Network Bandwidth**: Reduced bandwidth for single-channel transmission
- **Latency**: Near-zero additional latency for real-time applications
- **Threading**: Plugin operates in YARP's port threading context

## Advanced Usage

### Custom Configuration Scripts
```bash
#!/bin/bash
# setup_audio_channels.sh

# Setup multiple filtered channels from array microphone
yarp connect /micArray:o /frontLeft:i tcp+recv.portmonitor+type.textToFace+channel.0
yarp connect /micArray:o /frontRight:i tcp+recv.portmonitor+type.textToFace+channel.1
yarp connect /micArray:o /rearLeft:i tcp+recv.portmonitor+type.textToFace+channel.2
yarp connect /micArray:o /rearRight:i tcp+recv.portmonitor+type.textToFace+channel.3

echo "Audio channels configured"
```

### Integration with Speech Processing
```cpp
// C++ example: Setup filtered audio connection
yarp::os::Network::connect("/robot/microphone:o",
                          "/speechProcessor/audio:i",
                          "tcp+recv.portmonitor+type.textToFace+channel.0");
```

### Batch Channel Processing
```bash
# Process all channels from 8-channel array
for i in {0..7}; do
  yarp connect /arrayMic:o /channel$i:i tcp+recv.portmonitor+type.textToFace+channel.$i &
done
wait
```

## Extension and Customization

### Adding Features
To extend the plugin functionality:

1. **Additional Filters**: Add frequency filtering, noise reduction
2. **Format Conversion**: Support for different audio formats
3. **Multi-channel Output**: Extract multiple channels simultaneously
4. **Audio Analysis**: Add volume level analysis, clipping detection

### Plugin Development
The plugin follows YARP's MonitorObject interface. Key extension points:

```cpp
// Add custom processing in update() method
yarp::os::Things& MicrophoneFilter::update(yarp::os::Things& thing) {
    // Custom audio processing here
    // Volume normalization, filtering, etc.
    return processedThing;
}
```

## Safety Considerations

- **Audio Latency**: Monitor latency for real-time applications
- **Resource Usage**: Plugin runs in port thread - avoid blocking operations
- **Error Handling**: Plugin failures can affect entire audio pipeline
- **Parameter Validation**: Invalid channel indices can cause crashes