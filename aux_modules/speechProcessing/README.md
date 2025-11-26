# Speech Processing Modules

## Overview

The Speech Processing modules provide audio processing and speech recognition capabilities for the tour-guide robot system. These modules work together to enable voice-activated interactions through a modular, YARP-based architecture.

## Pipeline Architecture

```
Audio Input → Wake Word Detection → Voice Activity Detection → Speech Recognition
     ↓              ↓                        ↓
Face Expression ← Visual Feedback    Audio Filtering
```

## Modules

### 1. [Messages](./messages/) - Communication Interfaces
YARP IDL message interfaces for inter-module communication using Apache Thrift.

[📖 View Messages Documentation](./messages/README.md)

### 2. [Wake Word Detection](./wakeWordDetection/) - Picovoice-Based Activation
Real-time wake word detection using Picovoice Porcupine engine with custom wake word support.

[📖 View Wake Word Detection Documentation](./wakeWordDetection/README.md)

### 3. [OpenWakeWord](./openWakeWord/) - Custom Wake Word Training
Custom wake word detection with trainable models and Python-based implementation.

[📖 View OpenWakeWord Documentation](./openWakeWord/README.md)

### 4. [Silero VAD](./sileroVAD/) - Voice Activity Detection
Real-time voice activity detection using the Silero VAD neural network model.

[📖 View Silero VAD Documentation](./sileroVAD/README.md)

### 5. Voice Activation Detection (voiceActivationDetection)
Traditional voice activity detection using libfvad (conditional compilation).

## Quick Start

### Installation
```bash
# Install YARP (recommended: conda)
conda create -n robotics-env
conda activate robotics-env
conda install -c conda-forge -c robotology yarp
```

### Build
```bash
cd /path/to/tour-guide-robot
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Basic Pipeline
```bash
# Start YARP server
yarpserver &

# Connect modules (example)
yarp connect /microphone:o /wake/audio:i
yarp connect /wake/audio:o /vad/audio:i
yarp connect /vad/audio:o /speechRecognition/audio:i
```

## Build Options

Configure specific modules during build:
```bash
cmake .. -DWAKE_WORD=ON -DSILERO_VAD=ON
```

- **WAKE_WORD**: Enable Picovoice wake word detection
- **SILERO_VAD**: Enable Silero voice activity detection
- **libfvad**: Auto-detected for traditional VAD

## Support

For module-specific issues, see individual documentation:
- [Messages troubleshooting](./messages/README.md#troubleshooting)
- [Wake Word troubleshooting](./wakeWordDetection/README.md#troubleshooting)
- [OpenWakeWord troubleshooting](./openWakeWord/README.md#troubleshooting)
- [Silero VAD troubleshooting](./sileroVAD/README.md#troubleshooting)