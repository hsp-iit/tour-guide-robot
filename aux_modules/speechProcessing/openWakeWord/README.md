# OpenWakeWord Module

## Overview

The OpenWakeWord module provides custom wake word detection functionality for the tour-guide robot using the openWakeWord library. It enables the robot to listen for specific wake words/phrases and respond accordingly, supporting custom-trained models and real-time audio processing. The module integrates with YARP for audio streaming, face expression control, and notification systems, making it suitable for interactive robot applications.

## Key Features

- **Custom Wake Word Detection**: Support for custom-trained wake word models using openWakeWord
- **Real-time Audio Processing**: Low-latency audio processing with configurable buffer sizes
- **YARP Integration**: Seamless integration with YARP audio ports and RPC interfaces
- **Multi-Model Support**: Simultaneous detection of multiple wake words with individual thresholds
- **Visual Feedback**: Eye color changes and facial expressions on wake word detection
- **Audio Passthrough**: Optional audio streaming when not actively detecting
- **Model Training**: Complete pipeline for training custom wake word models
- **RPC Control**: Remote control interface for starting/stopping detection
- **Configurable Parameters**: Flexible configuration of detection thresholds and audio settings

## Technical Specifications

- **Audio Format**: 16kHz, 16-bit PCM audio input
- **Model Format**: ONNX models for cross-platform inference
- **Buffer Size**: Configurable (default: 1280 samples = 80ms at 16kHz)
- **Detection Method**: Neural network-based with configurable thresholds
- **Framework**: OpenWakeWord with ONNX runtime backend
- **Integration**: YARP-based communication and audio processing

## Dependencies

### System Dependencies
- **Python** (>= 3.11): Modern Python version with enhanced performance
- **CMake** (>= 3.1): Build system for C++ components integration
- **YARP** (Yet Another Robot Platform): Robot middleware and communication
  - Python bindings required for audio streaming
- **Git LFS**: Large file support for model downloads during training

### Python Dependencies
The module uses Python's built-in dependency management. Core dependencies:

- **openwakeword**: Main wake word detection library
- **numpy**: Numerical computing for audio processing
- **scipy**: Scientific computing for audio I/O operations
- **multiprocessing**: Parallel processing for model inference

### Ubuntu/Debian Installation

#### Basic System Dependencies
```bash
sudo apt-get update
sudo apt-get install cmake build-essential python3 python3-pip git git-lfs
sudo apt-get install portaudio19-dev python3-dev
```

#### YARP Installation

**Option 1: Using Conda (Recommended)**
```bash
# Install conda/miniconda if not already installed
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Create a new environment with YARP
conda create -n robotics-env python=3.11
conda activate robotics-env

# Install YARP from conda-forge
conda install -c conda-forge -c robotology yarp

# Install Python dependencies
pip install openwakeword numpy scipy
```

**Option 2: From Package Manager**
```bash
# Install YARP dependencies
sudo apt-get install libace-dev libeigen3-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5opengl5-dev

# Install YARP (if available in repositories)
sudo apt-get install libyarp-dev yarp python3-yarp

# Install Python dependencies
pip3 install openwakeword numpy scipy
```

**Option 3: Build YARP from Source**
```bash
# Install YARP dependencies
sudo apt-get install libace-dev libeigen3-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5opengl5-dev

# Clone and build YARP
git clone https://github.com/robotology/yarp.git
cd yarp
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DYARP_COMPILE_BINDINGS=ON -DCREATE_PYTHON=ON
make -j$(nproc)
sudo make install

# Set environment variables
echo 'export YARP_ROOT=/usr/local' >> ~/.bashrc
echo 'export PATH=$PATH:$YARP_ROOT/bin' >> ~/.bashrc
echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages' >> ~/.bashrc
source ~/.bashrc

# Install Python dependencies
pip3 install openwakeword numpy scipy
```

### Fedora/CentOS Installation

#### Basic System Dependencies
```bash
sudo yum install cmake gcc-c++ python3 python3-pip git git-lfs
sudo yum install portaudio-devel python3-devel
```

#### YARP Installation

**Option 1: Using Conda (Recommended)**
```bash
# Install conda/miniconda if not already installed
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Create a new environment with YARP
conda create -n robotics-env python=3.11
conda activate robotics-env

# Install YARP from conda-forge
conda install -c conda-forge -c robotology yarp

# Install Python dependencies
pip install openwakeword numpy scipy
```

**Option 2: Build YARP from Source**
```bash
# Install YARP dependencies
sudo yum install ace-devel eigen3-devel sqlite-devel tinyxml-devel qt5-qtbase-devel qt5-qtdeclarative-devel qt5-qtmultimedia-devel

# Clone and build YARP
git clone https://github.com/robotology/yarp.git
cd yarp
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DYARP_COMPILE_BINDINGS=ON -DCREATE_PYTHON=ON
make -j$(nproc)
sudo make install

# Set environment variables
echo 'export YARP_ROOT=/usr/local' >> ~/.bashrc
echo 'export PATH=$PATH:$YARP_ROOT/bin' >> ~/.bashrc
echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/site-packages' >> ~/.bashrc
source ~/.bashrc

# Install Python dependencies
pip3 install openwakeword numpy scipy
```

### Training Dependencies (Optional)
For custom model training, additional dependencies are required:

```bash
# Training-specific Python packages
pip install datasets scipy tqdm mutagen torchinfo torchmetrics speechbrain
pip install audiomentations torch-audiomentations acoustics onnx2tf onnx
pip install pronouncing deep-phonemizer piper-phonemize webrtcvad

# PyTorch (for model training)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
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
make -j$(nproc)
```

The Python script will be copied to the `build/bin` directory automatically.

### Standalone Usage
The module can be used directly without building:

1. Navigate to the openWakeWord directory:
```bash
cd /path/to/tour-guide-robot/aux_modules/speechProcessing/openWakeWord
```

2. Make the script executable:
```bash
chmod +x oww.py
```

3. Run directly:
```bash
python3 oww.py --models your_model.onnx
```

## Model Setup

### Using Pre-trained Models
Download pre-trained models from openWakeWord:

```bash
# Download default models (automatically done on first run)
python3 -c "import openwakeword; openwakeword.utils.download_models()"

# List available models
python3 -c "import openwakeword; print(openwakeword.utils.get_pretrained_model_names())"
```

### Training Custom Models

The module includes a Jupyter notebook for training custom wake word models:

#### Prerequisites for Training
```bash
# Install Jupyter and required packages
pip install jupyter datasets scipy tqdm mutagen torchinfo torchmetrics
pip install speechbrain audiomentations torch-audiomentations acoustics
pip install onnx2tf onnx pronouncing deep-phonemizer
pip install piper-phonemize webrtcvad torch torchvision torchaudio
```

#### Training Process
1. Open the training notebook:
```bash
jupyter notebook automatic_model_training_simple.ipynb
```

2. **Step 1**: Test pronunciation of your target wake word
   - Enter your wake word in the text field
   - Use phonetic spelling with underscores if needed (e.g., "hey_seer_e" for "hey siri")
   - Run the cell to hear the generated audio

3. **Step 2**: Download training data (takes ~15 minutes)
   - Downloads background noise, music, and room impulse responses
   - Downloads pre-computed openWakeWord features for training

4. **Step 3**: Configure and train the model
   - Adjust training parameters:
     - `number_of_examples`: 1,000 (default) to 50,000 (recommended)
     - `number_of_training_steps`: 10,000 (default) to 50,000
     - `false_activation_penalty`: Controls false positive rate
   - Training takes 30-60 minutes on CPU, faster on GPU
   - Model files (`.onnx` and `.tflite`) are automatically downloaded

#### Example Custom Model Training
```python
# In the notebook, set your target wake word
target_word = 'hey_robot'  # Replace with your desired wake word

# Adjust training parameters for better accuracy
number_of_examples = 5000      # More examples = better accuracy
number_of_training_steps = 20000  # Longer training = better performance
false_activation_penalty = 2000   # Higher = fewer false positives
```

## Configuration

### Command Line Parameters
The module accepts various command-line arguments:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--models` | `./hey_r_one.onnx` | Paths to wake word model files (multiple supported) |
| `--framework` | `onnx` | Inference framework (onnx/tflite) |
| `--buffer-size` | `1280` | Audio buffer size in samples |
| `--sample-rate` | `16000` | Audio sample rate in Hz |
| `--thresholds` | `0.15` | Detection thresholds for each model |

### Configuration Examples

#### Single Model Setup
```bash
python3 oww.py --models hey_robot.onnx --thresholds 0.2
```

#### Multi-Model Setup
```bash
python3 oww.py --models hey_robot.onnx wake_up.onnx --thresholds 0.2 0.15
```

#### Custom Audio Settings
```bash
python3 oww.py --models custom_model.onnx --buffer-size 1600 --sample-rate 16000 --thresholds 0.25
```

## Usage

### Basic Execution
```bash
# Start YARP server
yarpserver &

# Run with default model
python3 oww.py

# Run with custom model
python3 oww.py --models my_wake_word.onnx --thresholds 0.2
```

### YARP Port Connections
The module automatically creates and uses these YARP ports:

#### Input Ports
- `/wake/audio:i`: Audio input from microphone or audio source
- `/wake/rpc:i`: RPC server for remote control commands

#### Output Ports
- `/wake/audio:o`: Audio passthrough when not detecting
- `/wake/notification:o`: Wake word detection notifications
- `/wake/face:o`: Facial expression control commands

### Connection Examples
```bash
# Connect audio source to wake word detector
yarp connect /audioSource:o /wake/audio:i

# Connect wake word notifications to tour manager
yarp connect /wake/notification:o /tourManager/wakeword:i

# Connect face control to face expression module
yarp connect /wake/face:o /faceExpression/commands:i

# Connect audio passthrough to speech recognition
yarp connect /wake/audio:o /speechRecognition/audio:i
```

### RPC Control Interface
Control the module via RPC commands:

```bash
# Connect to RPC interface
yarp rpc /wake/rpc:i

# Available commands:
>> stop    # Pause audio passthrough and start wake word detection
```

## Integration with Other Modules

### Robot Behavior Integration
The wake word detector integrates with several robot modules:

#### Face Expression Module
```bash
# Eye color changes on wake word detection
# White eyes: Normal operation
# Cyan eyes: Wake word detected

yarp connect /wake/face:o /faceExpression/notify:i
```

#### Tour Manager Integration
```bash
# Send wake word notifications to tour manager
yarp connect /wake/notification:o /tourManager/wake:i
```

#### Speech Pipeline Integration
```bash
# Route audio through wake word detector to speech recognition
yarp connect /microphone:o /wake/audio:i
yarp connect /wake/audio:o /speechRecognition/audio:i
```

### Module State Management
The module operates in two states:

1. **Passthrough Mode**: Audio is streamed through without processing
2. **Detection Mode**: Active wake word detection with visual feedback

## Performance Optimization

### Buffer Size Tuning
```bash
# Smaller buffers = lower latency, higher CPU usage
python3 oww.py --buffer-size 640   # 40ms latency

# Larger buffers = higher latency, lower CPU usage
python3 oww.py --buffer-size 2560  # 160ms latency
```

### Threshold Optimization
```bash
# Lower thresholds = more sensitive, more false positives
python3 oww.py --thresholds 0.1

# Higher thresholds = less sensitive, fewer false positives
python3 oww.py --thresholds 0.3
```

### Multi-Processing Architecture
The module uses separate processes for:
- **Audio Callback**: Real-time audio buffering (main thread)
- **Model Inference**: Wake word detection (separate process)
- **RPC Server**: Command handling (separate thread)

## Troubleshooting

### Common Issues

1. **Module Won't Start**:
   ```bash
   # Check Python and dependencies
   python3 --version
   python3 -c "import openwakeword, yarp, numpy"

   # Check YARP server
   yarp detect

   # Verify model files exist
   ls *.onnx *.tflite
   ```

2. **No Audio Input**:
   ```bash
   # Check audio source connection
   yarp exists /wake/audio:i
   yarp info /wake/audio:i

   # Test audio source
   yarp read ... /audioSource:o
   ```

3. **Wake Word Not Detected**:
   ```bash
   # Lower detection threshold
   python3 oww.py --thresholds 0.1

   # Check model file and pronunciation
   # Retrain with more examples if needed
   ```

4. **False Positives**:
   ```bash
   # Increase detection threshold
   python3 oww.py --thresholds 0.3

   # Retrain with higher false_activation_penalty
   ```

5. **Training Issues**:
   ```bash
   # Check disk space (training needs several GB)
   df -h

   # Verify internet connection for downloads
   wget -O /dev/null https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/embedding_model.onnx

   # Install missing training dependencies
   pip install datasets scipy tqdm onnx2tf
   ```

### Debug Mode
Enable verbose output for debugging:

```bash
# Add debug prints to see detection scores
export PYTHONUNBUFFERED=1
python3 oww.py --models your_model.onnx | tee debug.log
```

### Port Monitoring
```bash
# Monitor audio input
yarp read ... /wake/audio:i

# Monitor notifications
yarp read ... /wake/notification:o

# Monitor face commands
yarp read ... /wake/face:o
```

### Model Validation
```bash
# Test model with audio file
python3 -c "
import openwakeword
from openwakeword.model import Model
import numpy as np
import scipy.io.wavfile

# Load your model
model = Model(wakeword_models=['your_model.onnx'])

# Load test audio (16kHz, mono)
rate, audio = scipy.io.wavfile.read('test_audio.wav')
audio = audio.astype(np.float32) / 32768.0

# Run prediction
prediction = model.predict(audio)
print(f'Prediction scores: {prediction}')
"
```

## Advanced Usage

### Custom Model Development
1. **Data Collection**: Record diverse examples of your wake word
2. **Pronunciation Tuning**: Use phonetic spelling for better synthesis
3. **Parameter Optimization**: Experiment with training parameters
4. **Validation**: Test in various noise conditions

### Multi-Language Support
```bash
# Train models for different languages
python3 oww.py --models english_model.onnx italian_model.onnx spanish_model.onnx
```

### Integration Scripts
```bash
#!/bin/bash
# wake_word_setup.sh - Complete setup script

# Start YARP server
yarpserver &
sleep 2

# Start wake word detector
python3 oww.py --models hey_robot.onnx --thresholds 0.2 &
sleep 2

# Connect to audio source
yarp connect /microphone:o /wake/audio:i

# Connect to face expression
yarp connect /wake/face:o /faceExpression/notify:i

# Connect to tour manager
yarp connect /wake/notification:o /tourManager/wake:i

echo "Wake word detection system started"
```

## Model Training Best Practices

### Wake Word Selection
- **Length**: 2-4 syllables work best
- **Uniqueness**: Avoid common words to reduce false positives
- **Pronunciation**: Use clear, distinct sounds
- **Phonetics**: Spell phonetically if synthesis sounds wrong

### Training Parameters
- **Examples**: Start with 1,000, increase to 30,000-50,000 for production
- **Training Steps**: 10,000 minimum, 30,000-50,000 for best results
- **False Positive Penalty**: 1500-3000 for balanced performance

### Data Diversity
- **Background Noise**: Include various environments (office, street, home)
- **Music**: Add music samples to reduce musical false positives
- **Speakers**: Use diverse speaker voices during validation

## Safety Considerations

- **Privacy**: Wake word detection runs locally, no cloud processing
- **Performance**: Monitor CPU usage with multiple models
- **Audio Quality**: Ensure 16kHz sampling for best performance
- **False Positives**: Tune thresholds for your specific environment
- **Model Security**: Validate custom models before deployment

## Extension and Customization

### Custom Actions on Detection
```python
def on_wake_word_detected(model_name, confidence):
    # Custom behavior when wake word is detected
    print(f"Wake word '{model_name}' detected with confidence {confidence}")
    # Add your custom logic here
```

### Multiple Detection Strategies
```python
# Require multiple consecutive detections
consecutive_detections = 0
required_consecutive = 3

if prediction_score > threshold:
    consecutive_detections += 1
    if consecutive_detections >= required_consecutive:
        trigger_wake_word()
else:
    consecutive_detections = 0
```

### Integration with Speech Recognition
```python
# Start speech recognition after wake word detection
def start_speech_recognition():
    # Enable microphone for speech input
    speech_recognizer.start_listening()
    # Set timeout for speech input
    speech_timeout = 5.0
```