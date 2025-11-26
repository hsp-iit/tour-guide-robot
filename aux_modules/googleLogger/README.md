# Google Logger Module

## Overview

The Google Logger module is responsible for logging user interactions with Google Dialog services during robot tours. It captures user questions, Google's responses, and the current Point of Interest (PoI) where the interaction occurred. The module creates timestamped CSV files containing this interaction data for analysis and tour improvement purposes.

## Key Features

- **User Input Logging**: Captures what users say to the robot
- **Google Response Logging**: Records Google Dialog service responses
- **PoI Context Tracking**: Associates interactions with specific Points of Interest
- **CSV Output**: Generates timestamped CSV files for easy data analysis
- **Thread-Safe Operations**: Mutex-protected file writing and data access
- **Tour Integration**: Integrates with Tour Manager to get current location context

## Technical Specifications

- **Output Format**: CSV files with columns: Question, Answer, PoI_Name
- **File Naming**: `{filename}_{epoch_timestamp}.csv`
- **Thread Safety**: Mutex-protected shared resources
- **Update Period**: Configurable (default: 1.0 seconds)

## Dependencies

### System Dependencies
- **CMake** (>= 3.12): Build system
- **C++ Compiler**: Supporting C++11 standard
- **YARP** (Yet Another Robot Platform): Middleware for robot communication
  - Components needed: `os`, `dev`, `init`

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

The executable will be created in `build/bin/googleLogger`.

### Building Standalone
If you want to build only the googleLogger module:

1. Navigate to the googleLogger directory:
```bash
cd aux_modules/googleLogger
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

**Note**: For standalone building, you'll need to ensure that the `tourManagerRPC` interface is built first from the `interfaces/tourManagerRPC` directory.

## Configuration Parameters

The module accepts the following configuration parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | "googleLogger" | Module name (used for port naming) |
| `period` | double | 1.0 | Update period in seconds |
| `file_name` | string | "googleLogger" | Base name for output CSV files |

## Port Configuration

### Input Ports
| Port Name | Format | Description |
|-----------|--------|-------------|
| `/{name}/userInput:i` | `yarp::os::Bottle` | Receives user speech input |
| `/{name}/googleDialog:i` | `yarp::os::Bottle` | Receives Google Dialog responses (via RPC) |

### Output Ports
| Port Name | Format | Description |
|-----------|--------|-------------|
| `/{name}/TourManager/thrift:c` | RPC Client | Connects to Tour Manager for PoI information |

## Execution

### Prerequisites
Before running the googleLogger module, ensure that:

1. **YARP Server is running**:
   ```bash
   yarp server
   ```

2. **Tour Manager is running**: The module needs to connect to a Tour Manager service to get current PoI information

3. **Google Dialog Service**: A Google Dialog service should be available to provide responses

4. **User Input Source**: A source providing user speech input (e.g., speech recognition module)

### Running the Module

1. Make sure the necessary services are running (yarp server, tour manager, etc.)

2. Run the executable:
```bash
./googleLogger
```

### Running with Configuration File
Create a configuration file (e.g., `googleLogger.ini`):
```ini
name googleLogger
period 1.0
file_name tour_interactions
```

Then run:
```bash
./googleLogger --from googleLogger.ini
```

### Running with Command Line Parameters
```bash
./googleLogger --name googleLogger --period 1.0 --file_name tour_interactions
```

### YARP Port Information
The module creates the following YARP ports:
- `/{name}/userInput:i`: Input port for user speech data
- `/{name}/googleDialog:i`: RPC port for Google Dialog responses
- `/{name}/TourManager/thrift:c`: RPC client port for Tour Manager communication

## Output Data Format

The module generates CSV files with the following structure:

### CSV Header
```csv
Question,Answer,PoI_Name
```

### Example Data
```csv
Question,Answer,PoI_Name
"What is this painting about?","This is the Mona Lisa by Leonardo da Vinci...","Gallery Room 1"
"When was it painted?","The Mona Lisa was painted between 1503 and 1519","Gallery Room 1"
"Who is the artist?","The artist is Leonardo da Vinci, an Italian Renaissance master","Gallery Room 1"
```

### File Naming Convention
Files are automatically timestamped with Unix epoch time:
- Format: `{base_name}_{epoch_timestamp}.csv`
- Example: `googleLogger_1695387600.csv`

## How It Works

### Data Flow
1. **User Input Capture**: User speech is received via the `userInput` port
2. **Google Dialog Processing**: Google's response is received via RPC callback
3. **PoI Context Retrieval**: Current Point of Interest is fetched from Tour Manager
4. **Data Logging**: All three pieces of information are written to CSV file
5. **Thread Safety**: Mutex protection ensures data integrity during concurrent access

### Callback System
- **UserInputReceiver**: Callback class that captures and stores the latest user input
- **RPC Respond**: Processes Google Dialog responses and triggers data logging
- **Synchronization**: Thread-safe access to shared resources using mutex locks

### Logging Process
```cpp
// Simplified workflow
userInput = getUserInput();           // From speech recognition
googleResponse = getGoogleResponse(); // From Dialog service
currentPoI = getCurrentPoI();         // From Tour Manager
logToCSV(userInput, googleResponse, currentPoI);
```

## Integration with Tour System

### Required Services
- **Tour Manager**: Provides current PoI information via RPC
- **Google Dialog Service**: Provides AI-powered responses to user questions
- **Speech Recognition**: Converts user speech to text
- **Text-to-Speech**: Converts Google responses to robot speech (optional)

### Data Usage
The logged data can be used for:
- **Tour Analysis**: Understanding what users ask at different locations
- **Content Improvement**: Identifying gaps in tour information
- **User Experience Research**: Analyzing interaction patterns
- **Machine Learning**: Training better dialog systems

## Dependencies on Other Modules

This module requires:
- **YARP Server**: Must be running for inter-process communication
- **Tour Manager**: Required for getting current PoI context
- **Google Dialog Service**: Source of AI responses to user questions
- **Speech Recognition System**: Source of user input text

## Troubleshooting

### Common Issues

1. **YARP Connection errors**:
   - Ensure `yarp server` is running
   - Check if YARP namespace is correctly configured: `yarp namespace`
   - Verify ports are not already in use: `yarp name list`

2. **File Writing errors**:
   - Check write permissions in the working directory
   - Verify disk space availability
   - Ensure the specified filename is valid

3. **Tour Manager connection issues**:
   - Verify Tour Manager is running and accessible
   - Check RPC port connectivity: `yarp name query /{name}/TourManager/thrift:c`
   - Ensure tourManagerRPC interface is properly built

4. **Missing User Input**:
   - Verify speech recognition system is running and connected
   - Check input port connection: `yarp name query /{name}/userInput:i`
   - Monitor input data: `yarp read ... /{name}/userInput:i`

5. **Google Dialog Integration**:
   - Ensure Google Dialog service is accessible
   - Check RPC port for Dialog responses
   - Verify authentication and API credentials if required

6. **Build errors**:
   - Make sure YARP is correctly installed and in the system path
   - Verify that tourManagerRPC interface is built
   - Check that all YARP components (os, dev, init) are available

7. **Port binding errors**:
   - Check if ports are already in use
   - Use `yarp clean` to remove stale port registrations
   - Try running with a different module name: `--name googleLogger2`

### Debug Mode
The module uses YARP logging. Enable debug output by setting the appropriate log level:

```bash
export YARP_VERBOSE=1
./googleLogger
```

### YARP Network Diagnostics
```bash
# Check YARP server status
yarp detect --write

# List all active ports
yarp name list

# Check specific port connections
yarp name query /googleLogger/userInput:i
yarp name query /googleLogger/googleDialog:i

# Monitor port data
yarp read ... /googleLogger/userInput:i
```

### Data Validation
Check the generated CSV files:
```bash
# View recent log files
ls -la *.csv

# Check file contents
head -n 10 googleLogger_*.csv

# Count logged interactions
wc -l googleLogger_*.csv
```

## Performance Considerations

- **File I/O**: CSV writing is mutex-protected but may block during concurrent access
- **Memory Usage**: Minimal - only stores the latest user input in memory
- **Update Frequency**: Default 1Hz is usually sufficient for logging purposes
- **Thread Safety**: Mutex protection ensures data integrity but may cause brief blocking
- **Disk Usage**: CSV files grow with each interaction - implement log rotation if needed

## Customization

### Extending Data Fields
To add more fields to the CSV output, modify:
1. `FileWriter::configure()` - Update CSV header
2. `FileWriter::writeToFile()` - Add new parameters
3. `GoogleLogger::respond()` - Pass additional data

### Custom File Formats
The `FileWriter` class can be extended to support:
- JSON output format
- XML structured data
- Database storage
- Remote logging services

### Integration Examples
```cpp
// Example: Connect to speech recognition
yarp connect /speechRec/text:o /googleLogger/userInput:i

// Example: Connect to Google Dialog
yarp connect /googleDialog/rpc /googleLogger/googleDialog:i

// Example: Monitor CSV output
tail -f googleLogger_*.csv
```