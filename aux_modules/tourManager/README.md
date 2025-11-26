# Tour Manager Module

## Overview

The Tour Manager is the central orchestration system for the tour-guide robot, responsible for managing guided tours through Points of Interest (PoIs). It coordinates speech synthesis, dance movements, navigation, and dialogue interactions to provide engaging and interactive tour experiences. The module serves as the primary state machine that controls the robot's behavior during tours, handling multi-language support, error recovery, and complex action sequences.

## Key Features

- **Tour Orchestration**: Complete management of guided tours with configurable Points of Interest
- **Multi-language Support**: Dynamic language switching with voice synthesis configuration
- **Action Coordination**: Synchronized execution of speech, dance, and navigation actions
- **Speech Integration**: Real-time speech synthesis, recognition, and dialogue processing
- **Dance Choreography**: Complex movement sequences with timing synchronization
- **Navigation Control**: Autonomous navigation between tour locations with error handling
- **Dialogue Management**: Integration with Google Dialogflow for interactive conversations
- **Error Recovery**: Comprehensive error handling for network, motor, and navigation failures
- **JSON Configuration**: Flexible tour and movement definition through JSON files
- **RPC Interface**: Remote procedure call interface for external control
- **State Persistence**: Maintains tour state and context across interactions

## Technical Specifications

- **Architecture**: YARP-based modular robot control system
- **Configuration**: JSON-based tour and movement definitions
- **Communication**: RPC interfaces and YARP port communications
- **Navigation**: Integration with Navigation2D stack for autonomous movement
- **Speech Processing**: Google Cloud Speech, Synthesis, and Dialogflow integration
- **Movement Control**: CTP (Cartesian Trajectory Planning) service integration
- **Threading**: Event-driven callback system with blocking/non-blocking action support

## Dependencies

### System Dependencies
- **CMake** (>= 3.16): Advanced build system for complex project structure
- **C++ Compiler**: Supporting C++11 standard with STL containers
- **YARP** (Yet Another Robot Platform): Robot middleware and communication
  - Components: `os`, `sig`, `dev`
- **nlohmann/json** (>= 3.10.5): Modern C++ JSON library for configuration parsing

### Ubuntu/Debian Installation

#### Basic Dependencies
```bash
sudo apt-get update
sudo apt-get install cmake build-essential pkg-config
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

# Install additional dependencies
conda install -c conda-forge nlohmann_json
```

**Option 2: From Package Manager**
```bash
# Install YARP dependencies
sudo apt-get install libace-dev libeigen3-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5opengl5-dev

# Install JSON library
sudo apt-get install nlohmann-json3-dev

# Install YARP (if available in repositories)
sudo apt-get install libyarp-dev yarp
```

**Option 3: Build from Source**
```bash
# Install YARP dependencies
sudo apt-get install libace-dev libeigen3-dev libsqlite3-dev libtinyxml-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5opengl5-dev

# Install JSON library
sudo apt-get install nlohmann-json3-dev

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
sudo yum install cmake gcc-c++ pkgconfig
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

# Install additional dependencies
conda install -c conda-forge nlohmann_json
```

**Option 2: Build from Source**
```bash
# Install YARP dependencies
sudo yum install ace-devel eigen3-devel sqlite-devel tinyxml-devel qt5-qtbase-devel qt5-qtdeclarative-devel qt5-qtmultimedia-devel

# Install JSON library (may need EPEL repository)
sudo yum install epel-release
sudo yum install json-devel

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

## Interface Dependencies

The Tour Manager requires several custom YARP interfaces (IDL files):

### Required Interfaces
- **headSynchronizerRPC**: Head movement and speech synchronization
- **google_speech**: Google Cloud Speech-to-Text interface
- **google_synthesis**: Google Cloud Text-to-Speech interface
- **google_dialog**: Google Dialogflow integration
- **tourManagerRPC**: Tour Manager's own RPC interface

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

3. Configure the build with all dependencies:
```bash
cmake ..
```

4. Build the project:
```bash
make -j$(nproc)
```

5. Install (optional):
```bash
make install
```

### Building Standalone
If you want to build only the tourManager:

1. Ensure all interface dependencies are built and installed first:
```bash
cd /path/to/tour-guide-robot/interfaces
mkdir build && cd build
cmake ..
make -j$(nproc)
make install
```

2. Navigate to the tourManager directory:
```bash
cd /path/to/tour-guide-robot/aux_modules/tourManager
```

3. Create and enter build directory:
```bash
mkdir build && cd build
```

4. Configure and build:
```bash
cmake ..
make -j$(nproc)
```

## JSON Configuration Rules and Structure

### JSON File Structure
The json tour file needs to have the following hierarchical structure:
- Tour
  - Languages for tour
    - PoI's in the tour for that language
      - Commands that this PoI in this language can understand
        - Actions that this command will take when called.

### Action Types
There are three types of Actions: Speak, Dance, Signal.

#### **Speak Action**
This action type makes the robot say the message that is in the params field of the Action. When the robot talks, the ears are automatically closed.

```json
{
  "m_type": "speak",
  "m_isBlocking": true,
  "m_param": "Welcome to our facility!"
}
```

#### **Dance Action**
This action type makes the robot execute the dance specified by name in the params field of the Action. The dance name is defined in another json file. A dance is made of the names of x movements (by name) and the total duration of the dance is calculated by accounting for the queuing the ctpService will do for same parts.

```json
{
  "m_type": "dance",
  "m_isBlocking": true,
  "m_param": "greeting_wave"
}
```

#### **Signal Action**
This is a special action type, as it executes specific logic in the code. The logic to execute is defined in the params field of the Action. **Signals are blocking and can't be changed!** Valid signals are:

- **startHearing**: Opens the ears of the robot. Usually used after we use a speak action to make the robot listen after a question.
- **setLanguage**: Changes the language of the system. It modifies the language of the PoIs loaded in the tour as well as the synthesizer, dialogueflow etc. The signal is a prefix and the suffix of that signal defines the language and the voice e.x. `setLanguage_en-US-Wavenet-C`
- **nextPoi**: Moves the index of the current PoI to the next or loops to the start if at the end, and updates the current PoI object from the Tour class.
- **reset**: Resets the index of the current PoI to zero, and reloads the PoI object from the Tour class.
- **delay_x**: Can be used to delay the execution of the command/action sequence at any place. The variable x is in seconds and can be a float.

```json
{
  "m_type": "signal",
  "m_isBlocking": false,
  "m_param": "startHearing"
}
```

### Special Configuration Considerations

#### **Fallback Handling**
The special command "fallback" is used to notify the users that the robot did not understand what they said. Typically, the command fallback should have one action, speak. The speak actions just tells the user that the robot could not understand what it just hears. Normally, the next action should be to **open the ears** so that the user can retry with a different text, but the **fallback does that automatically**. Moreover, if the fallback has been triggered consecutively more than a predefined threshold, then it will **automatically repeat the last action of type speak from the last non-fallback command**. That promotes the splitting of single speak actions to two speaks actions: explanation, question. In this case, only the question part will be repeated after the predefined failed attempts to understand.

#### **Command Variations**
A command can have multiple versions of itself. The format should add an index as a suffix to the variation. For example, you could have "greetings", "greetings1", "greetings2" etc. The default command should not have an index, and it is assumed to be 0. If there is more than one multiple of a command, the command to be executed is selected randomly from the variations using a uniform distribution.

#### **Action Blocking Behavior**
Every actions can be blocking or non blocking. In a list of actions inside a command, all actions are run in parallel (***in series with no delays***) if none of them is specified as blocking. The first one that is blocking, makes all be executed in "parallel" up to (including) the one that is blocking. It waits for **all** of the actions up to the blocking one to finish, and then proceeds to the others in the list with the same logic. For example if we have "b" for blocking and "n" for not blocking, then actions "n n b n b n n" will be executed first 3 in parallel waiting for all of them, then next 2 waiting, then last 2.

## Configuration Files

### Tours Configuration (`tours.json`)
Defines tour structure, Points of Interest, and available actions:

```json
{
  "TOUR_SIM_GAM": {
    "m_availablePoIs": {
      "it-IT": {
        "___generic___": {
          "m_name": "___generic___",
          "m_availableActions": {
            "fallback": [
              {
                "m_type": "speak",
                "m_isBlocking": true,
                "m_param": "Mi dispiace, non ho capito. Puoi ripetere?"
              }
            ],
            "startHearing": [
              {
                "m_type": "signal",
                "m_isBlocking": false,
                "m_param": "startHearing"
              }
            ]
          }
        },
        "poi_entrance": {
          "m_name": "poi_entrance",
          "m_availableActions": {
            "greeting": [
              {
                "m_type": "speak",
                "m_isBlocking": true,
                "m_param": "Benvenuti alla nostra struttura!"
              },
              {
                "m_type": "dance",
                "m_isBlocking": true,
                "m_param": "welcome_dance"
              }
            ]
          }
        }
      }
    },
    "m_activeTourPoIs": ["poi_entrance", "poi_main_hall", "poi_exit"]
  }
}
```

### Movements Configuration (`movements.json`)
Defines robot movements and dance choreographies:

```json
{
  "m_partNames": ["head", "torso", "left_arm", "right_arm"],
  "m_dances": {
    "welcome_dance": {
      "m_movements": [
        {
          "m_time": 2.0,
          "m_offset": 0.0,
          "m_partName": "head",
          "m_joints": [0.0, -10.0]
        },
        {
          "m_time": 1.5,
          "m_offset": 0.5,
          "m_partName": "torso",
          "m_joints": [0.0, 0.0, 5.0]
        }
      ]
    }
  }
}
```

### Runtime Parameters
The Tour Manager accepts several command-line parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `name` | "TourManager" | Module instance name |
| `nameJSONTours` | "tours.json" | Tours configuration file name |
| `nameJSONMovements` | "movements.json" | Movements configuration file name |
| `tourName` | "TOUR_SIM_GAM" | Active tour name to load |

### Navigation Configuration
Navigation settings can be configured via ResourceFinder:

```ini
[NAVIGATION2D-CLIENT]
device                navigation2D_nwc_yarp
local-suffix          /navClient
navigation_server     /navigation2D_nws_yarp
map_locations_server  /map2D_nws_yarp
localization_server   /localization2D_nws_yarp
```

## Usage

### Basic Execution
```bash
# Start YARP server
yarpserver &

# Run Tour Manager with default configuration
./tourManager

# Run with custom parameters
./tourManager --name MyTourManager --tourName CUSTOM_TOUR --nameJSONTours my_tours.json
```

### Required Service Dependencies
Before running Tour Manager, ensure these services are running:

#### Google Cloud Services
```bash
# Start Google Speech service
googleSpeech &

# Start Google Synthesis service
googleSynthesis &

# Start Google Dialog service
googleDialog &
```

#### Robot Control Services
```bash
# Start Head Synchronizer
HeadSynchronizer &

# Start CTP (Cartesian Trajectory Planning) services
ctpService --part head &
ctpService --part torso &
ctpService --part left_arm &
ctpService --part right_arm &
```

#### Navigation Services
```bash
# Start Navigation2D services
navigation2D_nws_yarp &
map2D_nws_yarp &
localization2D_nws_yarp &
```

### YARP Port Connections
Tour Manager automatically establishes these connections:

```bash
# Dialogue connections
yarp connect /googleDialog/result:o /TourManager/googleDialogInput
yarp connect /TourManager/dialogDialogOutput /googleDialog/text:i

# Speech and synthesis connections
yarp connect /TourManager/text:o /HeadSynchronizer/thrift:s
yarp connect /TourManager/speech/rpc /googleSpeech/rpc
yarp connect /TourManager/synthesis/rpc /googleSynthesis/rpc
yarp connect /TourManager/dialog/rpc /googleDialog/rpc

# Movement control connections
yarp connect /TourManager/head/rpc /ctpservice/head/rpc
yarp connect /TourManager/torso/rpc /ctpservice/torso/rpc
# ... (additional robot parts)
```

### RPC Interface Usage
The Tour Manager provides an RPC interface for external control:

```bash
# Connect to Tour Manager RPC
yarp rpc /TourManager/thrift:s

# Available RPC commands:
>> sendError "NETWORK_ERROR"       # Trigger error handling
>> recovered                       # Signal recovery from error
>> isAtPoI                        # Check if robot is at current PoI
>> sendToPoI                      # Navigate to current PoI
>> getCurrentPoIName              # Get current Point of Interest name
```

## Tour Flow Management

### Tour States
- **Initialization**: Load tour and movement configurations
- **Navigation**: Move between Points of Interest
- **Presentation**: Execute PoI-specific actions (speech, dance, etc.)
- **Interaction**: Handle user questions and responses
- **Error Handling**: Manage various error conditions
- **Recovery**: Return to normal operation after errors

### Point of Interest Types
- **Generic PoI** (`___generic___`): Universal commands available everywhere
- **Start PoI** (`*_start`): Initial tour locations with special behaviors
- **Regular PoIs**: Standard tour stops with specific content
- **Navigation PoIs**: Intermediate waypoints for complex routes

### Language Support
- **Dynamic Switching**: Real-time language changes during tours
- **Voice Configuration**: Language-specific synthesis voices
- **Content Localization**: PoI actions defined per language
- **Fallback Handling**: Error messages in appropriate language

## Error Handling and Recovery

### Error Types
The Tour Manager handles various error conditions:

| Error Type | Trigger | Response |
|------------|---------|----------|
| `NETWORK_ERROR` | Network connectivity issues | Network error message + sad face |
| `MOTORS_ERROR` | Motor system failures | Motor error message + stop navigation |
| `TOUCHED_ERROR` | Emergency stop activation | Touch error message + halt operations |
| `LOCALIZATION_ERROR` | Robot position uncertainty | Localization error message + relocalization prompt |
| `GOAL_ERROR` | Navigation goal failures | Goal error message + retry mechanism |

### Recovery Mechanisms
1. **Speech Recovery**: Resume interrupted speech after errors
2. **Navigation Recovery**: Restart navigation after clearing obstacles
3. **State Recovery**: Restore tour context after system restart
4. **Fallback Responses**: Handle unrecognized commands gracefully

## Integration with Other Modules

### Dependencies on Other Modules
- **Head Synchronizer**: Coordinates speech and facial expressions
- **Google Services**: Provides speech recognition, synthesis, and dialogue
- **Navigation2D**: Handles autonomous robot movement
- **CTP Services**: Controls precise robot movements and choreography
- **Face Expression**: Manages facial animations during interactions

### Data Flow
```
User Speech → Google Speech → Dialogflow → Tour Manager → Actions
                                               ↓
Navigation Commands → Navigation2D → Robot Movement
                                               ↓
Dance Commands → CTP Services → Robot Movements
                                               ↓
Speech Commands → Head Synchronizer → Audio Output
```

## Troubleshooting

### Common Issues

1. **Module Won't Start**:
   ```bash
   # Check YARP server is running
   yarp detect

   # Verify JSON configuration files exist
   ls tours.json movements.json

   # Check interface libraries are built
   find /usr/local -name "*tourManagerRPC*" 2>/dev/null
   ```

2. **No Speech Output**:
   ```bash
   # Verify Google services are running
   yarp exists /googleSynthesis/rpc
   yarp exists /HeadSynchronizer/thrift:s

   # Check port connections
   yarp info /TourManager/text:o
   yarp info /TourManager/synthesis/rpc
   ```

3. **Navigation Failures**:
   ```bash
   # Check navigation services
   yarp exists /navigation2D_nws_yarp
   yarp exists /map2D_nws_yarp

   # Verify map locations are loaded
   yarp rpc /navigation2D_nws_yarp
   >> help
   ```

4. **Movement Not Working**:
   ```bash
   # Check CTP services for each robot part
   yarp exists /ctpservice/head/rpc
   yarp exists /ctpservice/torso/rpc

   # Verify movement JSON configuration
   cat movements.json | jq '.m_partNames'
   ```

5. **JSON Configuration Errors**:
   ```bash
   # Validate JSON syntax
   cat tours.json | jq '.'
   cat movements.json | jq '.'

   # Check tour name exists in configuration
   cat tours.json | jq 'keys[]'
   ```

### Debug Mode
Enable verbose logging for troubleshooting:

```bash
export YARP_VERBOSE=1
./tourManager --name DebugTourManager
```

### Port Monitoring
Monitor YARP communications:

```bash
# Monitor dialogue input
yarp read ... /TourManager/googleDialogInput

# Monitor navigation commands
yarp read ... /TourManager/navClient/status:o

# Monitor speech synthesis
yarp read ... /TourManager/synthesis/rpc
```

### Configuration Validation
```bash
# Verify tour configuration structure
cat tours.json | jq '.TOUR_SIM_GAM.m_activeTourPoIs'

# Check movement definitions
cat movements.json | jq '.m_dances | keys'

# Validate action types
cat tours.json | jq '.. | objects | select(has("m_type")) | .m_type' | sort | uniq
```

## Performance Considerations

- **Memory Usage**: JSON configurations loaded once at startup
- **CPU Load**: Event-driven architecture minimizes continuous processing
- **Network Bandwidth**: Efficient YARP communication protocols
- **Real-time Constraints**: Blocking/non-blocking action execution
- **Scalability**: Modular design supports complex tour configurations

## Safety Considerations

- **Emergency Stops**: Immediate halt on TOUCHED_ERROR signals
- **Navigation Safety**: Obstacle avoidance and path validation
- **Speech Interruption**: Cancel speech during emergency situations
- **State Consistency**: Maintain valid tour state across errors
- **Resource Cleanup**: Proper cleanup of YARP ports and connections

## Advanced Usage

### Custom Tour Development
1. **Define New PoIs**: Add locations to tours.json with specific actions
2. **Create Choreographies**: Define movement sequences in movements.json
3. **Configure Languages**: Add multi-language content for international tours
4. **Design Interactions**: Create complex dialogue flows with Dialogflow

### Multi-Robot Tours
```bash
# Start multiple Tour Manager instances
./tourManager --name TourManager1 --tourName TOUR_A &
./tourManager --name TourManager2 --tourName TOUR_B &

# Each instance operates independently with separate configurations
```

### Extension and Customization

#### Adding New Action Types
```cpp
// Extend ActionTypes enum in action.h
enum ActionTypes {
    SPEAK,
    DANCE,
    SIGNAL,
    CUSTOM_ACTION,  // New action type
    INVALID = -1
};
```

#### Custom PoI Behaviors
Define specialized PoI types with unique behaviors:

```json
{
  "special_poi_interactive": {
    "m_name": "special_poi_interactive",
    "m_availableActions": {
      "quiz_mode": [
        {
          "m_type": "speak",
          "m_isBlocking": true,
          "m_param": "Welcome to our interactive quiz!"
        }
      ]
    }
  }
}
```

