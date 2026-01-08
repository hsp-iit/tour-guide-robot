# Documentation
This folder contains applications that allow R1 to dialog with users in the CONVINCE project Use Case 3. For more information about the project, please refer to the [UC3 repository](https://github.com/convince-project/UC3).

## Description for convince_new_dialog_VAD_test
Starts the convince dialog pipeline with VAD for execution in simulation.

| Module | Parameters  | Node | Description |
| :- | :------ | :- | :----------- |
| `yarprobotinterface` | --context speechPipeline --from micAudio.ini --mic_sound_on_stop 0 --mic_start 1 --mic_min_samples 4000 --mic_max_samples 4000 | console-llm | Starts the speech pipeline with microphone input. |
| `python3` | oww.py | console-llm | Runs the OpenWakeWord detection script. |
| `sileroVAD` |  | console-llm | Voice Activity Detection using Silero models. |
| `python3` | play_notification.py | console-llm | Plays a notification sound. |
| `yarprobotinterface` | --context vadModule --from audioPlayer.ini | console-llm | Starts the audio player module. |
| `yarprobotinterface` | --context convince --from madamaChat.ini | console-llm | Starts the convince module with madamaChat configuration. |
| `yarprobotinterface` | --context convince --from poiMadamaChat.ini | console-llm | Starts the convince module with poiMadamaChat configuration. |
| `yarprobotinterface` | --context convince --from welcomeTalkChat.ini | console-llm | Starts the convince module with welcomeTalkChat configuration. |
| `yarprobotinterface` | --context speechPipeline --from azureSynthesizer.ini | console-llm | Starts the speech pipeline with Azure synthesizer. |
| `yarprobotinterface` | --context speechPipeline --from azureTranscription.ini | console-llm | Starts the speech pipeline with Azure transcription. |
| `ros2_scheduler_component` | run scheduler_component scheduler_component conf/tours-with-italian-dates-in-chars.json TOUR_MADAMA_3 | bt |  |
| `ros2_speech_to_text_component` | run speech_to_text_component speech_to_text_component | bt | Runs the ROS2 speech to text component. |
| `ros2_text_to_speech_component` | run text_to_speech_component text_to_speech_component --from text_to_speech_config.ini | bt | Runs the ROS2 text to speech component. |
| `yarpActionsPlayer` | --filename configuration.ini --execute | bt | Executes the YARP actions player. |
| `ros2_dance_component` | run dance_component dance_component movements.json | bt |  |
| `ros2_execute_dance_component` | run execute_dance_component execute_dance_component | bt |  |
| `ros2_cartesian_pointing_component` | run cartesian_pointing_component cartesian_pointing_component --context r1_cartesian_control --artworks yarp-contexts/contexts/r1_cartesian_control/artwork_coords.json | bt |  |
| `ros2_cpp_dialog_component` | run dialog_component dialog_component --from config.ini | bt | Runs the ROS2 C++ dialog component. |
| `ros2_py_dialog_component` | run py_interaction_cliserv service | bt | Runs the ROS2 Python dialog component. |
| `ros2_dialog_skill` | run dialog_skill dialog_skill | bt | Runs the ROS2 dialog skill component. |

## Description for convince_new_dialog_test
Starts the convince dialog pipeline for execution in simulation.

| Module | Parameters  | Node | Description |
| :- | :------ | :- | :----------- |
| `yarprobotinterface` | --context speechPipeline --from micAudio.ini --mic_sound_on_stop 1 --mic_start 0 --mic_min_samples 4000 --mic_max_samples 4000 | console |  |
| `yarpaudiocontrolgui` | --local /controlMic:rpc --remote-rec /audioRecorder_nws | console-llm |  |
| `yarprobotinterface` | --context vadModule --from audioPlayer.ini | console | Starts the audio player module. |
| `yarprobotinterface` | --context convince --from madamaChat.ini | console-llm | Starts the convince module with madamaChat configuration. |
| `yarprobotinterface` | --context convince --from poiMadamaChat.ini | console-llm | Starts the convince module with poiMadamaChat configuration. |
| `yarprobotinterface` | --context convince --from welcomeTalkChat.ini | console-llm | Starts the convince module with welcomeTalkChat configuration. |
| `yarprobotinterface` | --context google --from googleSynthesizer.ini | console-llm |  |
| `yarprobotinterface` | --context google --from googleTranscription.ini | console-llm |  |
| `ros2_scheduler_component` | run scheduler_component scheduler_component conf/tours.json TOUR_MADAMA_3 | bt | Runs the ROS2 scheduler component. |

## Description for convince_new_dialog_VAD_robot
Starts the convince dialog pipeline with VAD for execution on the robot.

| Module | Parameters  | Node | Description |
| :- | :------ | :- | :----------- |
| `yarprobotinterface` | --context speechPipeline --from micAudio.ini --mic_sound_on_stop 0 --mic_start 1 --mic_min_samples 4000 --mic_max_samples 4000 | r1-torso | Starts the speech pipeline with microphone input. |
| `python3` | oww.py | console-llm | Runs the OpenWakeWord detection script. |
| `sileroVAD` |  | console-llm | Voice Activity Detection using Silero models. |
| `python3` | play_notification.py | r1-face | Plays a notification sound. |
| `yarprobotinterface` | --context vadModule --from audioPlayer.ini | r1-face | Starts the audio player module. |
| `yarprobotinterface` | --context convince --from madamaChat.ini | console-llm | Starts the convince module with madamaChat configuration. |
| `yarprobotinterface` | --context convince --from poiMadamaChat.ini | console-llm | Starts the convince module with poiMadamaChat configuration. |
| `yarprobotinterface` | --context convince --from welcomeTalkChat.ini | console-llm | Starts the convince module with welcomeTalkChat configuration. |
| `yarprobotinterface` | --context speechPipeline --from azureSynthesizer.ini | console-llm | Starts the speech pipeline with Azure synthesizer. |
| `yarprobotinterface` | --context speechPipeline --from azureTranscription.ini | console-llm | Starts the speech pipeline with Azure transcription. |
| `ros2_scheduler_component` | run scheduler_component scheduler_component conf/tours-with-italian-dates-in-chars.json TOUR_MADAMA_3 | bt |  |
| `ros2_speech_to_text_component` | run speech_to_text_component speech_to_text_component | bt | Runs the ROS2 speech to text component. |
| `ros2_text_to_speech_component` | run text_to_speech_component text_to_speech_component --from text_to_speech_config.ini | bt | Runs the ROS2 text to speech component. |
| `yarpActionsPlayer` | --filename configuration.ini --execute | bt | Executes the YARP actions player. |
| `ros2_dance_component` | run dance_component dance_component movements.json | bt |  |
| `ros2_execute_dance_component` | run execute_dance_component execute_dance_component | bt |  |
| `ros2_cartesian_pointing_component` | run cartesian_pointing_component cartesian_pointing_component --context r1_cartesian_control --artworks yarp-contexts/contexts/r1_cartesian_control/artwork_coords.json | bt |  |
| `ros2_cpp_dialog_component` | run dialog_component dialog_component --from config.ini | bt | Runs the ROS2 C++ dialog component. |
| `ros2_py_dialog_component` | run py_interaction_cliserv service | bt | Runs the ROS2 Python dialog component. |
| `ros2_dialog_skill` | run dialog_skill dialog_skill | bt | Runs the ROS2 dialog skill component. |

