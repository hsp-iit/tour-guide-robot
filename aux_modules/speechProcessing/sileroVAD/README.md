# Silero VAD YARP
Voice activation detection in YARP using Silero Voice activation model.

## Setup
Download and extract ONNX runtime (https://github.com/microsoft/onnxruntime/releases), set environment variable "ONNX_PATH" to the extracted folder
Download silero onnx file from their repo (https://github.com/snakers4/silero-models/tree/master) under data

## Notes
There is an RPC port to change threshold and gap allowance (deal with pauses in speech)