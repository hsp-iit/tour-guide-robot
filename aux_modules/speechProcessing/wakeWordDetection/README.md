# Wake Word Detection
Wake word detection with YARP and Pocupine.

Uses Porupine from Picovoice which expects input audio at 16 kHz.
Will stream audio out to a YARP port as soon as the keyword is detected and will continue to do so until it receives a message to stop.

Add lib and include from https://github.com/Picovoice/porcupine/tree/master to this directory.

You will need you own access key and keyword file which you can obtain by creating an account at https://picovoice.ai/.
