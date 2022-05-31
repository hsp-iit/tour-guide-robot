Face Expression Image:

Generates face image in 80x32 resolution for CER face display.

Feature:

The eyes will blink in with the following rate:
23 blinks (+/-3 random) per minute, meaning a random number of blinks between 20 and 26 per minute
15% of time the blink will be double

On command, the green bars at the left/right side of the face will change their size at random, with the
constraint of the inner bar being always bigger then outer bar.

How to use it:

Enable faceExpressionImage flag in the CMake, OpenCV is a required dependency.

run the module using 'faceExpressionImage' command.
Connect it with the input port of th faceDisplayServer on the robot.
A yarp manager xml template is provided for example.

The sizes and position of the bars can be set from configuration file.
An example of config file is provided as well in the app folder
