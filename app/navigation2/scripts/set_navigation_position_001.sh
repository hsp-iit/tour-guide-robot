#!/bin/bash
echo "set pos 0 -9.29" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 1 10.78" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 2 -10.19" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 3 35.09" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 4 0.0" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 5 0.03" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 6 0.0" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 7 0.0" | yarp write ... /cer/right_arm/rpc:i

echo "set pos 0 -9.29" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 1 10.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 2 -10.19" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 3 35.09" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 4 0.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 5 0.03" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 6 0.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 7 0.0" | yarp write ... /cer/left_arm/rpc:i

echo "set pos 0 0.036" | yarp write ... /cer/torso/rpc:i
