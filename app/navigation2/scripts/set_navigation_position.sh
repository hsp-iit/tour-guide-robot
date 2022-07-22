#!/bin/bash
echo "set pos 0 -9.0" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 1 9.0" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 2 -10.0" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 3 50.0" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 4 0.0" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 5 0.0" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 6 0.0" | yarp write ... /cer/right_arm/rpc:i
echo "set pos 7 0.0" | yarp write ... /cer/right_arm/rpc:i

echo "set pos 0 -9.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 1 9.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 2 -10.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 3 50.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 4 0.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 5 0.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 6 0.0" | yarp write ... /cer/left_arm/rpc:i
echo "set pos 7 0.0" | yarp write ... /cer/left_arm/rpc:i

echo "set pos 0 0.012" | yarp write ... /cer/torso/rpc:i
