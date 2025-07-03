#!/bin/bash
echo "set pos 0 16.0511" | yarp write ... /r1mk3Sim/right_arm/rpc:i
echo "set pos 1 10.00549318" | yarp write ... /r1mk3Sim/right_arm/rpc:i
echo "set pos 2 14.2878" | yarp write ... /r1mk3Sim/right_arm/rpc:i
echo "set pos 3 61.5511" | yarp write ... /r1mk3Sim/right_arm/rpc:i
echo "set pos 4 0.0109864" | yarp write ... /r1mk3Sim/right_arm/rpc:i
echo "set pos 5 0.0329591" | yarp write ... /r1mk3Sim/right_arm/rpc:i
echo "set pos 6 0.0" | yarp write ... /r1mk3Sim/right_arm/rpc:i
echo "set pos 7 0.0" | yarp write ... /r1mk3Sim/right_arm/rpc:i

echo "set pos 0 16.0511" | yarp write ... /r1mk3Sim/left_arm/rpc:i
echo "set pos 1 10.00549318" | yarp write ... /r1mk3Sim/left_arm/rpc:i
echo "set pos 2 14.2878" | yarp write ... /r1mk3Sim/left_arm/rpc:i
echo "set pos 3 61.5511" | yarp write ... /r1mk3Sim/left_arm/rpc:i
echo "set pos 4 0.0109864" | yarp write ... /r1mk3Sim/left_arm/rpc:i
echo "set pos 5 0.0329591" | yarp write ... /r1mk3Sim/left_arm/rpc:i
echo "set pos 6 0.0" | yarp write ... /r1mk3Sim/left_arm/rpc:i
echo "set pos 7 0.0" | yarp write ... /r1mk3Sim/left_arm/rpc:i

echo "set pos 0 0.012" | yarp write ... /r1mk3Sim/torso/rpc:i

