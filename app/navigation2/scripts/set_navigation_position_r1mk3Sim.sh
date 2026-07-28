#!/bin/bash

echo "positionMoveGroupRPC (0 1 2 3 4 5 6 7) (16.0511 9.1 14.2878 61.5511 0.0109864 0.0329591 0.0 0.0)" | yarp write ... /r1mk3Sim/right_arm/nws/rpc:i

echo "positionMoveGroupRPC (0 1 2 3 4 5 6 7) (16.0511 9.1 14.2878 61.5511 0.0109864 0.03 0.0 0.0)" | yarp write ... /r1mk3Sim/left_arm/nws/rpc:i

echo "positionMoveOneRPC 0 0.012" | yarp write ... /r1mk3Sim/torso/nws/rpc:i
