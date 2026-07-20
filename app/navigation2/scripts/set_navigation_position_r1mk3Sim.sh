#!/bin/bash

echo "positionMoveGroupRPC (0 1 2 3 4 5 6 7) (16.0511 9.1 14.2878 61.5511 0.0109864 0.0329591 0.0 0.0)" | yarp write ... /cer/right_arm/nws/rpc:i

echo "positionMoveGroupRPC (0 1 2 3 4 5 6 7) (16.0511 9.1 14.2878 61.5511 0.0109864 0.03 0.0 0.0)" | yarp write ... /cer/left_arm/nws/rpc:i

echo "positionMoveOneRPC 0 0.012" | yarp write ... /cer/torso/nws/rpc:i
