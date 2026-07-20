#!/bin/bash

echo "positionMoveGroupRPC (0 1 2 3 4 5 6 7) (16.0 9.0 14.0 61.0 0.01 0.03 0.0 0.0)" | yarp write ... /cer/right_arm/nws/rpc:i

echo "positionMoveGroupRPC (0 1 2 3 4 5 6 7) (16.0 9.0 14.0 61.0 0.01 0.03 0.0 0.0)" | yarp write ... /cer/left_arm/nws/rpc:i

echo "positionMoveOneRPC 0 0.012" | yarp write ... /cer/torso/nws/rpc:i
