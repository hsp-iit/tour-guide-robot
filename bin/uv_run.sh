#!/bin/bash

# This scripts wraps uv launch handling the SIGTERM signal
# In particular, the user can substitute the uv launch command in yarpmanager with this script.
# The syntax is the same but the full path to the launch_file is needed.
# Stopping the script via yarpmanager will correctly stop the nodes. Killing the script will leave the nodes dangling.


_term() {
	echo "Cleaning up the process"
	kill -2 -${group}
}

if [ "$#" -lt 1 ]
then
	echo "Usage uv_run.sh [package] <executable> [args]"
	exit -1
fi

trap _term SIGTERM SIGINT

setsid uv run $@ &

group=$!
echo "Group pid: $group"
child=$!
wait "$child"
