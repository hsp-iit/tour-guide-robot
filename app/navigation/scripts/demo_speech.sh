#!/bin/bash
echo "setLanguage \"en-US\" \"en-US-Wavenet-C\"" | yarp rpc /googleSynthesis/rpc
echo "say \"Hello everyone. Should we start the demo?\"" | yarp rpc /googleSynthesis/rpc
