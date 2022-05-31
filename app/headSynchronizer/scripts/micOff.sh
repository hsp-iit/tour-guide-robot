#!/bin/bash
echo "stopHearing" | yarp rpc /HeadSynchronizer/thrift:s
