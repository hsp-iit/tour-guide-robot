// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause


#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "VoiceActivationDetectionModule.h"

int main(int argc, char *argv[])
{

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    // TODO need to be checked
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc, argv);

    VoiceActivationDetectionModule speechModule;

    return speechModule.runModule(rf);
}
