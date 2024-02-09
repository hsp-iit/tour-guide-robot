#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "WakeWordModule.h"

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
    // rf.configure(argc, argv);

    WakeWordModule speechModule;

    return speechModule.runModule(rf);
}