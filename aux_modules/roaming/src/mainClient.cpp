#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <RoamingClient.h>
#include <yarp/os/LogStream.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    RoamingClient roaming;
    yInfo() << "Configuring and starting module...";
    // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    if (!roaming.runModule(rf))
    {
        yError() << "Error module did not start!";
    }

    yDebug() << "Main returning...";
    return EXIT_SUCCESS;
}