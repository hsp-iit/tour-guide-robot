#include "SpeechToTextComponent.hpp"

int main(int argc, char* argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    SpeechToTextComponent speechToTextComponent;
    if (!speechToTextComponent.ConfigureYARP(rf)){
        yError() << "[main] Unable to configure YARP in SpeechToTextComponent";
        return 1;
    }
    yInfo() << "[SpeechToTextComponent main] Configured YARP";
    if (!speechToTextComponent.start(argc, argv)) {
        yError() << "[main] Unable to start SpeechToTextComponent";
        return 1;
    }
    yInfo() << "[SpeechToTextComponent main] started";
    // blocking call
    speechToTextComponent.spin();

    speechToTextComponent.close();
    return 0;
}