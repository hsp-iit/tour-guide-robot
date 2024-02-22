#ifndef WAKEWORD_H
#define WAKEWORD_H
#include <string>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <memory>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/sig/Sound.h>

#include "AudioCallback.h"

#include "WakeServer.h"

#define TIMEOUT_MAX 100

class WakeWordModule : public yarp::os::RFModule
{
private:
    std::shared_ptr<AudioCallback> m_callback;
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioPortIn;
    std::unique_ptr<WakeServer> m_rpcServer;
    yarp::os::RpcServer m_rpcPort;

    bool configure(yarp::os::ResourceFinder &rf) override;
    bool close() override;
    bool updateModule() override;
};

#endif