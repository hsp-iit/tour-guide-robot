#include "WakeWordModule.h" 

#include <iostream>

bool WakeWordModule::configure(yarp::os::ResourceFinder &rf) {
    if (!m_audioPort.open(m_portNameIn))
    {
        std::cout << "cannot open port " << m_portNameIn;
        return false;
    } 
    m_audioPort.useCallback(*m_callback);
    
    m_rpcPort.open("/wake/rpc:i");

    m_rpcServer = std::make_unique<WakeServer>(m_callback);

    m_rpcServer->yarp().attachAsServer(m_rpcPort);

    return true;
}

double WakeWordModule::getPeriod()
{
    return m_period;
}

bool WakeWordModule::close()
{
    m_audioPort.close();
    std::cout << "Closing wake word port" << std::endl;
    return true;
}

bool WakeWordModule::updateModule()
{
    return true;
}