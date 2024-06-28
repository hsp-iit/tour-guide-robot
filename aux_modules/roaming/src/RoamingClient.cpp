#include <RoamingClient.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <algorithm>

RoamingClient::RoamingClient():
    roaming{WpaSupplicantRoaming()}
    {}

bool RoamingClient::configure(yarp::os::ResourceFinder& rf)
{
    m_name = rf.check("name",yarp::os::Value("RoamingClient")).asString();
    m_interface_name = rf.check("interface",yarp::os::Value("wlp0s20f3")).asString();
    m_ssid = rf.check("ssid",yarp::os::Value("r1_wifi")).asString();

    yDebug() << "Parameter name set to:" << m_name;
    yDebug() << "Parameter interface set to:" << m_interface_name;
    yDebug() << "Parameter ssid set to:" << m_ssid;

    m_port_ap_name = "/" + m_name + "/ap:i";
    m_port_ap.open(m_port_ap_name);
    
    return true;
}

bool RoamingClient::updateModule()
{
    yarp::os::Bottle ap_name_bot;
    if(m_port_ap.read(ap_name_bot))
    {
        std::string ap_name = ap_name_bot.toString();

        // Trimming the \" at the start and end of string introduced by the bottle
        ap_name.erase(std::remove(ap_name.begin(),ap_name.end(),'\"'),ap_name.end());
        if(!roaming.roam(m_interface_name,m_ssid,ap_name))
        {
            yError() << "Roaming to ap: " << ap_name << " has failed";
        }
    }

    return true;
}

bool RoamingClient::interruptModule()
{
    m_port_ap.interrupt();
    return true;
}

bool RoamingClient::close()
{
    m_port_ap.close();
    return true;
}

double RoamingClient::getPeriod()
{
    return 1;
}