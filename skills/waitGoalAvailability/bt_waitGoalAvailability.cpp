/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "bt_waitGoalAvailability.h"

YARP_LOG_COMPONENT(WAITGOALAVAILABILITY_ACTION, "behavior_tour_robot.skills.waitgoalavailability_action", yarp::os::Log::TraceType)

waitGoalAvailability::waitGoalAvailability(std::string name) : m_name(name),
                                                               m_stopped(false),
                                                               m_pBtName("/" + name + "/BT_rpc/server"),
                                                               m_tourManagerPortName("/"+name+"/TourManager/thrift:c"),
                                                               m_period(1)
{
}

bool waitGoalAvailability::configure(yarp::os::ResourceFinder &rf)
{
    // --------- navigation2D_nwc_yarp config --------- //
    bool okNav = rf.check("NAVIGATION2D-CLIENT");
    std::string device = "navigation2D_nwc_yarp";
    std::string local = "/" + m_name + "/navClient";
    std::string navServer = "/navigation2D_nws_yarp";
    std::string mapServer = "/map2D_nws_yarp";
    std::string locServer = "/localization2D_nws_yarp";
    if (okNav)
    {
        yarp::os::Searchable &nav_config = rf.findGroup("NAVIGATION2D-CLIENT");
        if (nav_config.check("device"))
        {
            device = nav_config.find("device").asString();
        }
        if (nav_config.check("local-suffix"))
        {
            local = "/" + m_name +nav_config.find("local-suffix").asString();
        }
        if (nav_config.check("navigation_server"))
        {
            navServer = nav_config.find("navigation_server").asString();
        }
        if (nav_config.check("map_locations_server"))
        {
            mapServer = nav_config.find("map_locations_server").asString();
        }
        if (nav_config.check("localization_server"))
        {
            locServer = nav_config.find("localization_server").asString();
        }
    }

    yarp::os::Property nav2DProp;
    nav2DProp.put("device", device);
    nav2DProp.put("local", local);
    nav2DProp.put("navigation_server", navServer);
    nav2DProp.put("map_locations_server", mapServer);
    nav2DProp.put("localization_server", locServer);
    nav2DProp.put("period",10);

    m_nav2DPoly.open(nav2DProp);
    if (!m_nav2DPoly.isValid())
    {
        yCError(WAITGOALAVAILABILITY_ACTION, "Error opening PolyDriver check parameters");
        return false;
    }
    m_nav2DPoly.view(m_iNav2D);
    if (!m_iNav2D)
    {
        yCError(WAITGOALAVAILABILITY_ACTION, "Error opening iFrameTransform interface. Device not available");
        return false;
    }

    if (!m_tourManagerPort.open(m_tourManagerPortName))
    {
        yCWarning(WAITGOALAVAILABILITY_ACTION, "Error! Cannot the tourManagerRPC client port (%s)",m_tourManagerPortName.c_str());
        return false;
    }
    if (!m_tourManager.yarp().attachAsClient(m_tourManagerPort))
    {
        yCWarning(WAITGOALAVAILABILITY_ACTION, "Error! Cannot attach the %s port as client",m_tourManagerPortName.c_str());
        return false;
    }

    if (!m_pBt.open(m_pBtName))
    {
        yCWarning(WAITGOALAVAILABILITY_ACTION, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_pBt))
    {
        yCWarning(WAITGOALAVAILABILITY_ACTION, "Error! Could not attach as server");
        return false;
    }

    yCInfo(WAITGOALAVAILABILITY_ACTION, "Configuration Done!");
    return true;
}

bool waitGoalAvailability::close()
{
    m_tourManagerPort.close();
    m_pBt.close();

    return true;
}

double waitGoalAvailability::getPeriod()
{
    return m_period;
}

bool waitGoalAvailability::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}

bool waitGoalAvailability::updateModule()
{
    return true;
}

yarp::os::Bottle waitGoalAvailability::sendCommandTo(std::string s, yarp::os::Port &p)
{
    yarp::os::Bottle c;
    yarp::os::Bottle response;
    c.addString(s);
    p.write(c, response);
    return response;
}

bool waitGoalAvailability::start()
{

    // m_iNav2D->stopNavigation();
    // m_teller.sadFaceWarning();
    // sentenceGenerator sg(m_scenarioFolder);
    // sg.setLanguage(m_teller.getCurrentLanguage());
    // int count = 0;
    // int limit = 60;
    // while (!m_stopped && count < limit)
    // {
    //     count++;
    //     Sentence tmp=sg.recoveryIfGoalNotAvailable();
    //     //send text to teller
    //     while(tmp.size()>0 && !m_stopped)
    //     {
    //         Action a= tmp.pop();
    //         //send text to teller
    //         m_teller.play(a.m_content);
    //         while(m_teller.isPlaying() && !m_stopped){
    //             yarp::os::Time::delay(0.1);
    //         }
    //     }

    //     yCInfo(WAITGOALAVAILABILITY_ACTION, "start listening");
    //     yarp::os::Time::delay(m_period);
    // }
    m_stopped = false;

    return true;
}

void waitGoalAvailability::stop()
{
    m_stopped = true;
}

SkillStatus waitGoalAvailability::get_status()
{
    return m_status;
}
