
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "bt_returnToChargePoint.h"

YARP_LOG_COMPONENT(RETURNTOCHARGEPOINT_ACTION, "behavior_tour_robot.skills.returntochargepoint_action", yarp::os::Log::TraceType)

returnToChargePoint::returnToChargePoint(std::string name) : m_name(name),
                                                             m_stopped(false),
                                                             m_inputPoIName("/" + name + "/goal/rpc:o"),
                                                             m_portName("/" + name + "/BT_rpc/server"),
                                                             m_period(100.0)
{
}

double returnToChargePoint::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

bool returnToChargePoint::updateModule()
{
    return true;
}

bool returnToChargePoint::configure(yarp::os::ResourceFinder &rf)
{
    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(RETURNTOCHARGEPOINT_ACTION, "Error! YARP Network is not initialized");
        return false;
    }

    bool okGeneral = rf.check("GENERAL");
    if (okGeneral)
    {
        yarp::os::Searchable &general_config = rf.findGroup("GENERAL");
        if (general_config.check("period"))
        {
            m_period = general_config.find("period").asFloat64();
        }
        if (general_config.check("rpc_port"))
        {
            m_portName = general_config.find("rpc_port").asString();
        }
        if (general_config.check("poi_port_in"))
        {
            m_inputPoIName = general_config.find("poi_port_in").asString();
        }
    }

    m_sleep = int(m_period * 1000 / 10);

    if (!m_port.open(m_portName))
    {
        yCWarning(RETURNTOCHARGEPOINT_ACTION, "Error! Cannot open YARP port");
        return false;
    }

    if (!m_inputPoI.open(m_inputPoIName))
    {
        yCWarning(RETURNTOCHARGEPOINT_ACTION, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_port))
    {
        yCWarning(RETURNTOCHARGEPOINT_ACTION, "Error! Could not attach as server");
        return false;
    }

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
        if (nav_config.check("local"))
        {
            local = nav_config.find("local").asString();
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
    nav2DProp.put("period", 10);

    m_nav2DPoly.open(nav2DProp);
    if (!m_nav2DPoly.isValid())
    {
        yCError(RETURNTOCHARGEPOINT_ACTION, "Error opening PolyDriver check parameters");
        return false;
    }
    m_nav2DPoly.view(m_iNav2D);
    if (!m_iNav2D)
    {
        yCError(RETURNTOCHARGEPOINT_ACTION, "Error opening iFrameTransform interface. Device not available");
        return false;
    }

    yCInfo(RETURNTOCHARGEPOINT_ACTION, "Configuration Done!");

    return true;
}

bool returnToChargePoint::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}

bool returnToChargePoint::close()
{
    yCInfo(RETURNTOCHARGEPOINT_ACTION) << "Calling close function\n";
    m_port.close();
    m_inputPoI.close();
    if (m_nav2DPoly.isValid())
        m_nav2DPoly.close();
    return true;
}

SkillStatus returnToChargePoint::get_status()
{
    return m_status;
}

bool returnToChargePoint::start()
{
    // go to charge poi
    std::string goalName = "charge_poi";
    m_iNav2D->gotoTargetByLocationName(goalName);

    // wait until charge poi reached
    while (!m_stopped)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep));
        m_stopped = checkPosition();
    }

    // wait until battery charged
    m_stopped = false;
    while (!m_stopped)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep));
        // m_stopped=checkBattery();
    }

    // reset indeces and states
    // resetGoogleDialog();
    // resetPoIManager();
    // resetsentenceGenerator();
    return true;
}

void returnToChargePoint::stop()
{
    m_stopped = true;
}

bool returnToChargePoint::checkPosition()
{

    std::string goal = "charge_poi";
    yarp::dev::Nav2D::NavigationStatusEnum currentStatus;
    m_iNav2D->getNavigationStatus(currentStatus);
    bool moving = currentStatus == yarp::dev::Nav2D::navigation_status_moving;
    yarp::dev::Nav2D::Map2DLocation theGoal;
    m_iNav2D->getLocation(goal, theGoal);
    yCInfo(RETURNTOCHARGEPOINT_ACTION) << "This is the goal: " << theGoal.toString();
    std::string targetname;
    m_iNav2D->getNameOfCurrentTarget(targetname);
    yCInfo(RETURNTOCHARGEPOINT_ACTION) << "This is the goal: " << goal << "and this is the target " << targetname;
    bool response = !moving && m_iNav2D->checkNearToLocation(theGoal, 1.0, 5.0) && targetname == goal;
    return response;
}
