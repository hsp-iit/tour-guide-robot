
/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#include "bt_goalAvailable.h"

YARP_LOG_COMPONENT(GOALAVAILABLE_CONDITION, "behavior_tour_robot.skills.goalavailable_condition", yarp::os::Log::TraceType)

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

goalAvailable::goalAvailable(std::string name) : m_name(name),
                                                 m_goalAvailable(true),
                                                 m_pBtName("/" + name + "/BT_rpc/server"),
                                                 m_period(0.2)
{
}

double goalAvailable::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

// This is our main function. Will be called periodically every getPeriod() seconds
bool goalAvailable::updateModule()
{
    // get name of next goal and check navigation status
    yarp::dev::Nav2D::NavigationStatusEnum currentStatus;
    m_iNav2D->getNavigationStatus(currentStatus);

    if (currentStatus == yarp::dev::Nav2D::navigation_status_moving)
    {
        m_iNav2D->getNameOfCurrentTarget(targetname);
    }

    if (!targetname.empty())
    {
        int n_humans = 0;
        double m_x_human = 0;
        double m_y_human = 0;

        //scan only for /human /shoulderCenter reference systems
        m_transformClientInt->getAllFrameIds(allFrameIds);
        filteredFrameIds.clear();
        for (int i = 0; i < allFrameIds.size(); i++)
        {
            //if (it2->compare(0, 6, "/human") == 0)
            if ((allFrameIds[i].compare(0, 6, "/human") == 0) && (allFrameIds[i].find("/shoulderCenter") != string::npos))
            {
                filteredFrameIds.push_back(allFrameIds[i]);
            }
        }

        // calculate area of interest
        yarp::dev::Nav2D::Map2DLocation theGoal;
        m_iNav2D->getLocation(targetname, theGoal);
        yCInfo(GOALAVAILABLE_CONDITION) << "Goal set to:" << theGoal.toString();

        // Get position of the humans
        for(const auto& it : filteredFrameIds)
        {
            if (m_transformClientInt->getTransform(it, targetFrame, transformMat) == false)
            {
                yCDebug(GOALAVAILABLE_CONDITION) << "no transform between: " << it << " and " << targetFrame;
                continue;
            }

            m_x_human = transformMat(0, 3);
            m_y_human = transformMat(1, 3);

            if (sqrt(pow(m_x_human-theGoal.x, 2.0) + pow(m_y_human-theGoal.y, 2.0)) <= m_area_radius)
            {
                n_humans++;
            }
        }
        yCInfo(GOALAVAILABLE_CONDITION) << "There are" << n_humans << " inside the goal area.";

        if (n_humans > 0)
            m_goalAvailable = false;
        else
            m_goalAvailable = true;
        yCInfo(GOALAVAILABLE_CONDITION) << "Goal is available:" << m_goalAvailable;

    }
    else
    {
        m_goalAvailable = true;
        yCDebugThreadThrottle(GOALAVAILABLE_CONDITION, 10) << "No goal specified... Returning true.";
    }
    return true;
}

bool goalAvailable::configure(yarp::os::ResourceFinder &rf)
{
    targetname = "";

    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(GOALAVAILABLE_CONDITION, "Error! YARP Network is not initialized");
        return false;
    }

    if (!m_pBt.open(m_pBtName))
    {
        yCWarning(GOALAVAILABLE_CONDITION, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_pBt))
    {
        yCWarning(GOALAVAILABLE_CONDITION, "Error! Could not attach as server");
        return false;
    }

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
        yCError(GOALAVAILABLE_CONDITION, "Error opening PolyDriver check parameters");
        return false;
    }
    m_nav2DPoly.view(m_iNav2D);
    if (!m_iNav2D)
    {
        yCError(GOALAVAILABLE_CONDITION, "Error opening iFrameTransform interface. Device not available");
        return false;
    }

    m_area_radius =rf.check("m_area_radiusGoalAvailable") ? rf.find("m_area_radiusGoalAvailable").asFloat64():1;

    okNav = rf.check("TRANSFORM_CLIENT");
    if (okNav)
    {
        yarp::os::Searchable &nav_config = rf.findGroup("TRANSFORM_CLIENT");
        if (nav_config.check("device"))
        {
            device = nav_config.find("device").asString();
        }
        device = nav_config.find("device").asString();
        local =nav_config.check("local-suffix")? "/" + m_name +nav_config.find("local-suffix").asString():  "/" + m_name +"/transformClient";
        remoteTCName =nav_config.check("remoteTC")? nav_config.find("remoteTC").asString():"/transformServer";
    }else{
        yError()<<"transform client group not found";
        return false;
    }
    okNav = rf.check("BT_SKILLS_PARAMETERS");
    if (okNav)
    {
        yarp::os::Searchable &nav_config = rf.findGroup("BT_SKILLS_PARAMETERS");
        targetFrame =nav_config.check("targetFrame")? nav_config.find("targetFrame").asString(): "map";
    }else{
        yError()<<"BT_SKILLS_PARAMETERS group not found";
        return false;
    }
    yInfo()<<rf.toString();
    // connect to transform client and load interface
    Property pTC;
    pTC.put("device", device);
    pTC.put("local", local);
    pTC.put("remote", remoteTCName);
    pTC.put("period", "10");
    m_transformClientDriver.open(pTC);

    m_transformClientDriver.view(m_transformClientInt);
    if (m_transformClientInt == nullptr)
    {
        yCError(GOALAVAILABLE_CONDITION) << "Unable to open Transform Client interface";
        return false;
    }
    yCInfo(GOALAVAILABLE_CONDITION) << "TranformClient successfully opened.";
    yCInfo(GOALAVAILABLE_CONDITION, "Configuration Done!");

    return true;
}
// Interrupt function.
bool goalAvailable::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}
// Close function, to perform cleanup.
bool goalAvailable::close()
{
    yCInfo(GOALAVAILABLE_CONDITION) << "Calling close function\n";
    m_pBt.close();
    return true;
}

SkillStatus goalAvailable::get_status()
{
    return m_status;
}

bool goalAvailable::start()
{
    yCInfo(GOALAVAILABLE_CONDITION) << "Module start() returned:" << m_goalAvailable;
    if (!m_goalAvailable)
        m_iNav2D->stopNavigation();
    return m_goalAvailable;
}
void goalAvailable::stop()
{
    yCError(GOALAVAILABLE_CONDITION) << "ConditionExample Received a stop. You probably defined this skill as Action Node in the Behavior Tree";
}
