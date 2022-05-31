
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "bt_atPoI_condition.h"

YARP_LOG_COMPONENT(ATPOI_CONDITION, "behavior_tour_robot.skills.atpoi_condition", yarp::os::Log::TraceType)

AtPoI_Cond::AtPoI_Cond(std::string name) : m_name(name),
                                           m_period(100.0),
                                           m_status(SKILL_IDLE),
                                           m_portName("/" + name + "/BT_rpc/server"),
                                           m_tourManagerClientPortName("/" + name + "/TourManager/thrift:c")
{
}

double AtPoI_Cond::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

bool AtPoI_Cond::updateModule()
{
    return true;
}

bool AtPoI_Cond::configure(yarp::os::ResourceFinder &rf)
{
    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(ATPOI_CONDITION, "Error! YARP Network is not initialized");
        return false;
    }

    if (!m_port.open(m_portName))
    {
        yCWarning(ATPOI_CONDITION, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_port))
    {
        yCWarning(ATPOI_CONDITION, "Error! Could not attach as server");
        return false;
    }

    if (!m_tourManager.yarp().attachAsClient(m_tourManagerClientPort))
    {
        yCWarning(ATPOI_CONDITION, "Error! Cannot attach the port as a client");
        return false;
    }

    if (!m_tourManagerClientPort.open(m_tourManagerClientPortName))
    {
        yCWarning(ATPOI_CONDITION, "Error! Cannot open YARP port");
        return false;
    }

    yCInfo(ATPOI_CONDITION, "Configuration Done!");

    return true;
}

bool AtPoI_Cond::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}

bool AtPoI_Cond::close()
{
    yCInfo(ATPOI_CONDITION) << "Calling close function\n";
    m_port.close();
    m_tourManagerClientPort.close();
    return true;
}

SkillStatus AtPoI_Cond::get_status()
{
    return m_status;
}

bool AtPoI_Cond::start()
{
    return m_tourManager.isAtPoI();
}

void AtPoI_Cond::stop()
{
    yCError(ATPOI_CONDITION) << "ConditionExample Received a stop. You probably defined this skill as Action Node in the Behavior Tree";
}
