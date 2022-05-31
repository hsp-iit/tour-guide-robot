
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "bt_goToPoI_action.h"

YARP_LOG_COMPONENT(GOTOPOI_ACTION, "behavior_tour_robot.skills.gotopoi_action", yarp::os::Log::TraceType)

GoToPoI_Act::GoToPoI_Act(std::string name) : m_name(name),
                                             m_stopped(false),
                                             m_tourManagerClientPortName("/" + name + "/TourManager/thrift:c"),
                                             m_portName("/" + name + "/BT_rpc/server"),
                                             m_period(100.0)
{
}

double GoToPoI_Act::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

bool GoToPoI_Act::updateModule()
{
    return true;
}

bool GoToPoI_Act::configure(yarp::os::ResourceFinder &rf)
{
    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(GOTOPOI_ACTION, "Error! YARP Network is not initialized");
        return false;
    }

    if (!m_port.open(m_portName))
    {
        yCWarning(GOTOPOI_ACTION, "Error! Cannot open YARP port");
        return false;
    }

    if (!m_tourManagerClientPort.open(m_tourManagerClientPortName))
    {
        yCWarning(GOTOPOI_ACTION, "Error! Cannot open YARP port");
        return false;
    }

    if (!m_tourManager.yarp().attachAsClient(m_tourManagerClientPort))
    {
        yCWarning(GOTOPOI_ACTION, "Error! Cannot attach the port as a client");
        return false;
    }

    if (!this->yarp().attachAsServer(m_port))
    {
        yCWarning(GOTOPOI_ACTION, "Error! Could not attach as server");
        return false;
    }

    yCInfo(GOTOPOI_ACTION, "Configuration Done!");

    return true;
}

bool GoToPoI_Act::interruptModule()
{
    m_stopped = true;
    // TODO: Add something to stop navigation?
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}

bool GoToPoI_Act::close()
{
    m_stopped = true;
    yCInfo(GOTOPOI_ACTION) << "Calling close function\n";
    m_port.close();
    m_tourManagerClientPort.close();
    return true;
}

SkillStatus GoToPoI_Act::get_status()
{
    return m_status;
}

bool GoToPoI_Act::start()
{
    return m_tourManager.sendToPoI();
}

void GoToPoI_Act::stop()
{
    m_stopped = true;
}
