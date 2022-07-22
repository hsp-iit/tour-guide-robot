
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "bt_recoveryMotorsInFault.h"

YARP_LOG_COMPONENT(RECOVERY_MOTORS_IN_FAULT, "behavior_tour_robot.skills.recoveryMotorsInFault", yarp::os::Log::TraceType)

recoveryMotorsInFault::recoveryMotorsInFault(std::string name) : m_name(name),
                                                                 m_portName("/" + name + "/BT_rpc/server"),
                                                                 m_tourManagerPortName("/" + name + "/TourManager/thrift:c"),
                                                                 m_period(10)
{
}

double recoveryMotorsInFault::getPeriod()
{
    return m_period;
}

bool recoveryMotorsInFault::updateModule()
{
    return true;
}

bool recoveryMotorsInFault::configure(yarp::os::ResourceFinder &rf)
{
    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(RECOVERY_MOTORS_IN_FAULT, "Error! YARP Network is not initialized");
        return false;
    }

    if (!m_port.open(m_portName))
    {
        yCWarning(RECOVERY_MOTORS_IN_FAULT, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_port))
    {
        yCWarning(RECOVERY_MOTORS_IN_FAULT, "Error! Could not attach as server");
        return false;
    }

    if (!m_tourManagerPort.open(m_tourManagerPortName))
    {
        yCError(RECOVERY_MOTORS_IN_FAULT) << "Error! Cannot the tourManagerRPC client port (%s)", m_tourManagerPortName.c_str();
        return false;
    }
    if (!m_tourManager.yarp().attachAsClient(m_tourManagerPort))
    {
        yCError(RECOVERY_MOTORS_IN_FAULT) << "Error! Cannot attach the %s port as client", m_tourManagerPortName.c_str();
        return false;
    }

    yCInfo(RECOVERY_MOTORS_IN_FAULT, "Configuration Done!");

    return true;
}

bool recoveryMotorsInFault::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}

bool recoveryMotorsInFault::close()
{
    m_port.close();
    m_tourManagerPort.close();
    return true;
}

SkillStatus recoveryMotorsInFault::get_status()
{
    return m_status;
}

bool recoveryMotorsInFault::start()
{
    m_tourManager.sendError("MOTORS_ERROR");
    return m_condition;
}

void recoveryMotorsInFault::stop()
{
    m_tourManager.recovered();
}
