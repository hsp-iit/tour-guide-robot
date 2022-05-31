
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "bt_failureNetworkNotReceived.h"

YARP_LOG_COMPONENT(FAILURENETWORK_NOTRECEIVED, "behavior_tour_robot.skills.failureNotReceived", yarp::os::Log::TraceType)

FailureNetworkNotReceived::FailureNetworkNotReceived(const std::string &name) : m_name(name),
                                                                                m_flag(true),
                                                                                m_pBtName("/" + name + "/BT_rpc/server"),
                                                                                m_tourManagerPortName("/" + name + "/TourManager/thrift:c"),
                                                                                m_period(1.0)
{
}

double FailureNetworkNotReceived::getPeriod()
{
    return m_period;
}

// This is our main function. Will be called periodically every getPeriod() seconds
bool FailureNetworkNotReceived::updateModule()
{
    m_flag = isNetworkUp();
    double time_start = yarp::os::Time::now();

    if (!m_flag)
    {
        m_tourManager.sendError("NETWORK_ERROR");
        do
        {
            if (yarp::os::Time::now() - time_start >= 5.0)
            {
                m_tourManager.sendError("NETWORK_ERROR");
                time_start = yarp::os::Time::now();
            }

            m_flag = isNetworkUp();
            if (m_flag)
            {
                m_tourManager.recovered();
            }
            yarp::os::Time::delay(m_period);
        } while (!m_flag);
    }

    return true;
}

bool FailureNetworkNotReceived::isNetworkUp()
{
    if (system("ping -c1 -s1 www.google.com") != 0) // Not success to ping google
    {
        return false;
    }
    else
    { // All is good
        return true;
    }
}

bool FailureNetworkNotReceived::configure(yarp::os::ResourceFinder &rf)
{
    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(FAILURENETWORK_NOTRECEIVED, "Error! YARP Network is not initialized");
        return false;
    }

    if (!m_pBt.open(m_pBtName))
    {
        yCWarning(FAILURENETWORK_NOTRECEIVED, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_pBt))
    {
        yCWarning(FAILURENETWORK_NOTRECEIVED, "Error! Could not attach as server");
        return false;
    }

    if (!m_tourManagerPort.open(m_tourManagerPortName))
    {
        yCError(FAILURENETWORK_NOTRECEIVED) << "Error! Cannot the tourManagerRPC client port (%s)", m_tourManagerPortName.c_str();
        return false;
    }
    if (!m_tourManager.yarp().attachAsClient(m_tourManagerPort))
    {
        yCError(FAILURENETWORK_NOTRECEIVED) << "Error! Cannot attach the %s port as client", m_tourManagerPortName.c_str();
        return false;
    }

    yCInfo(FAILURENETWORK_NOTRECEIVED, "Configuration Done!");

    return true;
}

// Interrupt function.
bool FailureNetworkNotReceived::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup";
    return true;
}
// Close function, to perform cleanup.
bool FailureNetworkNotReceived::close()
{
    yCInfo(FAILURENETWORK_NOTRECEIVED) << "Calling close function";
    m_pBt.close();
    return true;
}

SkillStatus FailureNetworkNotReceived::get_status()
{
    return m_status;
}

bool FailureNetworkNotReceived::start()
{
    return m_flag;
}

void FailureNetworkNotReceived::stop()
{
}
