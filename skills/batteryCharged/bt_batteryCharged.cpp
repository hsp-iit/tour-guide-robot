
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "bt_batteryCharged.h"

YARP_LOG_COMPONENT(BATTERYCHARGED_CONDITION, "behavior_tour_robot.skills.batterycharged_condition", yarp::os::Log::TraceType)

BatteryCharged::BatteryCharged(std::string name) : m_name(name),
                                                   m_status(SKILL_IDLE),
                                                   m_flag(true),
                                                   m_pBtName("/" + name + "/BT_rpc/server"),
                                                   m_pQueryName("/" + name + "/cmd:i"),
                                                   m_period(1)
{
}

double BatteryCharged::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

// This is our main function. Will be called periodically every getPeriod() seconds
bool BatteryCharged::updateModule()
{
    double charge;
    m_iBattery->getBatteryCharge(charge);
    yCInfo(BATTERYCHARGED_CONDITION) << "initial battery charge  " << charge;
    return true;
}

bool BatteryCharged::configure(yarp::os::ResourceFinder &rf)
{

    yarp::os::Property options;
    options.put("device", "batteryClient");
    options.put("remote", "/battery");
    options.put("local", "/batteryClient");
    M_poliDriver.open(options);
    if (!M_poliDriver.isValid())
    {
        yCError(BATTERYCHARGED_CONDITION, "Error opening PolyDriver check parameters");
        return false;
    }
    M_poliDriver.view(m_iBattery);
    if (!m_iBattery)
    {
        yCError(BATTERYCHARGED_CONDITION, "Error opening iFrameTransform interface. Device not available");
        return false;
    }
    m_period = rf.check("period") ? rf.find("period").asInt32() : 1;
    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(BATTERYCHARGED_CONDITION, "Error! YARP Network is not initialized");
        return false;
    }

    if (!m_pBt.open(m_pBtName))
    {
        yCWarning(BATTERYCHARGED_CONDITION, "Error! Cannot open YARP port");
        return false;
    }

    if (!m_pQuery.open(m_pQueryName))
    {
        yCWarning(BATTERYCHARGED_CONDITION, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_pBt))
    {
        yCWarning(BATTERYCHARGED_CONDITION, "Error! Could not attach as server");
        return false;
    }

    yCInfo(BATTERYCHARGED_CONDITION, "Configuration Done!");

    return true;
}

// Interrupt function.
bool BatteryCharged::interruptModule()
{
    yCInfo(BATTERYCHARGED_CONDITION) << "Interrupting your module, for port cleanup";
    return true;
}

// Close function, to perform cleanup.
bool BatteryCharged::close()
{
    yCInfo(BATTERYCHARGED_CONDITION) << "Calling close function";
    m_pQuery.close();
    m_pBt.close();
    M_poliDriver.close();
    return true;
}

SkillStatus BatteryCharged::get_status()
{
    return m_status;
}

bool BatteryCharged::start()
{
    double charge;
    m_iBattery->getBatteryCharge(charge);
    return true;
}

void BatteryCharged::stop()
{
}
