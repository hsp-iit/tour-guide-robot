
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "bt_motorsNotInFault.h"

YARP_LOG_COMPONENT(MOTORS_NOT_IN_FAULT, "behavior_tour_robot.skills.motorsNotInFault", yarp::os::Log::TraceType)

motorsNotInFault::motorsNotInFault(std::string name) : m_name(name),
                                                       m_portName("/" + name + "/BT_rpc/server"),
                                                       m_tourManagerPortName("/" + name + "/TourManager/thrift:c"),
                                                       m_period(0.1)
{
}

double motorsNotInFault::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

bool motorsNotInFault::updateModule()
{
    m_condition = !(BaseInFault() || LeftArmInFault() || RightArmInFault());
    double time_start = yarp::os::Time::now();

    if (!m_condition)
    {
        m_tourManager.sendError("MOTORS_ERROR");
        do
        {
            if (yarp::os::Time::now() - time_start >= 5.0)
            {
                m_tourManager.sendError("MOTORS_ERROR");
                time_start = yarp::os::Time::now();
            }

            m_condition = !(BaseInFault() || LeftArmInFault() || RightArmInFault());
            if (m_condition)
            {
                m_tourManager.recovered();
            }
            yarp::os::Time::delay(m_period);
        } while (!m_condition);
    }
    return true;
}

bool motorsNotInFault::BaseInFault()
{

    int modes[10];
    m_IcontrolBoard_base->getControlModes(modes);
    // yCInfo(MOTORS_NOT_IN_FAULT) << modes[0] << " " << modes[1];

    bool baseFault = (modes[0] == VOCAB_CM_HW_FAULT) || (modes[1] == VOCAB_CM_HW_FAULT);
    bool baseIdle = ((modes[0] == VOCAB_CM_IDLE) && (modes[1] == VOCAB_CM_IDLE));
#ifdef BASE_INCOHERENT_TO_FAULT
    bool baseIncoherent = ((modes[0] == VOCAB_CM_VELOCITY) && (modes[1] != VOCAB_CM_VELOCITY)) || ((modes[1] == VOCAB_CM_VELOCITY) && (modes[0] != VOCAB_CM_VELOCITY));
#else
    bool baseIncoherent = ((modes[0] == VOCAB_CM_IDLE) && (modes[1] != VOCAB_CM_IDLE)) || ((modes[1] == VOCAB_CM_IDLE) && (modes[0] != VOCAB_CM_IDLE));
#endif

    if (baseIdle)
    {
        yCWarning(MOTORS_NOT_IN_FAULT) << "The base is in idle";
    }
    if (baseIncoherent)
    {
        yCWarning(MOTORS_NOT_IN_FAULT) << "The two wheels are in a different mode. Please check.";
    }

    return baseFault || baseIncoherent;
}

bool motorsNotInFault::LeftArmInFault()
{

    int modes[10];
    std::string idleMsg = "LEFT_ARM - Joints in IDLE state for the left arm:";
    std::string unknownMsg = "LEFT_ARM - Joints in UNKNOWN state for the left arm:";
    bool thereAreIdles = false;
    bool thereAreUnknowns = false;
    m_IcontrolBoard_leftArm->getControlModes(modes);
    for (int i = 0; i < 7; i++)
    {
        switch (modes[i])
        {
        case VOCAB_CM_HW_FAULT:
            return true;
        case VOCAB_CM_IDLE:
            idleMsg += " " + std::to_string(i);
            if (!thereAreIdles)
            {
                thereAreIdles = true;
            }
            break;
        case VOCAB_CM_UNKNOWN:
        case VOCAB_CM_NOT_CONFIGURED:
        case VOCAB_CM_CONFIGURED:
        case VOCAB_CM_CALIB_DONE:
        case VOCAB_CM_CALIBRATING:
            unknownMsg += " " + std::to_string(i);
            if (!thereAreUnknowns)
            {
                thereAreUnknowns = true;
            }
            break;
        default:
            yCDebugThreadThrottle(MOTORS_NOT_IN_FAULT, 5.0) << "LEFT_ARM - Unsupported control mode:" << modes[i] << "Check the joint using yarpmotorgui is deemed necessary";
            break;
        }
    }

    if (thereAreUnknowns)
    {
        yCWarning(MOTORS_NOT_IN_FAULT) << unknownMsg;
    }
    if (thereAreIdles)
    {
        yCWarning(MOTORS_NOT_IN_FAULT) << idleMsg;
    }

    return false;
}

bool motorsNotInFault::RightArmInFault()
{

    int modes[10];
    std::string idleMsg = "RIGHT_ARM - Joints in IDLE state for the right arm:";
    std::string unknownMsg = "RIGHT_ARM - Joints in UNKNOWN state for the right arm:";
    bool thereAreIdles = false;
    bool thereAreUnknowns = false;
    m_IcontrolBoard_rightArm->getControlModes(modes);
    for (int i = 0; i < 7; i++)
    {
        switch (modes[i])
        {
        case VOCAB_CM_HW_FAULT:
            return true;
        case VOCAB_CM_IDLE:
            idleMsg += " " + std::to_string(i);
            if (!thereAreIdles)
            {
                thereAreIdles = true;
            }
            break;
        case VOCAB_CM_UNKNOWN:
        case VOCAB_CM_NOT_CONFIGURED:
        case VOCAB_CM_CONFIGURED:
        case VOCAB_CM_CALIB_DONE:
        case VOCAB_CM_CALIBRATING:
            unknownMsg += " " + std::to_string(i);
            if (!thereAreUnknowns)
            {
                thereAreUnknowns = true;
            }
            break;
        default:
            yCDebugThreadThrottle(MOTORS_NOT_IN_FAULT, 5.0) << "RIGHT_ARM - Unsupported control mode:" << modes[i] << "Check the joint using yarpmotorgui is deemed necessary";
            break;
        }
    }

    if (thereAreUnknowns)
    {
        yCWarning(MOTORS_NOT_IN_FAULT) << unknownMsg;
    }
    if (thereAreIdles)
    {
        yCWarning(MOTORS_NOT_IN_FAULT) << idleMsg;
    }

    return false;
}

bool motorsNotInFault::configure(yarp::os::ResourceFinder &rf)
{
    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(MOTORS_NOT_IN_FAULT, "Error! YARP Network is not initialized");
        return false;
    }

    if (!m_port.open(m_portName))
    {
        yCWarning(MOTORS_NOT_IN_FAULT, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_port))
    {
        yCWarning(MOTORS_NOT_IN_FAULT, "Error! Could not attach as server");
        return false;
    }
    std::string robotName;

    robotName = "cer";
    // create your module
    bool groupAvailable = rf.check("BT_SKILLS_PARAMETERS");
    if (groupAvailable)
    {
        yarp::os::Searchable &searchable_path = rf.findGroup("BT_SKILLS_PARAMETERS");
        robotName = searchable_path.find("robot").asString();
    }
    else
    {
        yCError(MOTORS_NOT_IN_FAULT, "robot name required");
        return false;
    }

    // mobile base configuration
    yarp::os::Property analog2DProp;
    std::string device = "remote_controlboard";
    analog2DProp.put("device", device);
    analog2DProp.put("local", "/" + m_name + "base");
    analog2DProp.put("remote", "/" + robotName + "/mobile_base");
    analog2DProp.put("carrier", "tcp");

    m_controlBoard_base.open(analog2DProp);
    if (!m_controlBoard_base.isValid())
    {
        yCError(MOTORS_NOT_IN_FAULT, "Error opening PolyDriver check parameters");
        return false;
    }
    m_controlBoard_base.view(m_IcontrolBoard_base);
    if (!m_IcontrolBoard_base)
    {
        yCError(MOTORS_NOT_IN_FAULT, "Error opening iFrameTransform interface. Device not available");
        return false;
    }

    // left_arm configuration
    device = "remote_controlboard";
    analog2DProp.put("device", device);
    analog2DProp.put("local", "/" + m_name + "left");
    analog2DProp.put("remote", "/" + robotName + "/left_arm");
    analog2DProp.put("carrier", "tcp");

    m_controlBoard_leftArm.open(analog2DProp);
    if (!m_controlBoard_leftArm.isValid())
    {
        yCError(MOTORS_NOT_IN_FAULT, "Error opening PolyDriver check parameters");
        return false;
    }
    m_controlBoard_leftArm.view(m_IcontrolBoard_leftArm);
    if (!m_IcontrolBoard_leftArm)
    {
        yCError(MOTORS_NOT_IN_FAULT, "Error opening iFrameTransform interface. Device not available");
        return false;
    }

    // right_arm configuration
    device = "remote_controlboard";
    analog2DProp.put("device", device);
    analog2DProp.put("local", "/" + m_name + "right");
    analog2DProp.put("remote", "/" + robotName + "/right_arm");
    analog2DProp.put("carrier", "tcp");

    m_controlBoard_rightArm.open(analog2DProp);
    if (!m_controlBoard_rightArm.isValid())
    {
        yCError(MOTORS_NOT_IN_FAULT, "Error opening PolyDriver check parameters");
        return false;
    }
    m_controlBoard_rightArm.view(m_IcontrolBoard_rightArm);
    if (!m_IcontrolBoard_rightArm)
    {
        yCError(MOTORS_NOT_IN_FAULT, "Error opening iFrameTransform interface. Device not available");
        return false;
    }

    if (!m_tourManagerPort.open(m_tourManagerPortName))
    {
        yCError(MOTORS_NOT_IN_FAULT) << "Error! Cannot the tourManagerRPC client port (%s)", m_tourManagerPortName.c_str();
        return false;
    }
    if (!m_tourManager.yarp().attachAsClient(m_tourManagerPort))
    {
        yCError(MOTORS_NOT_IN_FAULT) << "Error! Cannot attach the %s port as client", m_tourManagerPortName.c_str();
        return false;
    }

    yCInfo(MOTORS_NOT_IN_FAULT, "Configuration Done!");

    return true;
}

bool motorsNotInFault::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}

bool motorsNotInFault::close()
{
    m_port.close();
    m_tourManagerPort.close();
    if (m_controlBoard_base.isValid())
        m_controlBoard_base.close();
    if (m_controlBoard_rightArm.isValid())
        m_controlBoard_rightArm.close();
    if (m_controlBoard_leftArm.isValid())
        m_controlBoard_leftArm.close();
    return true;
}

SkillStatus motorsNotInFault::get_status()
{
    return m_status;
}

bool motorsNotInFault::start()
{
    yCDebug(MOTORS_NOT_IN_FAULT) << "Skill status returned" << m_condition;
    return m_condition;
}

void motorsNotInFault::stop()
{
}
