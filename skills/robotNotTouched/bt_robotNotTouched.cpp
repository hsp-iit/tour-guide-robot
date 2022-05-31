
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "bt_robotNotTouched.h"

YARP_LOG_COMPONENT(ROBOT_NOT_TOUCHED, "behavior_tour_robot.skills.robotNotTouched", yarp::os::Log::TraceType)

robotNotTouched::robotNotTouched(std::string name) : m_name(name),
                                                     m_tourManagerPortName("/" + name + "/TourManager/thrift:c"),
                                                     m_portName("/" + name + "/BT_rpc/server"),
                                                     m_period(0.1),
                                                     m_threshold(500)
{
}

double robotNotTouched::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

bool robotNotTouched::updateModule()
{
    m_notTouched = isTouched();
    double time_start = yarp::os::Time::now();

    if (!m_notTouched)
    {
        m_tourManager.sendError("TOUCHED_ERROR");
        do
        {
            if (yarp::os::Time::now() - time_start >= 5.0)
            {
                m_tourManager.sendError("TOUCHED_ERROR");
                time_start = yarp::os::Time::now();
            }

            m_notTouched = isTouched();
            if (m_notTouched)
            {
                m_tourManager.recovered();
            }
            yarp::os::Time::delay(m_period);
        } while (!m_notTouched);
    }
    return true;
}

bool robotNotTouched::isTouched()
{
    yarp::sig::Vector vec_left;
    yarp::sig::Vector vec_right;
    m_IAnalog_left->read(vec_left);
    m_IAnalog_right->read(vec_right);
    if (!m_calibrated)
    {
        m_constVecLeft(7);
        m_constVecRight(7);
        for (int i = 0; i < 3; i++)
        {
            m_constVecLeft.push_back(vec_left[i]);
            m_constVecRight.push_back(vec_right[i]);
        }
        m_calibrated = true;
    }
    int sum_left = 0;
    int sum_right = 0;
    for (int i = 0; i < 3; i++)
    {
        sum_left += pow(vec_left[i] - m_constVecLeft[i], 2.0);
        sum_right += pow(vec_right[i] - m_constVecRight[i], 2.0);
    }
    yCDebug(ROBOT_NOT_TOUCHED) << "Left sum:" << std::to_string(sum_left);
    yCDebug(ROBOT_NOT_TOUCHED) << "Right sum:" << std::to_string(sum_right);

    return !((sum_left > m_threshold) || (sum_right > m_threshold));
}

bool robotNotTouched::configure(yarp::os::ResourceFinder &rf)
{
    m_threshold = 500;
    std::string robotname = "cer";
    bool groupAvailable = rf.check("BT_SKILLS_PARAMETERS");
    if (groupAvailable)
    {
        yarp::os::Searchable &searchable_path = rf.findGroup("BT_SKILLS_PARAMETERS");
        m_threshold = searchable_path.find("thresholdRobotNotTouched").asInt32();
        robotname = searchable_path.find("robot").asString();
    }
    else
    {
        yError() << "group not found";
        return false;
    }

    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(ROBOT_NOT_TOUCHED, "Error! YARP Network is not initialized");
        return false;
    }

    if (!m_port.open(m_portName))
    {
        yCWarning(ROBOT_NOT_TOUCHED, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_port))
    {
        yCWarning(ROBOT_NOT_TOUCHED, "Error! Could not attach as server");
        return false;
    }

    bool okNav = rf.check("ANALOGSENSOR_CLIENT");
    std::string device = "analogsensorclient";
    std::string local_suffix = "/analogClient";
    if (okNav)
    {
        yarp::os::Searchable &nav_config = rf.findGroup("ANALOGSENSOR_CLIENT");
        if (nav_config.check("device"))
        {
            device = nav_config.find("device").asString();
        }
        if (nav_config.check("local_suffix"))
        {
            local_suffix = nav_config.find("local_suffix").asString();
        }
    }
    // left
    yarp::os::Property analog2DProp;
    analog2DProp.put("device", device);
    analog2DProp.put("local", "/" + m_name + "/left" + local_suffix);
    analog2DProp.put("remote", "/" + robotname + "/left_arm/FT:o");
    analog2DProp.put("carrier", "tcp");

    m_AnalogPoly_left.open(analog2DProp);
    if (!m_AnalogPoly_left.isValid())
    {
        yCError(ROBOT_NOT_TOUCHED, "Error opening PolyDriver check parameters");
        return false;
    }
    m_AnalogPoly_left.view(m_IAnalog_left);
    if (!m_IAnalog_left)
    {
        yCError(ROBOT_NOT_TOUCHED, "Error opening iFrameTransform interface. Device not available");
        return false;
    }

    // right
    analog2DProp.put("local", "/" + m_name + "/right" + local_suffix);
    analog2DProp.put("remote", "/" + robotname + "/right_arm/FT:o");
    m_AnalogPoly_right.open(analog2DProp);
    if (!m_AnalogPoly_right.isValid())
    {
        yCError(ROBOT_NOT_TOUCHED, "Error opening PolyDriver check parameters");
        return false;
    }
    m_AnalogPoly_right.view(m_IAnalog_right);
    if (!m_IAnalog_right)
    {
        yCError(ROBOT_NOT_TOUCHED, "Error opening iFrameTransform interface. Device not available");
        return false;
    }

    if (!m_tourManagerPort.open(m_tourManagerPortName))
    {
        yCError(ROBOT_NOT_TOUCHED) << "Error! Cannot the tourManagerRPC client port (%s)", m_tourManagerPortName.c_str();
        return false;
    }
    if (!m_tourManager.yarp().attachAsClient(m_tourManagerPort))
    {
        yCError(ROBOT_NOT_TOUCHED) << "Error! Cannot attach the %s port as client", m_tourManagerPortName.c_str();
        return false;
    }

    yCInfo(ROBOT_NOT_TOUCHED, "Configuration Done!");

    return true;
}

bool robotNotTouched::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}

bool robotNotTouched::close()
{
    m_port.close();
    if (m_AnalogPoly_left.isValid())
        m_AnalogPoly_left.close();
    if (m_AnalogPoly_right.isValid())
        m_AnalogPoly_right.close();
    return true;
}

SkillStatus robotNotTouched::get_status()
{
    return m_status;
}

bool robotNotTouched::start()
{

    return m_notTouched;
}

void robotNotTouched::stop()
{
}
