/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file yarp_condition.h
 * @authors: Michele Colledanchise <michele.colledanchise@iit.it>
 */


#include <yarp_condition.h>
#include <yarp/os/LogStream.h>
YARPCondition::YARPCondition(string name, const NodeConfiguration& config) :
        ConditionNode(name, config),
        YARPNode(name, name)
{
    Optional<std::string> msg = getInput<std::string>("carrier");
    if (msg.has_value()) {
        set_carrier(msg.value());
    }

    bool ok = init();
    if(!ok)
    {
       yError() << "Something went wrong in the node init() of " << name;
    }
}

NodeStatus YARPCondition::tick()
{
    bool is_ok = m_bt_request_start.start();
    return (is_ok ? NodeStatus::SUCCESS : NodeStatus::FAILURE);
}

PortsList YARPCondition::providedPorts()
{
    // This action has a 2 input ports ("port_name" and "carrier")
    // Any port must have a name. The type is optional.
    return { InputPort<std::string>("port_name"),
             InputPort<std::string>("carrier") };
}
