/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file YARPNode.cpp
 * @authors: Michele Colledanchise <michele.colledanchise@iit.it>
 */


#include <yarp_node.h>
#include <behaviortree_cpp_v3/leaf_node.h>
#include <Skill_request.h>
#include <iostream>
#include <yarp/os/RpcClient.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

using namespace std;
using namespace BT;

YARPNode::YARPNode(string name, string server_port_name, string carrier) :
        name(name),
        m_client_port_name_tick("/"+name+"/BT_rpc/client/tick"),
        m_server_port_name("/"+name+"/BT_rpc/server"),
        m_carrier(std::move(carrier))
{
}

void YARPNode::set_carrier(std::string carrier)
{
    m_carrier = std::move(carrier);
}

bool YARPNode::init()
{
    yarp::os::Network yarp;

    if (!m_rpc_client_tick.open(m_client_port_name_tick))
    {
        yError() << "Could not open port " << m_client_port_name_tick;
        return false;
    }

    yarp::os::Network::sync(m_server_port_name, false);

    if (!yarp::os::Network::connect(m_client_port_name_tick, m_server_port_name, m_carrier))
    {
        yError() << "Could not connect to port " << m_server_port_name;
        return false;
    }

    if (!m_bt_request_start.yarp().attachAsClient(m_rpc_client_tick))
    {
        yError() << "Could not attach as client to " << m_server_port_name;
        return false;
    }
    yDebug() << "Node " << name << "initialized correctly";

    return true;
}

//NodeStatus YARPNode::status() const
//{
//    yDebug() << "Node " << name << "requesting get_status";

//    SkillStatus skill_status = m_bt_request_get_status.get_status();

//    return skill_to_bt_status(skill_status);
//}

NodeStatus YARPNode::skill_to_bt_status(SkillStatus skill_status) const
{
    switch (skill_status) {
    case SKILL_RUNNING:
       yDebug() << "Node" << name << "returns running";
        return NodeStatus::RUNNING;// may be two different enums (thrift and BT library). Making sure that the return status are the correct ones
    case SKILL_SUCCESS:
       yDebug() << "Node" << name << "returns success";
        return NodeStatus::SUCCESS;
    case SKILL_FAILURE:
       yDebug() << "Node" << name << "returns failure";
        return NodeStatus::FAILURE;
    case SKILL_IDLE:
       yDebug() << "Node" << name << "returns idle";
        return NodeStatus::IDLE;
    default:
       yError() << "Invalid return status for received by node " << name;
        return NodeStatus::FAILURE;
    }
}
