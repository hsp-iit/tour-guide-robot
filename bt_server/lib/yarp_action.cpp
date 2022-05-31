/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file yarp_action.cpp
 * @authors: Michele Colledanchise <michele.colledanchise@iit.it>
 */


#include <yarp_action.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <chrono>
#include <thread>


YARPAction::YARPAction(string name, const NodeConfiguration& config) :
    CoroActionNode(name, config),
    YARPNode(name, name), m_client_port_name_halt("/"+name+"/BT_rpc/client/halt")
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

bool YARPAction::init()
{
 // Need to inizialize the port for sending the stop command
    yarp::os::Network yarp;

    if (!m_rpc_client_halt.open(m_client_port_name_halt))
    {
        yError() << "Could not open port " << m_client_port_name_halt;
        return false;
    }

    yarp::os::Network::sync(m_server_port_name, false);

    if (!yarp::os::Network::connect(m_client_port_name_halt, m_server_port_name, m_carrier))
    {
        yError() << "Could not connect to port " << m_server_port_name;
        return false;
    }


    if (!m_bt_request_stop.yarp().attachAsClient(m_rpc_client_halt))
    {
        yError() << "Could not attach as client to " << m_server_port_name;
        return false;
    }

    return YARPNode::init();
}

NodeStatus YARPAction::tick()
{
    yDebug() << "YARPAction::tick()";
    m_thread_start_handle = std::async(std::launch::async, [this]() {
        bool is_ok = m_bt_request_start.start();
        return (is_ok ? NodeStatus::SUCCESS : NodeStatus::FAILURE);
    });

    std::chrono::milliseconds span(100);

    while(m_thread_start_handle.wait_for(span)==std::future_status::timeout)
    {
        yDebug() << "Waiting for skill to respond";
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
        setStatusRunningAndYield();
    }

    NodeStatus node_status = m_thread_start_handle.get();
    m_thread_start_handle = {};
    return NodeStatus::SUCCESS;
}

PortsList YARPAction::providedPorts()
{
    // This action has a 2 input ports ("port_name" and "carrier")
    // Any port must have a name. The type is optional.
    return { InputPort<std::string>("port_name"),
                InputPort<std::string>("carrier") };
}

void YARPAction::halt()
{
    yDebug() << "Node" << CoroActionNode::name() << "sending halt to skill";
    m_bt_request_stop.stop();
    // send halt request to server
    CoroActionNode::halt();
}
