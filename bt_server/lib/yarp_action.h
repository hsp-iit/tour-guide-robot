/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file yarp_action.h
 * @authors: Michele Colledanchise <michele.colledanchise@iit.it>
 */

#pragma once


#include <yarp_node.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <string>
#include <future>
class YARPAction :  public CoroActionNode, public YARPNode
        // public virtual ActionNodeBase because the BT factory accepts only classes that explicitly inherits from ActionNodeBase or ConditionNode
{
public:
    YARPAction(string name, const NodeConfiguration &config);
    void halt() override;
    NodeStatus tick() override;
    static PortsList providedPorts();
    bool init();
private:
    std::future<NodeStatus> m_thread_start_handle;
    yarp::os::RpcClient m_rpc_client_halt;
    string m_client_port_name_halt;
    Skill_request m_bt_request_stop;


};
