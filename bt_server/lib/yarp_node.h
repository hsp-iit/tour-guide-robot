/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file yarp_node.h
 * @authors: Michele Colledanchise <michele.colledanchise@iit.it>
 */


#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <yarp/os/RpcClient.h>
#include <Skill_request.h>

using namespace std;
using namespace BT;

class YARPNode //: public LeafNode
{
public:
    YARPNode(string name, string server_port_name, string carrier = "tcp"s);
    virtual NodeStatus tick() = 0;
    NodeStatus skill_to_bt_status(SkillStatus status) const;
    bool init();
    string name;

    void set_carrier(std::string carrier);
    //NodeStatus status() const;
private:
    string m_client_port_name_tick;
    yarp::os::RpcClient m_rpc_client_tick;
    // I need two different Skill_request client since the leaf node may request the get_status() while the skill is busy with replying to the start()
    //mutable Skill_request m_bt_request_get_status; // mutable NodeStatus because status() is const. Actually, no longer used.
protected:
    string m_server_port_name;
    string m_carrier;
    Skill_request m_bt_request_start;



};
