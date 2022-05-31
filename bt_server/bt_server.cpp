
/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#include "bt_server.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

YARP_LOG_COMPONENT(BT_SERVER, "behavior_tour_robot.bt_server", yarp::os::Log::TraceType)

btServer::btServer(std::string name) : m_name(name),
                                       m_pBtName("/" + name + "/rpc:i"),
                                       m_period(1)
{
}

double btServer::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

bool btServer::startRpc()
{
    yCInfo(BT_SERVER) << "start received";
    m_process = true;
    return true;
}

bool btServer::stopRpc()
{
    yCInfo(BT_SERVER) << "stop received";
    m_process = false;
    return true;
}

// This is our main function. Will be called periodically every getPeriod() seconds
bool btServer::updateModule()
{
    // Create some logger
    StdCoutLogger logger_cout(m_tree);
    MinitraceLogger logger_minitrace(m_tree, "/tmp/bt_trace.json");
    FileLogger logger_file(m_tree, "/tmp/bt_trace.fbl");

#ifdef ZMQ_FOUND
    PublisherZMQ publisher_zmq(m_tree);
#endif
    printTreeRecursively(m_tree.rootNode());

    vector<TreeNode::Ptr> all_nodes_prt = m_tree.nodes;
    yCInfo(BT_SERVER) << "start2";
    while (!m_closed)
    {
        yCInfo(BT_SERVER) << "start";
        if (m_process)
        {
            yCDebug(BT_SERVER) << "Ticking the root node";
            m_tree.rootNode()->executeTick();
        }
    }

    //yCInfo(BT_SERVER) << "ConditionExample Server Skill Running";
    return true;
}

bool btServer::configure(yarp::os::ResourceFinder &rf)
{
    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(BT_SERVER, "Error! YARP Network is not initialized");
        return false;
    }

    if (!m_pBt.open(m_pBtName))
    {
        yCWarning(BT_SERVER, "Error! Cannot open YARP port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_pBt))
    {
        yCWarning(BT_SERVER, "Error! Could not attach as server");
        return false;
    }

    yCInfo(BT_SERVER, "Configuration Done!");

    m_bt_factory.registerNodeType<YARPAction>("YARPAction");
    m_bt_factory.registerNodeType<YARPCondition>("YARPCondition");
    std::string fileName = rf.findFile("from");

    yInfo(fileName.c_str());
    m_tree = m_bt_factory.createTreeFromFile(fileName);
    yCInfo(BT_SERVER) << "bt initialized";
    // Create some logger
    return true;
}
// Interrupt function.
bool btServer::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}
// Close function, to perform cleanup.
bool btServer::close()
{
    m_closed = true;
    yCInfo(BT_SERVER) << "Calling close function\n";
    //m_pQuery.close();
    m_pBt.close();
    return true;
}
