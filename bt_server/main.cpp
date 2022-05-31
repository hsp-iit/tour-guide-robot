/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file main.cpp
 * @authors: Michele Colledanchise <michele.colledanchise@iit.it>
 */

#ifdef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif

#include <iostream>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <yarp_condition.h>
#include <yarp_action.h>
#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds
#include <yarp/os/LogStream.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>

using namespace std;
using namespace BT;

int main(int argc, char *argv[])
{

    yarp::os::Network yarp;

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    std::string fileName = rf.findFile("from");

    BehaviorTreeFactory bt_factory;
    bt_factory.registerNodeType<YARPAction>("YARPAction");
    bt_factory.registerNodeType<YARPCondition>("YARPCondition");

    yInfo(fileName.c_str());
    BT::Tree tree = bt_factory.createTreeFromFile(fileName);

    // Create some logger
    StdCoutLogger logger_cout(tree);
    MinitraceLogger logger_minitrace(tree, "/tmp/bt_trace.json");
    FileLogger logger_file(tree, "/tmp/bt_trace.fbl");

#ifdef ZMQ_FOUND
    PublisherZMQ publisher_zmq(tree);
#endif
    printTreeRecursively(tree.rootNode());

    vector<TreeNode::Ptr> all_nodes_prt = tree.nodes;

    while (true)
    {
        yDebug() << "Ticking the root node";
        tree.tickRoot();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
