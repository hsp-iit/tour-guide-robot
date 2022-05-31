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


#pragma once

#include <yarp_node.h>
#include <string>
#include <behaviortree_cpp_v3/condition_node.h>

class YARPCondition :  public ConditionNode, public YARPNode
{
public:
    YARPCondition(string name, const NodeConfiguration& config);
    NodeStatus tick() override;

    static PortsList providedPorts();
};
