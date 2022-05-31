
/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#ifndef BT_SERVER_H
#define BT_SERVER_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/os/RpcClient.h>
#include <iostream>
#include <yarp/os/Time.h>
#include <thread>
#include <atomic>

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
#include <yarp/os/BufferedPort.h>

using namespace std;
using namespace BT;

class btServer : public yarp::os::RFModule, public rpc_request
{
public:
  btServer(std::string name);
  double getPeriod();
  // This is our main function. Will be called periodically every getPeriod() seconds
  bool updateModule();
  bool configure(yarp::os::ResourceFinder &rf);
  // Interrupt function.
  bool interruptModule();
  // Close function, to perform cleanup.
  bool close();
  virtual bool startRpc() override;
  virtual bool stopRpc() override;

private:
  std::string m_name;
  std::string m_pBtName;
  bool m_closed = false;
  bool m_process = true;
  double m_period;
  yarp::os::RpcServer m_pBt;
  BehaviorTreeFactory m_bt_factory;
  BT::Tree m_tree;
};

#endif //ATPOICOND_H
