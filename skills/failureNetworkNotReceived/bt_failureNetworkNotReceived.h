
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef FAILURE_NETWORK_NOT_RECEIVED_H
#define FAILURE_NETWORK_NOT_RECEIVED_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <iostream>
#include <atomic>
#include <tourManagerRPC.h>
#include <Skill_request.h>

class FailureNetworkNotReceived : public yarp::os::RFModule, public Skill_request
{
public:
  FailureNetworkNotReceived(const std::string &name);
  double getPeriod();
  // This is our main function. Will be called periodically every getPeriod() seconds
  bool updateModule();
  bool configure(yarp::os::ResourceFinder &rf);
  bool interruptModule();
  bool close();

  SkillStatus get_status() override;
  bool start() override;
  void stop() override;

private:
  std::string m_name;
  std::string m_pBtName;

  double m_period;
  bool m_flag;

  yarp::os::Port m_pBt;
  std::atomic<SkillStatus> m_status;

  tourManagerRPC m_tourManager;
  std::string m_tourManagerPortName;
  yarp::os::Port m_tourManagerPort;

  bool isNetworkUp();
};

#endif // FAILURE_NETWORK_NOT_RECEIVED_H
