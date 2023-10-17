
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef RECOVERY_MOTORS_IN_FAULT_ACT_H
#define RECOVERY_MOTORS_IN_FAULT_ACT_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <iostream>
#include <atomic>
#include <tourManagerRPC.h>
#include <Skill_request.h>
#include <yarp/os/LogComponent.h>

class recoveryMotorsInFault : public yarp::os::RFModule, public Skill_request
{
public:
  recoveryMotorsInFault(std::string name);
  double getPeriod();
  // This is our main function. Will be called periodically every getPeriod() seconds
  bool updateModule();
  bool configure(yarp::os::ResourceFinder &rf);
  // Interrupt function.
  bool interruptModule();
  // Close function, to perform cleanup.
  bool close();
  SkillStatus get_status() override;
  bool start() override;
  void stop() override;

private:
  std::string m_name;
  std::string m_portName;
  bool m_condition = true;
  double m_period;
  yarp::os::RpcServer m_port;
  std::atomic<SkillStatus> m_status; // actually, no longer used

  tourManagerRPC m_tourManager;
  std::string m_tourManagerPortName;
  yarp::os::Port m_tourManagerPort;
};

#endif // RECOVERY_MOTORS_IN_FAULT_ACT_H
