
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef MOTORS_NOT_IN_FAULT_COND_H
#define MOTORS_NOT_IN_FAULT_COND_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/os/Vocab.h>
#include <iostream>
#include <atomic>
#include <tourManagerRPC.h>
#include <Skill_request.h>
#include <yarp/os/LogComponent.h>

class motorsNotInFault : public yarp::os::RFModule, public Skill_request
{
public:
  motorsNotInFault(std::string name);
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
  bool LeftArmInFault();
  bool RightArmInFault();
  bool BaseInFault();

private:
  std::string m_name;
  std::string m_portName;
  bool m_condition = true;
  double m_period;
  yarp::os::RpcServer m_port;
  std::atomic<SkillStatus> m_status; // actually, no longer used
  yarp::dev::PolyDriver m_controlBoard_base;
  yarp::dev::IControlMode *m_IcontrolBoard_base{nullptr};

  yarp::dev::PolyDriver m_controlBoard_leftArm;
  yarp::dev::IControlMode *m_IcontrolBoard_leftArm{nullptr};

  yarp::dev::PolyDriver m_controlBoard_rightArm;
  yarp::dev::IControlMode *m_IcontrolBoard_rightArm{nullptr};

  tourManagerRPC m_tourManager;
  std::string m_tourManagerPortName;
  yarp::os::Port m_tourManagerPort;
};

#endif // MOTORS_NOT_IN_FAULT_COND_H
