
/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#ifndef NOT_TOUCHED_H
#define NOT_TOUCHED_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <iostream>
#include <atomic>
#include <Skill_request.h>
#include <yarp/os/LogComponent.h>
#include <math.h>
#include <tourManagerRPC.h>

class robotNotTouched : public yarp::os::RFModule, public Skill_request
{
public:
  robotNotTouched(std::string name);
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
  double m_period;
  bool m_calibrated = false;
  bool m_notTouched = true;
  int m_threshold;

  yarp::os::RpcServer m_port;
  std::atomic<SkillStatus> m_status; // actually, no longer used
  yarp::dev::PolyDriver m_AnalogPoly_left;
  yarp::dev::PolyDriver m_AnalogPoly_right;
  yarp::dev::IAnalogSensor *m_IAnalog_left{nullptr};
  yarp::dev::IAnalogSensor *m_IAnalog_right{nullptr};
  yarp::sig::Vector m_constVecLeft;
  yarp::sig::Vector m_constVecRight;

  tourManagerRPC m_tourManager;
  std::string m_tourManagerPortName;
  yarp::os::Port m_tourManagerPort;

  bool isTouched();
};

#endif //NOT_TOUCHED_H
