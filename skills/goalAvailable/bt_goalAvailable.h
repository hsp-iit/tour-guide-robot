
/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#ifndef GOALAVAILABLE_H
#define GOALAVAILABLE_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/os/RpcClient.h>
#include <iostream>
#include <atomic>
#include <Skill_request.h>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>
#include <yarp/sig/Vector.h>
#include <yarp/os/Log.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/LogComponent.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

class goalAvailable : public yarp::os::RFModule, public Skill_request
{
public:
  goalAvailable(std::string name);
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
  //name
  std::string m_name;
  std::string m_pBtName;
  std::string targetname;
  //names crowd detector
  std::string targetFrame;
  std::string remoteTCName;
  std::vector<std::string> allFrameIds;
  std::vector<std::string> filteredFrameIds;
  yarp::sig::Matrix transformMat;

  double m_period;
  bool m_goalAvailable;
  double m_area_radius;

  //ports BT
  yarp::os::RpcServer m_pBt;
  std::atomic<SkillStatus> m_status;

  //drivers and interfaces
  yarp::dev::PolyDriver m_nav2DPoly;
  yarp::dev::Nav2D::INavigation2D *m_iNav2D{nullptr};
  yarp::dev::PolyDriver m_transformClientDriver;
  yarp::dev::IFrameTransform *m_transformClientInt;
};

#endif //GOALAVAILABLE_H
