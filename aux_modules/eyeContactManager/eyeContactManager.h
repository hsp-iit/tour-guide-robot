
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef EYE_CONTACT_MANAGER_H
#define EYE_CONTACT_MANAGER_H

#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Vocab.h>
#include <iostream>
#include <yarp/os/LogComponent.h>
#include <yarp/dev/IPositionControl.h>
#include <headSynchronizerRPC.h>
#include <math.h>

class Person
{
public:
  int x;
  int y;
  double depth;
  int looking;
  double confidence;
};

enum EyeContactStatus
{
  NOBODY,
  LOOKING,
  NOT_LOOKING
};

class eyeContactManager : public yarp::os::RFModule
{
public:
  eyeContactManager(std::string name);

  // RFModule members
  bool configure(yarp::os::ResourceFinder &rf);
  bool interruptModule();
  double getPeriod();
  bool updateModule();
  bool close();

private:
  double m_period;
  double m_lastEyeContactTime;
  bool m_isHeadReset;
  
  std::string m_gazeInputName;
  std::string m_eyeContactOutputName;
  std::string m_gazeControlName;

  yarp::os::BufferedPort<yarp::os::Bottle> m_pGazeInput;
  yarp::os::Port m_pEyeContactOutput;
  yarp::os::Port m_pGazeControl;
  headSynchronizerRPC m_headSynchronizer;

  // Navigation client
  yarp::dev::Nav2D::NavigationStatusEnum m_nav_status;
  yarp::dev::PolyDriver m_pNav;
  yarp::dev::Nav2D::INavigation2D *m_iNav;
  std::string m_remote_localization = "/localization2D_nws_yarp";
  std::string m_remote_map = "/map2D_nws_yarp";
  std::string m_remote_navigation = "/navigation2D_nws_yarp";

  EyeContactStatus GetClosestPersonLooking(yarp::os::Bottle *peopleDetected, Person &p);
  bool sendBufferTo(std::string s, yarp::os::BufferedPort<yarp::os::Bottle> &p);
  void resetNeck();
  bool lookAtPixel(std::string mode, double px, double py);
};

#endif // EYE_CONTACT_MANAGER_H
