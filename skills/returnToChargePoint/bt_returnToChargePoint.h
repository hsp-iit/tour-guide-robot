
/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#ifndef GOTOPOIACT_H
#define GOTOPOIACT_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include<yarp/os/LogStream.h>
#include<yarp/os/RpcClient.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <iostream>
#include <yarp/os/LogComponent.h>
#include <atomic>
#include <Skill_request.h>

#include <chrono>
#include <thread>

class returnToChargePoint : public yarp::os::RFModule, public Skill_request
{
public:
    returnToChargePoint(std::string name);
    double getPeriod();
    // This is our main function. Will be called periodically every getPeriod() seconds
    bool updateModule();
    bool configure(yarp::os::ResourceFinder &rf);
    // Interrupt function.
    bool interruptModule();
    // Close function, to perform cleanup.
    bool close();
    bool checkPosition();
    SkillStatus get_status() override;
    bool start() override;
    void stop() override;

private:
    std::string                      m_name;
    std::string                      m_inputPoIName;
    std::string                      m_portName;
    double                           m_period;
    int                              m_sleep;
    yarp::os::RpcServer              m_port;
    std::atomic<SkillStatus>         m_status; // actually, no longer used
    std::atomic<bool>                m_stopped;
    yarp::os::Port                   m_inputPoI;
    yarp::dev::PolyDriver            m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D* m_iNav2D{nullptr};
};

#endif //GOTOPOIACT_H
