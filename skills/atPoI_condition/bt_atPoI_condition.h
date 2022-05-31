
/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#ifndef ATPOICOND_H
#define ATPOICOND_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <iostream>
#include <yarp/os/LogComponent.h>
#include <atomic>
#include <Skill_request.h>
#include <tourManagerRPC.h>


class AtPoI_Cond : public yarp::os::RFModule, public Skill_request
{
public:
    AtPoI_Cond(std::string name);
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
    std::string                      m_name;
    double                           m_period;
    std::string                      m_portName;
    yarp::os::RpcServer              m_port;
    std::atomic<SkillStatus>         m_status;
    std::string                      m_tourManagerClientPortName;
    yarp::os::Port                   m_tourManagerClientPort;
    tourManagerRPC                   m_tourManager;
};

#endif  //ATPOICOND_H
