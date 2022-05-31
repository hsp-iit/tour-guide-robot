
/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#ifndef BATTERY_CHARGED_H
#define BATTERY_CHARGED_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <iostream>
#include <atomic>
#include <Skill_request.h>
#include <yarp/dev/IBattery.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/LogComponent.h>


class BatteryCharged : public yarp::os::RFModule, public Skill_request
{
public:
    BatteryCharged(std::string name);
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
    void resetCounter();

    // Internal member functions

private:

    std::string                       m_name;
    std::string                       m_pBtName;
    std::string                       m_pQueryName;
    double                            m_period;
    bool                              m_flag;
    yarp::os::Port                    m_pBt;
    yarp::os::BufferedPort<yarp::os::Bottle>              m_pQuery;
    std::atomic<SkillStatus>          m_status;
    yarp::dev::IBattery   *m_iBattery{nullptr};
    yarp::dev::PolyDriver M_poliDriver;


};

#endif  //ATPOICOND_H
