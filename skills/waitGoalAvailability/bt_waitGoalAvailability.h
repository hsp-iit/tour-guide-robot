/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef WAIT_ACT_H
#define WAIT_ACT_H

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <Skill_request.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <atomic>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <tourManagerRPC.h>

class waitGoalAvailability : public yarp::os::RFModule, public Skill_request
{
protected:
    std::string m_name;
    std::string m_pBtName;
    std::string m_sentence_output_name;
    tourManagerRPC m_tourManager;
    std::string m_tourManagerPortName;
    yarp::os::Port m_tourManagerPort;
    double m_period;
    yarp::os::RpcServer m_pBt;
    std::atomic<SkillStatus> m_status; // actually, no longer used
    std::atomic<bool> m_stopped;
    yarp::dev::PolyDriver m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D *m_iNav2D{nullptr};

public:
    waitGoalAvailability(std::string name);
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool interruptModule();
    virtual bool updateModule();
    yarp::os::Bottle sendCommandTo(std::string s, yarp::os::Port &p);
    SkillStatus get_status() override;
    bool start() override;
    void stop() override;
};

#endif
