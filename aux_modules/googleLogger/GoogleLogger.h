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

#ifndef BEHAVIOR_TOUR_ROBOT_GOOGLE_LOGGER_RECEIVER_H
#define BEHAVIOR_TOUR_ROBOT_GOOGLE_LOGGER_RECEIVER_H

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <tourManagerRPC.h>
#include "FileWriter.h"

class UserInputReceiver : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
private:
    std::string m_lastInput;
    std::mutex m_mutex; /** Internal mutex. **/

public:
    UserInputReceiver() = default;
    ~UserInputReceiver() = default;

    void onRead(yarp::os::Bottle &bot) override;
    std::string getLastInput();
};

class GoogleLogger : public yarp::os::RFModule
{
private:
    double m_period;
    std::string m_name;
    std::string m_userInputPortName;
    std::string m_googleDialogPortName;
    std::string m_tourManagerClientPortName;
    FileWriter m_fileWriter;
    yarp::os::BufferedPort<yarp::os::Bottle> m_userInputPort; // Port that receives what the user said to the robot
    UserInputReceiver m_userInput;
    yarp::os::Port m_googleDialogPort; // Port that receives the reply from google dialogue to what the user said
    yarp::os::Port m_tourManagerClientPort;
    tourManagerRPC m_tourManager;

public:
    GoogleLogger(const std::string &name);
    bool configure(yarp::os::ResourceFinder &rf) override;
    bool respond(const yarp::os::Bottle &b, yarp::os::Bottle &reply) override;
    bool close() override;
    double getPeriod() override;
    bool interruptModule() override;
    bool updateModule() override;
};

#endif
