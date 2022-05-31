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

#include "GoogleLogger.h"

YARP_LOG_COMPONENT(GOOGLE_LOGGER, "behavior_tour_robot.aux_modules.GoogleLogger", yarp::os::Log::TraceType)

GoogleLogger::GoogleLogger(const std::string &name) : m_period(1.0),
                                                      m_name(name),
                                                      m_tourManagerClientPortName("/" + name + "/TourManager/thrift:c"),
                                                      m_userInputPortName("/" + name + "/userInput:i"),
                                                      m_googleDialogPortName("/" + name + "/googleDialog:i")
{
}

void UserInputReceiver::onRead(yarp::os::Bottle &input_text)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_lastInput = input_text.toString();
}

std::string UserInputReceiver::getLastInput()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string result = m_lastInput;
    m_lastInput = "";
    return result;
}

/*
 * this is configured as callback for google dialog commands
 */
bool GoogleLogger::respond(const yarp::os::Bottle &b, yarp::os::Bottle &reply)
{
    YARP_UNUSED(reply);

    // parsing the received command
    std::string command = b.toString();
    if (command.rfind("\"", 0) == 0)
    {
        command = command.substr(1, command.size() - 2);
    }
    // pick up last user input received
    std::string userInput = m_userInput.getLastInput();

    // getting the current poi
    std::string currentPoiName = m_tourManager.getCurrentPoIName();

    // store the sequence
    m_fileWriter.writeToFile(userInput, command, currentPoiName);
    yCInfo(GOOGLE_LOGGER) << "User said:" << userInput << "at PoI:" << currentPoiName << "and received:" << command;
    return true;
}

bool GoogleLogger::configure(yarp::os::ResourceFinder &rf)
{
    // period
    if (!rf.check("period"))
    {
        yCWarning(GOOGLE_LOGGER) << "period parameter not found, using default value of " << m_period;
    }
    else
    {
        m_period = rf.find("period").asFloat64();
    }

    // managing file
    std::string fileName = rf.check("file_name") ? rf.find("file_name").asString() : "googleLogger";

    if (!m_fileWriter.configure(fileName))
    {
        yCError(GOOGLE_LOGGER) << "cannot open file " + fileName;
        return false;
    }

    if (!m_userInputPort.open(m_userInputPortName))
    {
        yCError(GOOGLE_LOGGER) << "failed to open port" << m_userInputPortName;
        return false;
    }
    m_userInputPort.useCallback(m_userInput);

    if (!m_googleDialogPort.open(m_googleDialogPortName))
    {
        yCError(GOOGLE_LOGGER) << "failed to open port" << m_googleDialogPortName;
        return false;
    }
    attach(m_googleDialogPort);

    if (!m_tourManagerClientPort.open(m_tourManagerClientPortName))
    {
        yCError(GOOGLE_LOGGER) << "failed to open port" << m_tourManagerClientPortName;
        return false;
    }
    m_tourManager.yarp().attachAsClient(m_tourManagerClientPort);

    yCInfo(GOOGLE_LOGGER) << "Configuration finished!";
    return true;
}

bool GoogleLogger::close()
{
    m_fileWriter.close();
    m_userInputPort.close();
    m_googleDialogPort.close();
    m_tourManagerClientPort.close();
    return true;
}

double GoogleLogger::getPeriod()
{
    return m_period;
}

bool GoogleLogger::interruptModule()
{
    close();
    return true;
}

bool GoogleLogger::updateModule()
{
    return true;
}
