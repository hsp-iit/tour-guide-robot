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

#include "poi.h"

YARP_LOG_COMPONENT(POI, "behavior_tour_robot.aux_modules.PoI", yarp::os::Log::TraceType)

PoI::PoI(std::string name,
         std::unordered_map<std::string, std::vector<Action>> availableActions) : m_name(std::move(name)),
                                                                                  m_availableActions(std::move(availableActions))
{
}

std::string PoI::getName()
{
    return m_name;
}

bool PoI::isCommandValid(const std::string &command)
{
    return m_availableActions.count(command);
}

bool PoI::getActions(const std::string &command, std::vector<Action> &actions)
{
    if (!isCommandValid(command))
    {
        yCError(POI) << "Command" << command << "not supported";
        return false;
    }

    actions = m_availableActions[command];

    return true;
}

int PoI::getCommandMultiplesNum(const std::string &command)
{
    int c = 0;
    for (std::string cmd : getAvailableCommands())
    {
        if (cmd.find(command) != std::string::npos)
        {
            c++;
        }
    }
    return c;
}

std::vector<std::string> PoI::getAvailableCommands()
{
    std::vector<std::string> availableCommands;
    for (const auto &mapElement : m_availableActions)
    {
        availableCommands.push_back(mapElement.first);
    }
    return availableCommands;
}
