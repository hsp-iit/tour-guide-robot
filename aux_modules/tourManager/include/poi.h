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

#ifndef BEHAVIOR_TOUR_ROBOT_POI_H
#define BEHAVIOR_TOUR_ROBOT_POI_H

#include <action.h>

class PoI
{
private:
    std::string m_name;
    std::unordered_map<std::string, std::vector<Action>> m_availableActions;

public:
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PoI, m_name, m_availableActions)

    PoI() = default;

    /**
     * Constructor
     *
     * @param name the name of the PoI
     * @param availableActions a map containing the available Actions for the PoI
     */
    PoI(std::string name, std::unordered_map<std::string, std::vector<Action>> availableActions);

    /**
     * Copy constructor.
     *
     * @param inputPoI the data to copy.
     */
    PoI(const PoI &inputPoI) = default;

    /**
     * @brief Move constructor.
     *
     * @param movingPoI the PoI to be moved
     */
    PoI(PoI &&movingPoI) noexcept = default;

    ~PoI() = default;

    /**
     * Copy operator=
     * @param a const reference to a PoI object
     * @return
     */
    PoI &operator=(const PoI &p) = default;

    [[nodiscard]] std::string getName();

    // the command is a key of the map
    [[nodiscard]] bool isCommandValid(const std::string &command);

    bool getActions(const std::string &command, std::vector<Action> &actions);

    std::vector<std::string> getAvailableCommands();
    int getCommandMultiplesNum(const std::string &command);
};

#endif // BEHAVIOR_TOUR_ROBOT_POI_H