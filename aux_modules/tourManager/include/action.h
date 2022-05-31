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

#ifndef BEHAVIOR_TOUR_ROBOT_ACTION_H
#define BEHAVIOR_TOUR_ROBOT_ACTION_H

#include <vector>
#include <map>
#include <nlohmann/json.hpp>
#include <yarp/os/LogStream.h>

// TODO: MAKE action types into an enum or better a checkable type

/**
 * Valid action types:
 *  - speak
 *  - dance
 *  - signal/command
 */

using json = nlohmann::json;

// example enum type declaration
enum ActionTypes
{
    SPEAK,
    DANCE,
    SIGNAL,
    INVALID = -1
};

// map TaskState values to JSON as strings
NLOHMANN_JSON_SERIALIZE_ENUM(ActionTypes, {{INVALID, nullptr},
                                           {SPEAK, "speak"},
                                           {DANCE, "dance"},
                                           {SIGNAL, "signal"}})

class Action
{
private:
    bool m_isBlocking{true};
    ActionTypes m_type{ActionTypes::INVALID};
    std::string m_param;

public:
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Action, m_type, m_isBlocking, m_param)

    /**
     * Invalid constructor
     */
    Action() = default; // Returns an invalid Action

    /**
     * Constructor
     *
     * @param type the type of the Action
     * @param isBlocking true if the action is blocking
     * @param param the Action parameter
     */
    Action(ActionTypes type, bool isBlocking, std::string param);

    /**
     * Copy constructor.
     *
     * @param inputAct the data to copy.
     */
    Action(const Action &inputAct) = default;

    /**
     * @brief Move constructor.
     *
     * @param movingAct the Action to be moved
     */
    Action(Action &&movingAct) noexcept = default;

    ~Action() = default;

    /**
     * Copy operator=
     * @param a const reference to an Action object
     * @return
     */
    Action &operator=(const Action &a) = default;

    [[nodiscard]] bool isBlocking() const;
    [[nodiscard]] ActionTypes getType() const;
    [[nodiscard]] std::string getParam() const;
};

#endif // BEHAVIOR_TOUR_ROBOT_ACTION_H
