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

#include "action.h"
#include <utility>

YARP_LOG_COMPONENT(ACTION, "behavior_tour_robot.aux_modules.Action", yarp::os::Log::TraceType)

Action::Action(ActionTypes type, bool isBlocking, std::string param) : m_type(type),
                                                                       m_isBlocking(isBlocking),
                                                                       m_param(std::move(param))
{
}

bool Action::isBlocking() const
{
    return m_isBlocking;
}

ActionTypes Action::getType() const
{
    return m_type;
}

std::string Action::getParam() const
{
    return m_param;
}
