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

#include "tour.h"

YARP_LOG_COMPONENT(TOUR, "behavior_tour_robot.aux_modules.Tour", yarp::os::Log::TraceType)

Tour::Tour(std::string lang,
           std::unordered_map<std::string, std::unordered_map<std::string, PoI>> availablePoIs,
           std::vector<std::string> activeTourPoIs) : m_currentLanguage(std::move(lang)),
                                                      m_availablePoIs(std::move(availablePoIs)),
                                                      m_activeTourPoIs(std::move(activeTourPoIs))
{
}

std::vector<std::string> Tour::getAvailableLanguages() const
{
    std::vector<std::string> languages;
    for (auto &p : m_availablePoIs)
    {
        languages.push_back(p.first);
    }

    return languages;
}

std::string Tour::getCurrentLanguage() const
{
    return m_currentLanguage;
}

bool Tour::languageSupported(const std::string &lang)
{
    for (auto &p : m_availablePoIs)
    {
        if (lang == p.first)
        {
            return true;
        }
    }
    return false;
}

bool Tour::setCurrentLanguage(const std::string &langToSet)
{
    if (m_availablePoIs.count(langToSet) > 0)
    {
        m_currentLanguage = langToSet;
        return true;
    }

    yCError(TOUR) << "The selected language is not supported";
    return false;
}

bool Tour::getPoI(const std::string &poiName, PoI &outputPoI)
{
    if (m_availablePoIs[m_currentLanguage].count(poiName) > 0)
    {
        outputPoI = m_availablePoIs[m_currentLanguage][poiName];
        return true;
    }
    return false;
}

bool Tour::getPoI(const std::string &poiName, const std::string &lang, PoI &outputPoI)
{
    if (m_availablePoIs[lang].count(poiName) > 0)
    {
        outputPoI = m_availablePoIs[lang][poiName];
        return true;
    }
    return false;
}

std::vector<std::string> Tour::getPoIsList() const
{
    return m_activeTourPoIs;
}
