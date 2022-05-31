
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "eyeContactManager.h"

YARP_LOG_COMPONENT(EYE_CONTACT, "behavior_tour_robot.aux_modules.eyeContactManager", yarp::os::Log::TraceType)

eyeContactManager::eyeContactManager(std::string name) : m_gazeInputName("/" + name + "/pred:i"),
                                                         m_eyeContactOutputName("/" + name + "/pred:o"),
                                                         m_gazeControlName("/" + name + "/control:o"),
                                                         m_period(0.1)
{
}

double eyeContactManager::getPeriod()
{
    return m_period;
}

EyeContactStatus eyeContactManager::GetClosestPersonLooking(yarp::os::Bottle *peopleBottle, Person &closestPerson)
{
    bool found = false;
    if (peopleBottle->size() == 0)
    {
        return EyeContactStatus::NOBODY;
    }

    std::vector<Person> peopleDetected;

    for (int personIndex = 0; personIndex < peopleBottle->size(); personIndex++)
    {
        Person pTmp;
        yarp::os::Bottle *person = peopleBottle->get(personIndex).asList();
        yarp::os::Bottle *cluster = person->get(1).asList();
        pTmp.x = cluster->get(0).asInt32() / 2;
        pTmp.y = cluster->get(1).asInt32() / 2;
        pTmp.depth = person->get(2).asFloat64();
        pTmp.looking = person->get(3).asInt32() == 1 ? 1 : 0;
        pTmp.confidence = person->get(4).asFloat64();
        if (pTmp.looking && pTmp.depth < 1.2 && pTmp.depth > 0)
        {
            found = true;
            peopleDetected.push_back(pTmp);
        }
    }
    if (!found)
        return EyeContactStatus::NOT_LOOKING;

    // there is a person looking the robot
    double minDepth = peopleDetected[0].depth;
    Person result = peopleDetected[0];

    for (Person ptmp2 : peopleDetected)
    {
        if (ptmp2.depth < minDepth)
        {
            result = ptmp2;
            minDepth = ptmp2.depth;
        }
    }

    closestPerson = result;
    return EyeContactStatus::LOOKING;
}

bool eyeContactManager::updateModule()
{
    yarp::os::Bottle *peopleBottle = m_pGazeInput.read();
    Person closestPerson = Person();
    EyeContactStatus contactStatus = GetClosestPersonLooking(peopleBottle, closestPerson);

    switch (contactStatus)
    {
    case EyeContactStatus::LOOKING:
        m_iNav->getNavigationStatus(m_nav_status);

        if (m_nav_status != yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_moving)
        {
            lookAtPixel("track-face", closestPerson.x, closestPerson.y);
            m_lastEyeContactTime = yarp::os::Time::now();
            m_isHeadReset = false;
            //m_headSynchronizer.startHearing();
        }
        else
        {
            resetNeck();
        }
        break;
    case EyeContactStatus::NOT_LOOKING:
        //m_headSynchronizer.stopHearing();
        break;
    case EyeContactStatus::NOBODY:
        //m_headSynchronizer.stopHearing();
        break;
    default:
        break;
    }

    // reset the head if timeout expires without any eye contact
    if (yarp::os::Time::now() - m_lastEyeContactTime > 6.0)
    {
        resetNeck();
    }

    return true;
}

void eyeContactManager::resetNeck()
{
    if (!m_isHeadReset)
    {
        yarp::os::Property prop;
        yarp::os::Bottle b2;
        yarp::os::Bottle &b3 = b2.addList();
        b3.addFloat64(0.0);
        b3.addFloat64(0.0);
        prop.put("control-frame", "depth");
        prop.put("target-type", "angular");
        prop.put("target-location", b2.get(0));

        if (m_pGazeControl.write(prop))
        {
            m_isHeadReset = true;
            yCInfo(EYE_CONTACT) << "Reset neck position succesfully";
        }
        else
        {
            yCWarning(EYE_CONTACT) << "Failed to reset neck position";
        }
    }
}

bool eyeContactManager::configure(yarp::os::ResourceFinder &rf)
{
    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCWarning(EYE_CONTACT) << "Error! YARP Network is not initialized";
        return false;
    }

    if (!m_pGazeInput.open(m_gazeInputName))
    {
        yCWarning(EYE_CONTACT) << "Error! Cannot open gazeInput port";
        return false;
    }

    if (!m_pEyeContactOutput.open(m_eyeContactOutputName))
    {
        yCWarning(EYE_CONTACT) << "Error! Cannot open eyeContactOutputName port";
        return false;
    }
    m_headSynchronizer.yarp().attachAsClient(m_pEyeContactOutput);

    if (!m_pGazeControl.open(m_gazeControlName))
    {
        yCWarning(EYE_CONTACT) << "Error! Cannot open gazeControl port";
        return false;
    }

    // open the navigation interface
    yarp::os::Property nav_options;
    nav_options.put("device", "navigation2D_nwc_yarp");
    nav_options.put("local", "/eyeContactManager/navigation2D_nwc_yarp");
    nav_options.put("navigation_server", m_remote_navigation);
    nav_options.put("map_locations_server", m_remote_map);
    nav_options.put("localization_server", m_remote_localization);
    if (m_pNav.open(nav_options) == false)
    {
        yCError(EYE_CONTACT) << "Unable to open navigation2D_nwc_yarp device";
        return false;
    }
    m_pNav.view(m_iNav);
    if (m_iNav == 0)
    {
        yCError(EYE_CONTACT) << "Unable to open navigation interface";
        return false;
    }

    yCInfo(EYE_CONTACT) << "Configuration succesful.";

    return true;
}

bool eyeContactManager::interruptModule()
{
    yCInfo(EYE_CONTACT) << "Interrupting your module, for port cleanup" << '\n';
    return true;
}

bool eyeContactManager::sendBufferTo(std::string message, yarp::os::BufferedPort<yarp::os::Bottle> &port)
{
    yarp::os::Bottle &output = port.prepare();
    output.clear();
    output.addString(message);
    port.write();
    return true;
}

bool eyeContactManager::close()
{
    m_pEyeContactOutput.close();
    m_pGazeInput.close();
    if (m_pNav.isValid())
        m_pNav.close();
    m_iNav = nullptr;
    return true;
}

bool eyeContactManager::lookAtPixel(std::string mode, double px, double py)
{
    yarp::os::Property p;

    if (mode.compare("look-around") == 0)
    {

        p.put("control-frame", "gaze");
        p.put("target-type", "angular");
    }
    else
    {
        p.put("control-frame", "depth");
        p.put("target-type", "image");
        p.put("image", "depth");
    }

    yarp::os::Bottle b2;
    yarp::os::Bottle &b3 = b2.addList();
    b3.addFloat64(px);
    b3.addFloat64(py);
    p.put("target-location", b2.get(0));
    return m_pGazeControl.write(p);
}
