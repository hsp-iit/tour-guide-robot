#include <tourManager.h>

YARP_LOG_COMPONENT(TOUR_MANAGER, "behavior_tour_robot.aux_modules.tourmanager", yarp::os::Log::TraceType)

TourManager::TourManager(const std::string &name, const std::string &pathJSONTours, const std::string &pathJSONMovements, const std::string &tourName) : m_name(name),
                                                                                                                                                         m_period(1.0),
                                                                                                                                                         m_random_gen(m_rand_engine()),
                                                                                                                                                         m_uniform_distrib(1, 2),
                                                                                                                                                         m_fallback_repeat_counter(0),
                                                                                                                                                         m_fallback_threshold(3),
                                                                                                                                                         m_hasReachedPoI(false),
                                                                                                                                                         m_isFirstStart(true),
                                                                                                                                                         m_isSpeechInterrupted(false),
                                                                                                                                                         m_defaultLanguage("setLanguage_it-IT-Wavenet-A"),
                                                                                                                                                         m_pHeadSynchronizerName("/" + name + "/text:o"),
                                                                                                                                                         m_speechName("/" + name + "/speech/rpc"),
                                                                                                                                                         m_dialogName("/" + name + "/dialog/rpc"),
                                                                                                                                                         m_synthesisName("/" + name + "/synthesis/rpc"),
                                                                                                                                                         m_dialogflowOutputName("/" + name + "/dialogDialogOutput"),
                                                                                                                                                         m_dialogflowInputName("/" + name + "/googleDialogInput"),
                                                                                                                                                         m_tourManagerThriftPortName("/" + name + "/thrift:s"),
                                                                                                                                                         m_PoIndex(0)

{
    m_tourStorage = &TourStorage::GetInstance(pathJSONTours, tourName); // Loads the tour json from the file and saves a reference to the class.
    m_moveStorage = &MovementStorage::GetInstance(pathJSONMovements);   // Loads the movements json from the file and saves a reference to the class.
}

bool TourManager::configure(yarp::os::ResourceFinder &rf)
{
    m_dialogflowCallback = new DialogflowCallback(this);

    m_tourStorage->GetTour().setCurrentLanguage("it-IT");
    UpdatePoI();

    if (!m_tourStorage->GetTour().getPoI("___generic___", m_genericPoI))
    {
        yCError(TOUR_MANAGER) << "Generic PoI failed to update for the first time.";
        return false;
    }

    if (!m_pDialogflowInput.open(m_dialogflowInputName))
    {
        yCError(TOUR_MANAGER, "Cannot open dialogflowInput port");
        return false;
    }
    m_pDialogflowInput.useCallback(*m_dialogflowCallback);
    yarp::os::Network::connect("/googleDialog/result:o", m_dialogflowInputName);

    if (!m_pHeadSynchronizer.open(m_pHeadSynchronizerName))
    {
        yCError(TOUR_MANAGER, "Cannot open HeadSynchronizer port");
        return false;
    }
    m_headSynchronizer.yarp().attachAsClient(m_pHeadSynchronizer);
    yarp::os::Network::connect(m_pHeadSynchronizerName, "/HeadSynchronizer/thrift:s");

    if (!m_pDialog.open(m_dialogName))
    {
        yCError(TOUR_MANAGER, "Cannot open Dialog rpc port");
        return false;
    }
    m_Dialog.yarp().attachAsClient(m_pDialog);
    yarp::os::Network::connect(m_dialogName, "/googleDialog/rpc");

    if (!m_pSynthesis.open(m_synthesisName))
    {
        yCError(TOUR_MANAGER, "Cannot open Synthesis rpc port");
        return false;
    }
    m_Synthesis.yarp().attachAsClient(m_pSynthesis);
    yarp::os::Network::connect(m_synthesisName, "/googleSynthesis/rpc");

    if (!m_pSpeech.open(m_speechName))
    {
        yCError(TOUR_MANAGER, "Cannot open Speech rpc port");
        return false;
    }
    m_speech.yarp().attachAsClient(m_pSpeech);
    yarp::os::Network::connect(m_speechName, "/googleSpeech/rpc");

    if (!m_pDialogflowOutput.open(m_dialogflowOutputName))
    {
        yCError(TOUR_MANAGER, "Cannot open dialogflowOutput port");
        return false;
    }
    yarp::os::Network::connect(m_dialogflowOutputName, "/googleDialog/text:i");

    // Ctp Service
    std::set<std::string> ctpServiceParts = m_moveStorage->GetMovementsContainer().GetPartNames();
    if (!ctpServiceParts.empty())
    {
        for (std::string part : ctpServiceParts)
        {
            yarp::os::Port *ctpPort = new yarp::os::Port;
            std::string portName = "/" + m_name + "/" + part + "/rpc";
            bool b = ctpPort->open(portName);
            if (!b)
            {
                yCError(TOUR_MANAGER) << "Cannot open" << part << " ctpService port";
                return false;
            }
            m_pCtpService.insert({part, *ctpPort});
            yarp::os::Network::connect(portName, "/ctpservice/" + part + "/rpc");
        }
    }
    else
    {
        yCWarning(TOUR_MANAGER) << "Movement part names are empty. No movements will be executed!";
    }

    // --------- Thrift interface server side config --------- //
    if (!m_tourManagerThriftPort.open(m_tourManagerThriftPortName))
    {
        yCWarning(TOUR_MANAGER, "Error! Cannot open the thrift port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_tourManagerThriftPort))
    {
        yCWarning(TOUR_MANAGER, "Error! Cannot attach port %s as a server", m_tourManagerThriftPortName.c_str());
        return false;
    }

    // --------- navigation2D_nwc_yarp config --------- //
    bool okNav = rf.check("NAVIGATION2D-CLIENT");
    std::string device = "navigation2D_nwc_yarp";
    std::string local = "/" + m_name + "/navClient";
    std::string navServer = "/navigation2D_nws_yarp";
    std::string mapServer = "/map2D_nws_yarp";
    std::string locServer = "/localization2D_nws_yarp";
    if (okNav)
    {
        yarp::os::Searchable &nav_config = rf.findGroup("NAVIGATION2D-CLIENT");
        if (nav_config.check("device"))
        {
            device = nav_config.find("device").asString();
        }
        if (nav_config.check("local-suffix"))
        {
            local = "/" + m_name + nav_config.find("local-suffix").asString();
        }
        if (nav_config.check("navigation_server"))
        {
            navServer = nav_config.find("navigation_server").asString();
        }
        if (nav_config.check("map_locations_server"))
        {
            mapServer = nav_config.find("map_locations_server").asString();
        }
        if (nav_config.check("localization_server"))
        {
            locServer = nav_config.find("localization_server").asString();
        }
    }

    yarp::os::Property nav2DProp;
    nav2DProp.put("device", device);
    nav2DProp.put("local", local);
    nav2DProp.put("navigation_server", navServer);
    nav2DProp.put("map_locations_server", mapServer);
    nav2DProp.put("localization_server", locServer);
    nav2DProp.put("period", 5);

    m_nav2DPoly.open(nav2DProp);
    if (!m_nav2DPoly.isValid())
    {
        yCError(TOUR_MANAGER) << "Error opening Nav Client PolyDriver. Check parameters";
        return false;
    }
    m_nav2DPoly.view(m_iNav2D);
    if (!m_iNav2D)
    {
        yCError(TOUR_MANAGER) << "Error opening iNav2D interface. Device not available";
        return false;
    }

    m_headSynchronizer.reset(); // Reset the status of the headSynchronizer for safety
    yCInfo(TOUR_MANAGER, "Configuration Done!");
    return true;
}

bool TourManager::close()
{
    m_pHeadSynchronizer.close();
    m_pDialogflowInput.close();
    delete m_dialogflowCallback;
    for (auto port : m_pCtpService)
    {
        delete &port.second;
    }
    if (m_nav2DPoly.isValid())
        m_nav2DPoly.close();
    return true;
}

double TourManager::getPeriod()
{
    return m_period;
}

bool TourManager::interruptModule()
{
    return true;
}

bool TourManager::updateModule()
{
    return true;
}

bool TourManager::InterpretCommand(const std::string &command)
{
    bool isOk = false;
    std::vector<Action> actions;
    std::string cmd;

    bool isCurrent = m_currentPoI.isCommandValid(command);
    bool isGeneric = m_genericPoI.isCommandValid(command);

    if (isCurrent || isGeneric) // If the command is available either in the current PoI or the generic ones
    {
        int cmd_multiples;
        if (isCurrent) // If it is in the current overwrite the generic
        {
            cmd_multiples = m_currentPoI.getCommandMultiplesNum(command);
        }
        else
        {
            cmd_multiples = m_genericPoI.getCommandMultiplesNum(command);
        }

        if (cmd_multiples > 1)
        {
            m_uniform_distrib.param(std::uniform_int_distribution<std::mt19937::result_type>::param_type(1, cmd_multiples));
            int index = m_uniform_distrib(m_random_gen) - 1;
            cmd = command;
            if (index != 0)
            {
                cmd = cmd.append(std::to_string(index));
            }
        }
        else // The is only 1 command. It cannot be 0 because we checked if the command is available at the beginning
        {
            cmd = command;
        }

        if (isCurrent)
        {
            isOk = m_currentPoI.getActions(cmd, actions);
        }
        else
        {
            isOk = m_genericPoI.getActions(cmd, actions);
        }
    }
    else // Command is not available anywhere, return error and skip
    {
        yCWarning(TOUR_MANAGER) << "Command" << command << "not supported in either the PoI or the generics list. Skipping...";
    }

    if (isOk && !actions.empty())
    {
        int actionIndex = 0;
        bool isCommandBlocking = true;
        Action lastNonSignalAction;

        while (actionIndex < actions.size())
        {
            std::vector<Action> tempActions;
            for (int i = actionIndex; i < actions.size(); i++)
            {
                tempActions.push_back(actions[i]);
                if (actions[i].getType() != ActionTypes::SIGNAL)
                {
                    lastNonSignalAction = actions[i];
                }
                if (actions[i].isBlocking())
                {
                    actionIndex = i + 1;
                    break;
                }
                else
                {
                    if (i == actions.size() - 1)
                    {
                        actionIndex = actions.size();
                        if (!lastNonSignalAction.isBlocking())
                        {
                            isCommandBlocking = false;
                        }
                    }
                }
            }

            bool containsSpeak = false;
            float danceTime = 0.0f;

            for (Action action : tempActions) // Loops through all the actions until the blocking one. Execute all of them
            {
                switch (action.getType())
                {
                case ActionTypes::SPEAK:
                {
                    // Speak, but make it invalid if it is a fallback or it is an error message
                    Speak(action.getParam(), (cmd != "fallback" && cmd.find("Error") == std::string::npos));
                    containsSpeak = true;
                    break;
                }
                case ActionTypes::DANCE:
                {
                    danceTime += DoDance(action.getParam()); // By adding we can guarantee that if there multiple dances without blocking we can wait the max amount of time of them.
                    break;
                }
                case ActionTypes::SIGNAL:
                {
                    Signal(action.getParam());
                    bool isDelay = action.getParam().find("delay_") != std::string::npos;

                    // Patch of code to handle a delay signal blocking the parallel execution
                    if (danceTime != 0.0f && isDelay)
                    {
                        danceTime -= std::stof(action.getParam().substr(action.getParam().find("_") + 1, std::string::npos)); // Delay for the specified time in seconds.
                        if (danceTime < 0.0f)
                        {
                            danceTime = 0.0f;
                        }
                    }
                    if (containsSpeak && !m_headSynchronizer.isSpeaking() && isDelay)
                    {
                        containsSpeak = false;
                    }
                    break;
                }
                case ActionTypes::INVALID:
                {
                    yCError(TOUR_MANAGER) << "I got an INVALID ActionType in command" << command;
                    break;
                }
                default:
                {
                    yCError(TOUR_MANAGER) << "I got an unknown ActionType.";
                    break;
                }
                }
            }

            if ((containsSpeak || danceTime != 0.0f) && isCommandBlocking) // Waits for the longest move in the temp list of blocked moves and speak. If there is nothing in the temp list because we are not blocking it is skipped.
            {
                while (containsSpeak && !m_headSynchronizer.isSpeaking())
                {
                    yarp::os::Time::delay(0.1);
                }
                double startTime = yarp::os::Time::now();
                while ((yarp::os::Time::now() - startTime) < danceTime)
                {
                    yarp::os::Time::delay(0.1);
                }
                while (m_headSynchronizer.isSpeaking())
                {
                    yarp::os::Time::delay(0.1);
                }
            }
        }

        if (cmd == "fallback")
        {
            m_fallback_repeat_counter++;
            if (m_fallback_repeat_counter == m_fallback_threshold)
            { // If the same command has been received as many times as the threshold, then repeat the question.
                Speak(m_last_valid_speak, true);
                BlockSpeak();
                m_fallback_repeat_counter = 0;
            }
            Signal("startHearing"); // Open the ears after we handled the fallback to get a response.
        }
        else
        {
            m_fallback_repeat_counter = 0;
        }
        return true;
    }
    return false;
}

bool TourManager::NextPoI()
{
    try
    {
        m_PoIndex++;
        m_PoIndex = m_PoIndex % m_tourStorage->GetTour().getPoIsList().size();
        return UpdatePoI();
    }
    catch (...)
    {
        yCError(TOUR_MANAGER) << "NextPoI failed to execute.";
        return false;
    }
}

bool TourManager::UpdatePoI()
{
    bool b = m_tourStorage->GetTour().getPoI(m_tourStorage->GetTour().getPoIsList()[m_PoIndex], m_currentPoI); // Loads the next poi in the active pois specified in the tour object.
    if (!b)
    {
        yCError(TOUR_MANAGER) << "UpdatePoI failed to execute. Is the poi name in the active poi's?";
        return false;
    }
    yCDebug(TOUR_MANAGER) << "Updated PoI successfully.";
    return true;
}

void TourManager::Speak(const std::string &text, bool isValid)
{
    if (m_headSynchronizer.say(text))
    {
        yCDebug(TOUR_MANAGER) << "I am playing:" << text;
    }
    else
    {
        yCError(TOUR_MANAGER) << "Failed to speak text given.";
    }
    if (isValid)
    {
        m_last_valid_speak = text;
    }
}

float TourManager::DoDance(const std::string &danceName)
{
    Dance currentDance;
    if (!m_moveStorage->GetMovementsContainer().GetDance(danceName, currentDance))
    {
        yCWarning(TOUR_MANAGER) << "Dance" << danceName << "not found. Skipping...";
        return 0.0f;
    }

    bool status;
    for (Movement currentMove : currentDance.GetMovements())
    {
        if (!m_moveStorage->GetMovementsContainer().GetPartNames().count(currentMove.GetPartName()))
        {
            yCWarning(TOUR_MANAGER) << "Part" << currentMove.GetPartName() << "not supported. Skipping...";
            continue;
        }

        yarp::os::Port &port = m_pCtpService.at(currentMove.GetPartName());
        status = SendMovement(currentMove.GetTime(), currentMove.GetOffset(), currentMove.GetJoints(), port);
        if (!status)
        {
            yCError(TOUR_MANAGER) << "Movement failed to sent. Is ctpService for part" << currentMove.GetPartName() << "running?";
            continue;
        }
    }
    yCDebug(TOUR_MANAGER) << "I danced:" << danceName << "with duration:" << currentDance.GetDuration();
    return currentDance.GetDuration();
}

void TourManager::Signal(const std::string &param)
{
    if (param == "startHearing")
    {
        m_headSynchronizer.startHearing(); // Open the microphone and listen
        yCDebug(TOUR_MANAGER) << "I started hearing.";
    }
    else if (param.find("setLanguage") != std::string::npos)
    { // Change the language to the specified one
        std::string voice = param.substr(param.find("_") + 1, std::string::npos);
        std::string language = voice.substr(0, voice.find("-", 4)); // Get a copy of the language part of the string after the first delimiter

        if (voice.empty() || language.empty())
        {
            yCError(TOUR_MANAGER) << "Failed to change language. Voice and language is empty";
            return;
        }

        m_speech.setLanguage(language);
        m_Dialog.setLanguage(language);
        std::string debug_text = m_Synthesis.setLanguage(language, voice);

        double startTime = yarp::os::Time::now();
        while (m_Synthesis.getLanguageCode() != language)
        {
            // Setting timeout of change language to 5 seconds
            if ((yarp::os::Time::now() - startTime) >= 5.0)
            {
                yCError(TOUR_MANAGER) << "Language failed to change in google and timed out." << debug_text;
                return;
            }
            yarp::os::Time::delay(0.1);
        }
        if (!m_tourStorage->GetTour().setCurrentLanguage(language))
        {
            yCError(TOUR_MANAGER) << "Language failed to change in TourStorage and timed out.";
            return;
        }
        yCDebug(TOUR_MANAGER) << "Changed language successfully to:" << language;
        if (!m_tourStorage->GetTour().getPoI("___generic___", m_genericPoI))
        {
            yCError(TOUR_MANAGER) << "Generic PoI failed update after changing language.";
            return;
        }
        UpdatePoI();
    }
    else if (param == "nextPoi") // Received by googleDialog
    {
        NextPoI();
    }
    else if (param == "reset")
    {
        m_PoIndex = 0;
        if (m_currentPoI.getName().find("_start") == std::string::npos)
        {
            m_isFirstStart = true;
        }
        m_headSynchronizer.reset();
        Signal(m_defaultLanguage);
        UpdatePoI();
        Signal("startHearing");
        yCDebug(TOUR_MANAGER) << "Reset TourManager and HeadSynchronizer successfully.";
    }
    else if (param.find("delay_") != std::string::npos)
    {
        float delay = std::stof(param.substr(param.find("_") + 1, std::string::npos)); // Delay for the specified time in seconds.
        yCDebug(TOUR_MANAGER) << "I am delaying for:" << delay << "seconds.";
        yarp::os::Time::delay(delay);
    }
    else
    {
        yCError(TOUR_MANAGER) << "Received an empty signal. That should never happen!";
    }
}

bool TourManager::recovered()
{
    m_headSynchronizer.reset();
    yarp::dev::Nav2D::NavigationStatusEnum currentStatus;
    m_iNav2D->getNavigationStatus(currentStatus);

    // If it was speaking and the robot is not about to move again
    if (currentStatus == yarp::dev::Nav2D::navigation_status_goal_reached)
    {
        if (m_isSpeechInterrupted)
        {
            Speak(m_last_valid_speak, true);
            BlockSpeak();
            m_isSpeechInterrupted = false;
        }
        // Start hearing only if the robot will not move. Otherwise it is expected to speak
        m_headSynchronizer.startHearing();
    }
    m_headSynchronizer.happyFace();
    return true;
}

bool TourManager::sendError(const std::string &error)
{
    yCError(TOUR_MANAGER) << "Received error:" << error;

    yarp::dev::Nav2D::NavigationStatusEnum currentStatus;
    m_iNav2D->getNavigationStatus(currentStatus);
    if (currentStatus != yarp::dev::Nav2D::navigation_status_idle || currentStatus != yarp::dev::Nav2D::navigation_status_goal_reached || currentStatus != yarp::dev::Nav2D::navigation_status_aborted)
    {
        m_iNav2D->stopNavigation(); // Can only stop navigation goals that have been sent by YARP. Can't stop override commands from rviz
    }

    if (m_headSynchronizer.isSpeaking())
    {
        m_headSynchronizer.reset();
        m_isSpeechInterrupted = true;
    }

    if (error == "NETWORK_ERROR")
    {
        InterpretCommand("networkError");
    }
    else if (error == "MOTORS_ERROR")
    {
        InterpretCommand("motorsError");
    }
    else if (error == "TOUCHED_ERROR")
    {
        InterpretCommand("touchedError");
    }
    else if (error == "LOCALIZATION_ERROR")
    {
        InterpretCommand("localizationError");
    }
    else if (error == "GOAL_ERROR")
    {
        InterpretCommand("goalNotAvailableError");
    }
    else
    {
        yCError(TOUR_MANAGER) << "Received an error:" << error << ", which I don't handle!";
    }
    m_headSynchronizer.sadFaceError();

    return true;
}

bool TourManager::isAtPoI()
{
    // Check if hasReachedPoI so that we can override manually without depending on the navigation status
    if (m_hasReachedPoI && m_currentPoI.getName() == m_previousPoIname)
    {
        return true;
    }
    return false;
}

bool TourManager::sendToPoI()
{
    m_hasReachedPoI = false;

    yarp::dev::Nav2D::Map2DLocation current_target_coord;
    if (!m_iNav2D->getLocation(m_currentPoI.getName(), current_target_coord))
    {
        yCError(TOUR_MANAGER) << "Could not get next location coordinates for PoI" << m_currentPoI.getName();
        return false;
    }

    bool isDifferentPoI = current_target_coord != m_previousPoIloc;
    if (isDifferentPoI)
    {
        std::string validSpeakTmp = m_last_valid_speak;
        // if the poi is at different coordinates, move. Else skip.
        if (!m_isFirstStart)
        {
            InterpretCommand("warningNextNavigation");
        }

        // Set to navigation position
        float duration = DoDance("navigationPosition");

        // Wait for the navigation dance to finish executing
        double startTime = yarp::os::Time::now();
        while ((yarp::os::Time::now() - startTime) < duration)
        {
            yarp::os::Time::delay(0.1);
        }

        m_iNav2D->gotoTargetByLocationName(m_currentPoI.getName());
        yCDebug(TOUR_MANAGER) << "Moving to next PoI:" << m_currentPoI.getName();

        yarp::dev::Nav2D::NavigationStatusEnum currentStatus;
        m_iNav2D->getNavigationStatus(currentStatus);
        startTime = yarp::os::Time::now();
        while (currentStatus != yarp::dev::Nav2D::navigation_status_moving)
        {
            // Setting timeout of navigation status change to 2 seconds
            if ((yarp::os::Time::now() - startTime) >= 2.0)
            {
                yCError(TOUR_MANAGER) << "Navigation command failed to reach move_base and timed out.";
                return false;
            }
            m_iNav2D->getNavigationStatus(currentStatus);
            yarp::os::Time::delay(0.1);
        }

        // If it was interrupted during previous navigation, only repeat the last sentence that is relevant
        if (m_isSpeechInterrupted)
        {
            Speak(validSpeakTmp, true);
            BlockSpeak();
            m_isSpeechInterrupted = false;
        }
        else
        {
            InterpretCommand("sayWhileNavigating");
        }
        m_headSynchronizer.happyFace();

        while (currentStatus != yarp::dev::Nav2D::navigation_status_goal_reached)
        {
            m_iNav2D->getNavigationStatus(currentStatus);
            if (currentStatus == yarp::dev::Nav2D::navigation_status_aborted)
            {
                if (m_headSynchronizer.isSpeaking())
                {
                    m_headSynchronizer.reset();
                }
                InterpretCommand("navigationError");
                m_headSynchronizer.sadFaceWarning();
                return false;
            }
            else if (currentStatus == yarp::dev::Nav2D::navigation_status_idle)
            {
                return false;
            }
            yarp::os::Time::delay(0.1);
        }
    }
    m_hasReachedPoI = true;
    m_previousPoIname = m_currentPoI.getName(); // Name is unique on every PoI
    m_previousPoIloc = current_target_coord;    // Coordinates of two pois can be the same. This is only used when we have two consecutive poi's on same location to skip movement

    if (m_currentPoI.getName().find("_start") != std::string::npos)
    {
        Signal(m_defaultLanguage);
        if (!m_isFirstStart)
        { // Don't send startpoi if it is the first time the tour is starting
            SendToDialogue("startpoi");
        }
        else
        {
            m_isFirstStart = false;
        }
        Signal("startHearing");
    }
    else
    {
        SendToDialogue("samelocation");
    }

    return true;
}

void TourManager::BlockSpeak()
{
    while (!m_headSynchronizer.isSpeaking())
    {
        yarp::os::Time::delay(0.1);
    }
    while (m_headSynchronizer.isSpeaking())
    {
        yarp::os::Time::delay(0.1);
    }
}

std::string TourManager::getCurrentPoIName()
{
    return m_currentPoI.getName();
}

void TourManager::SendToDialogue(const std::string &command)
{
    yarp::os::Bottle cmd;
    cmd.addString(command);
    m_pDialogflowOutput.write(cmd);
    yCDebug(TOUR_MANAGER) << "I am sending to dialogueFlow:" << command;
}

bool TourManager::SendMovement(float time, float offset, std::vector<float> joints, yarp::os::Port &port)
{
    yarp::os::Bottle res;
    yarp::os::Bottle cmd;
    cmd.addVocab32("ctpq");
    cmd.addVocab32("time");
    cmd.addFloat64(time);
    cmd.addVocab32("off");
    cmd.addFloat64(offset);
    cmd.addVocab32("pos");
    yarp::os::Bottle &list = cmd.addList();
    for (auto joint : joints)
    {
        list.addFloat64(joint);
    }
    return port.write(cmd, res);
}

DialogflowCallback::DialogflowCallback(TourManager *tourManager) : m_tourManager(tourManager)
{
}

void DialogflowCallback::onRead(yarp::os::Bottle &b)
{
    m_tourManager->InterpretCommand(b.toString());
}
