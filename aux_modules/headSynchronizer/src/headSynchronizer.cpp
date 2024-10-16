#include <headSynchronizer.h>

YARP_LOG_COMPONENT(HEAD_SYNCHRONIZER, "behavior_tour_robot.aux_modules.head_synchronizer", yarp::os::Log::TraceType)

HeadSynchronizer::HeadSynchronizer(const std::string &name) : m_name(name),
                                                              m_period(0.2),
                                                              m_isSpeaking(false),
                                                              m_isError(false),
                                                              m_synthesisOutputName("/" + name + "/result:o"),
                                                              m_eyeContactName("/" + name + "/eyeContact/rpc"),
                                                              m_statusInputName("/" + name + "/googleStatus:i"),
                                                              m_microphoneStatusName("/" + name + "/microphoneStatus:i"),
                                                              m_microphoneOutputName("/" + name + "/microphone:o"),
                                                              m_playerStatusName("/" + name + "/playerStatus:i"),
                                                              m_playerOutputName("/" + name + "/player:o"),
                                                              m_faceOutputName("/" + name + "/face:o"),
                                                              m_headSynchronizerThriftPortName("/" + name + "/thrift:s")

{
}

bool HeadSynchronizer::configure(yarp::os::ResourceFinder &rf)
{
    m_statusCallback = new StatusCallback(this);

    if (!m_pStatusInput.open(m_statusInputName))
    {
        yCError(HEAD_SYNCHRONIZER, "Cannot open statusInput port");
        return false;
    }
    m_pStatusInput.useCallback(*m_statusCallback);

    if (!m_pSynthesisOutput.open(m_synthesisOutputName))
    {
        yCError(HEAD_SYNCHRONIZER, "Cannot open synthesisOutput port");
        return false;
    }

    if (!m_pMicrophoneStatus.open(m_microphoneStatusName))
    {
        yCError(HEAD_SYNCHRONIZER, "Cannot open microphoneStatus port");
        return false;
    }

    if (!m_pMicrophoneOutput.open(m_microphoneOutputName))
    {
        yCError(HEAD_SYNCHRONIZER, "Cannot open microphoneOutput port");
        return false;
    }

    if (!m_pPlayerStatus.open(m_playerStatusName))
    {
        yCError(HEAD_SYNCHRONIZER, "Cannot open playerStatus port");
        return false;
    }

    if (!m_pPlayerOutput.open(m_playerOutputName))
    {
        yCError(HEAD_SYNCHRONIZER, "Cannot open playerOutput port");
        return false;
    }

    if (!m_pFaceOutput.open(m_faceOutputName))
    {
        yCError(HEAD_SYNCHRONIZER, "Cannot open faceOutput port");
        return false;
    }

    // --------- Thrift interface server side config --------- //
    if (!m_headSynchronizerThriftPort.open(m_headSynchronizerThriftPortName))
    {
        yCWarning(HEAD_SYNCHRONIZER, "Error! Cannot open the thrift port");
        return false;
    }

    if (!this->yarp().attachAsServer(m_headSynchronizerThriftPort))
    {
        yCWarning(HEAD_SYNCHRONIZER, "Error! Cannot attach port %s as a server", m_headSynchronizerThriftPortName.c_str());
        return false;
    }

    yCInfo(HEAD_SYNCHRONIZER) << "Configuration done!";
    return true;
}

bool HeadSynchronizer::close()
{
    delete m_statusCallback;
    m_pEyeContact.close();
    m_pStatusInput.close();
    m_pSynthesisOutput.close();
    m_pMicrophoneStatus.close();
    m_pMicrophoneOutput.close();
    m_pPlayerStatus.close();
    m_pPlayerOutput.close();
    m_pFaceOutput.close();
    m_headSynchronizerThriftPort.close();
    return true;
}

double HeadSynchronizer::getPeriod()
{
    return m_period;
}

bool HeadSynchronizer::interruptModule()
{
    return true;
}

bool HeadSynchronizer::updateModule()
{
    if (!isAudioPlaying())
    {
        std::unique_lock<std::mutex> lck(m_mutex);
        if (!m_textBuffer.empty())
        {
            std::string str = m_textBuffer.at(0);
            lck.unlock();
            if (isHearing())
            {
                stopHearing();
            }
            bool res = writeToPort(str, m_pSynthesisOutput);
            if (!res)
            {
                yCError(HEAD_SYNCHRONIZER) << "I failed to sent the text to SynthesisOutput.";
                return false;
            }
            yCDebug(HEAD_SYNCHRONIZER) << "I sent successfully to googleSynthesis:" << str;
            while (!isAudioPlaying())
            {
                yarp::os::Time::delay(0.1);
            }
            yCDebug(HEAD_SYNCHRONIZER) << "I started speaking:" << str;
            while (isAudioPlaying())
            {
                yarp::os::Time::delay(0.1);
            }
            yCDebug(HEAD_SYNCHRONIZER) << "I finished speaking:" << str;
            lck.lock();
            if (!m_textBuffer.empty()) // Needed check, as the buffer can be emptied async
            {
                m_textBuffer.erase(m_textBuffer.begin());
                yCDebug(HEAD_SYNCHRONIZER) << "I removed from the buffer the text:" << str;
            }
            lck.unlock();
        }
        else
        {
            m_isSpeaking = false;
        }
    }
    return true;
}

bool HeadSynchronizer::getIsError()
{
    return m_isError;
}

bool HeadSynchronizer::isAudioPlaying()
{
    yarp::sig::AudioPlayerStatus *playerStatus = m_pPlayerStatus.read();
    if (playerStatus)
    {
        return playerStatus->current_buffer_size > 0;
    }
    yCWarning(HEAD_SYNCHRONIZER) << "Player status failed to update. This should not happen. ";
    return false;
}

bool HeadSynchronizer::writeToPort(const std::string &s, yarp::os::Port &port)
{
    yarp::os::Bottle bot;
    bot.addString(s);
    return port.write(bot);
}

bool HeadSynchronizer::writeToPort(const std::string &s, yarp::os::Port &port, yarp::os::Bottle &res)
{
    yarp::os::Bottle bot;
    bot.addString(s);
    return port.write(bot, res);
}

bool HeadSynchronizer::say(const std::string &s)
{
    m_isSpeaking = true;
    std::unique_lock<std::mutex> lck(m_mutex);
    m_textBuffer.push_back(s);
    lck.unlock();
    yCDebug(HEAD_SYNCHRONIZER) << "I added to the buffer the text:" << s;
    return true;
}

bool HeadSynchronizer::pauseSpeaking()
{
    yarp::os::Bottle bot;
    yarp::os::Bottle res;
    bot.addString("stop");
    bool result = m_pPlayerOutput.write(bot, res);
    if (result)
    {
        yCDebug(HEAD_SYNCHRONIZER) << "Successfully paused the speech.";
        return true;
    }
    else
    {
        yCError(HEAD_SYNCHRONIZER) << "Failed to pause the speech.";
        return false;
    }
}

bool HeadSynchronizer::continueSpeaking()
{
    yarp::os::Bottle bot;
    yarp::os::Bottle res;
    bot.addString("start");
    bool result = m_pPlayerOutput.write(bot, res);
    if (result)
    {
        yCDebug(HEAD_SYNCHRONIZER) << "Successfully paused the speech.";
        return true;
    }
    else
    {
        yCError(HEAD_SYNCHRONIZER) << "Failed to pause the speech.";
        return false;
    }
}

bool HeadSynchronizer::startHearing()
{
    if (!isHearing())
    {
        if (isAudioPlaying())
        {
            yCWarning(HEAD_SYNCHRONIZER) << "Microphone told to enable while still speaking.";
        }

        yarp::os::Bottle res;
        bool result = writeToPort("start", m_pMicrophoneOutput, res);

        if (result && res.get(0).asVocab32() == VOCAB_OK)
        {
            yCDebug(HEAD_SYNCHRONIZER) << "Microphone enabled successfully";
            colorEars(0, 128, 0);
            return true;
        }
        else
        {
            yCError(HEAD_SYNCHRONIZER) << "Microphone failed to enable.";
            return false;
        }
    }
    else
    {
        yCWarning(HEAD_SYNCHRONIZER) << "Can't enable microphone. It is not disabled.";
        colorEars(0, 128, 0);
        return true;
    }
}

bool HeadSynchronizer::stopHearing()
{
    if (isHearing())
    {
        yarp::os::Bottle res;
        bool result = writeToPort("stop", m_pMicrophoneOutput, res);

        if (result == true && res.get(0).asVocab32() == VOCAB_OK)
        {
            yCDebug(HEAD_SYNCHRONIZER) << "Microphone disabled successfully";
            colorEars(0, 0, 255);
            return true;
        }
        else
        {
            yCError(HEAD_SYNCHRONIZER) << "Microphone failed to disable.";
            return false;
        }
    }
    else
    {
        yCWarning(HEAD_SYNCHRONIZER) << "Can't disable microphone. It is not enabled.";
        colorEars(0, 0, 255);
        return true;
    }
}

bool HeadSynchronizer::isSpeaking()
{
    return m_isSpeaking;
}

bool HeadSynchronizer::isHearing()
{
    yarp::sig::AudioRecorderStatus *recorderStatus = m_pMicrophoneStatus.read();
    if (recorderStatus)
    {
        return recorderStatus->enabled;
    }
    return false;
}

bool HeadSynchronizer::reset()
{
    std::unique_lock<std::mutex> lck(m_mutex);
    m_textBuffer.clear();
    lck.unlock();
    yCDebug(HEAD_SYNCHRONIZER) << "Cleared the text buffer";
    writeToPort("clear", m_pPlayerOutput);
    while (isAudioPlaying())
    {
        yarp::os::Time::delay(0.1);
    }
    yCDebug(HEAD_SYNCHRONIZER) << "Audio player finished playing";
    happyFace();
    if (stopHearing())
    {
        yCInfo(HEAD_SYNCHRONIZER) << "Reset successfully";
        return true;
    }
    else
    {
        yCError(HEAD_SYNCHRONIZER) << "Reset failed";
        return false;
    }
}

bool HeadSynchronizer::changeEmotion(int i)
{
    yarp::os::Bottle bot;
    bot.addString("emotion");
    bot.addFloat32(i);
    bool result = m_pFaceOutput.write(bot);
    if (result)
    {
        yCDebug(HEAD_SYNCHRONIZER) << "Emotion changed successfully to:" << i;
        return true;
    }
    else
    {
        yCError(HEAD_SYNCHRONIZER) << "Emotion failed to change";
        return false;
    }
}

bool HeadSynchronizer::colorEars(int r, int g, int b)
{
    yarp::os::Bottle bot;
    bot.addString("color_ears");
    bot.addFloat32(r);
    bot.addFloat32(g);
    bot.addFloat32(b);
    bool result = m_pFaceOutput.write(bot);
    if (result)
    {
        yCDebug(HEAD_SYNCHRONIZER) << "Ears color changed successfully to:" << r << g << b;
        return true;
    }
    else
    {
        yCError(HEAD_SYNCHRONIZER) << "Ears color failed to change";
        return false;
    }
}

bool HeadSynchronizer::colorMouth(int r, int g, int b)
{
    yarp::os::Bottle bot;
    bot.addString("color_mouth");
    bot.addFloat32(r);
    bot.addFloat32(g);
    bot.addFloat32(b);
    bool result = m_pFaceOutput.write(bot);
    if (result)
    {
        yCDebug(HEAD_SYNCHRONIZER) << "Mouth color changed successfully to:" << r << g << b;
        return true;
    }
    else
    {
        yCError(HEAD_SYNCHRONIZER) << "Mouth color failed to change";
        return false;
    }
    return true;
}

bool HeadSynchronizer::sadFaceWarning()
{
    changeEmotion(2);
    colorMouth(238, 210, 2); // Yellow alert color code
    yCInfo(HEAD_SYNCHRONIZER) << "Changed to warning face";
    return true;
}

bool HeadSynchronizer::sadFaceError()
{
    changeEmotion(0);
    colorEars(255, 0, 0);
    colorMouth(255, 0, 0);
    m_isError = true;
    yCInfo(HEAD_SYNCHRONIZER) << "Changed to error face";
    return true;
}

bool HeadSynchronizer::busyFaceError()
{
    changeEmotion(2);
    colorEars(255, 0, 0);
    colorMouth(255, 0, 0);
    m_isError = true;
    yCInfo(HEAD_SYNCHRONIZER) << "Changed to error face";
    return true;
}

bool HeadSynchronizer::happyFace()
{
    changeEmotion(1);
    if (isHearing())
    {
        colorEars(0, 128, 0);
    }
    else
    {
        colorEars(0, 0, 255);
    }
    colorMouth(0, 128, 0);
    m_isError = false;
    yCInfo(HEAD_SYNCHRONIZER) << "Changed to happy face";
    return true;
}

bool HeadSynchronizer::busyFace()
{
    changeEmotion(2);
    colorMouth(255, 255, 255);
    yCInfo(HEAD_SYNCHRONIZER) << "Changed to busy face";
    return true;
}

StatusCallback::StatusCallback(HeadSynchronizer *headSynchronizer) : m_headSynchronizer(headSynchronizer)
{
}

void StatusCallback::onRead(yarp::os::Bottle &b)
{
    std::string status = b.get(0).asString();
    if (status.compare("Busy") == 0)
    {
        if (m_headSynchronizer->getIsError())
        {
            m_headSynchronizer->busyFaceError();
        }
        else
        {
            m_headSynchronizer->busyFace();
        }
        yCDebug(HEAD_SYNCHRONIZER) << "Received google status: Busy";
    }
    else if (status.compare("Empty_input") == 0 || status.compare("Empty") == 0)
    {
        m_headSynchronizer->happyFace();
        m_headSynchronizer->startHearing();
        yCDebug(HEAD_SYNCHRONIZER) << "Received google status: Empty";
    }
    else if ((status.compare("Done") == 0) || (status.compare("done") == 0))
    {
        if (m_headSynchronizer->getIsError())
        {
            m_headSynchronizer->sadFaceError();
        }
        else
        {
            m_headSynchronizer->happyFace();
        }
        yCDebug(HEAD_SYNCHRONIZER) << "Received google status: Done";
    }
    else if (status.find("Failure") != std::string::npos)
    {
        m_headSynchronizer->sadFaceError();
        m_headSynchronizer->startHearing();
        yCError(HEAD_SYNCHRONIZER) << "Received status: Failure.";
    }
    else
    {
        m_headSynchronizer->sadFaceError();
        yCError(HEAD_SYNCHRONIZER) << "Status received not recognized:" << status;
    }
}
