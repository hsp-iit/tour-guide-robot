// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <iostream>
#include "MicrophoneStatusCallback.h"
YARP_LOG_COMPONENT(VADAUDIOMICROPHONESTATUSCALLBACK, "behavior_tour_robot.voiceActivationDetection.MicrophoneStatusCallback", yarp::os::Log::TraceType)

MicrophoneStatusCallback::MicrophoneStatusCallback(std::string synchronizationRpcPortName)
{
}

void MicrophoneStatusCallback::addMicrophoneOpener(
    std::shared_ptr<IAudioProcessorMicrophoneOpener> iAudioProcessorMicrophoneOpener)
{
    m_audioProcessorMicrophoneOpener = iAudioProcessorMicrophoneOpener;
}

bool MicrophoneStatusCallback::init()
{
    m_headSynchronizerClientName = "/vad/micCB/HeadSynchronizer/thrift:c";

    if (!m_headSynchronizer.yarp().attachAsClient(m_pHeadSynchronizerClient))
    {
        yCWarning(VADAUDIOMICROPHONESTATUSCALLBACK) << "Error! Cannot attach the port as a client";
        return false;
    }

    if (!m_pHeadSynchronizerClient.open(m_headSynchronizerClientName))
    {
        yCWarning(VADAUDIOMICROPHONESTATUSCALLBACK) << "Error! Cannot open YARP port";
        return false;
    }
    return true;
}

bool MicrophoneStatusCallback::close()
{
    m_pHeadSynchronizerClient.close();
    return true;
}

void MicrophoneStatusCallback::onRead(yarp::os::Bottle &microphoneStatus)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    // the first element of the bottle can be either ok or fail
    if (microphoneStatus.get(0).asVocab32() == yarp::os::createVocab32('f', 'a', 'i', 'l'))
    {
        if (m_microphoneJustClosed)
        {
            m_microphoneJustClosed = false;
            m_microphoneNeedsToStart = true;
            std::cout << "need to start" << std::endl;
        }
    }
    else
    {
        if (m_microphoneNeedsToStart && m_audioProcessorMicrophoneOpener != nullptr)
        {
            m_audioProcessorMicrophoneOpener->openMicrophone();
            m_microphoneNeedsToStart = false;
            std::cout << "opening microphone" << std::endl;
        }
    }
}

void MicrophoneStatusCallback::closeMicrophone()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::cout << "closing microphone" << std::endl;
    m_headSynchronizer.stopHearing();
    m_microphoneJustClosed = true;
}