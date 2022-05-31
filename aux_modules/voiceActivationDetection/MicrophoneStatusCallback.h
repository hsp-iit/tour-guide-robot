// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BEHAVIOR_TOUR_ROBOT_MICROPHONESTATUSCALLBACK_H
#define BEHAVIOR_TOUR_ROBOT_MICROPHONESTATUSCALLBACK_H

#include <yarp/os/TypedReaderCallback.h>
#include <yarp/os/LogStream.h>
#include "Interfaces/IAudioProcessorMicrophoneCloser.h"
#include "Interfaces/IAudioProcessorMicrophoneOpener.h"
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Vocab.h>
#include <memory>
#include "headSynchronizerRPC.h"

class MicrophoneStatusCallback : public yarp::os::TypedReaderCallback<yarp::os::Bottle>, public IAudioProcessorMicrophoneCloser
{
public:
    MicrophoneStatusCallback(std::string synchronizationRpcPortName);
    void addMicrophoneOpener(std::shared_ptr<IAudioProcessorMicrophoneOpener> iAudioProcessorMicrophoneOpener);
    using TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle &microphoneStatus) override;
    void closeMicrophone() override;
    bool close();
    bool init();

private:
    std::mutex m_mutex; /** Internal mutex. **/
    bool m_microphoneJustClosed{false};
    bool m_microphoneNeedsToStart{true};
    std::shared_ptr<IAudioProcessorMicrophoneOpener> m_audioProcessorMicrophoneOpener{nullptr};
    std::string m_headSynchronizerClientName;
    yarp::os::Port m_pHeadSynchronizerClient;
    headSynchronizerRPC m_headSynchronizer;
};

#endif // BEHAVIOR_TOUR_ROBOT_MICROPHONESTATUSCALLBACK_H
