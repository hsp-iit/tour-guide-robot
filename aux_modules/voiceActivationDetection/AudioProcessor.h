// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause


#ifndef BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSOR_H
#define BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSOR_H

#include <yarp/sig/Sound.h>
#include <yarp/os/TypedReaderCallback.h>
#include <yarp/os/LogStream.h>
#include "Interfaces/IAudioProcessorMicrophoneCloser.h"

#include <functional>
#include <cmath>
#include <deque>
#include <memory>

#include <mutex>
#include <deque>
#include <fvad.h>
#include <yarp/os/BufferedPort.h>
#include <Interfaces/IAudioProcessorFeeder.h>
#include <Interfaces/IAudioProcessorMicrophoneOpener.h>

// if put here it is used also by main.cpp

class AudioProcessor: public yarp::os::Thread, public IAudioProcessorFeeder, public IAudioProcessorMicrophoneOpener{
public:
    AudioProcessor(int vadFrequency,
                   int vadSampleLength,
                   int vadAggressiveness,
                   int bufferSize,
                   std::string filteredAudioPortOutName,
                   std::shared_ptr<IAudioProcessorMicrophoneCloser> microphoneManager);
    void addSound(yarp::sig::Sound&& sound) override;

    [[noreturn]] void run() override;
    bool threadInit() override;
    void threadRelease() override;
    void openMicrophone() override;

private:

    int m_vadFrequency;
    int m_vadSampleLength;
    int m_vadAggressiveness;
    Fvad * m_fvadObject {nullptr}; /** The voice activity detection object. **/
    std::deque<std::shared_ptr<std::vector<int16_t>>> m_soundToSend; /** Internal sound buffer. **/
    std::mutex m_mutex; /** Internal mutex. **/
    int m_bufferSize;
    int m_paddingCurrentSize{0};
    bool m_soundDetected{false};
    std::string m_filteredAudioPortOutName;
    yarp::os::BufferedPort<yarp::sig::Sound> m_filteredAudioOutputPort; /** The output port for sending the filtered audio. **/
    bool m_microphoneOpen{false};
    std::shared_ptr<IAudioProcessorMicrophoneCloser> m_microphoneManager;
    std::deque<yarp::sig::Sound> m_soundToProcess;

    void processAudio(yarp::sig::Sound& inputSound);
    void processPacket(std::shared_ptr<std::vector<int16_t>> copiedSound);
    void sendSound();
    void clearVector();
    void closeMicrophone();
    std::shared_ptr<std::vector<int16_t>> createVector();
};

#endif //BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSOR_H
