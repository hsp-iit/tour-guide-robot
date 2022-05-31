// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <iostream>
#include "AudioCallback.h"



AudioCallback::AudioCallback(std::shared_ptr<IAudioProcessorFeeder> iAudioProcessorFeeder):
                             m_iAudioProcessorFeeder(iAudioProcessorFeeder)
                             {}


void AudioCallback::onRead(yarp::sig::Sound &soundReceived) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_iAudioProcessorFeeder->addSound(std::move(soundReceived));
}