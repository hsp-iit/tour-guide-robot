// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <cstdlib>

#include <iostream>
#include "AudioProcessor.h"

YARP_LOG_COMPONENT(VADAUDIOPROCESSOR, "behavior_tour_robot.voiceActivationDetection.AudioProcessor", yarp::os::Log::TraceType)

AudioProcessor::AudioProcessor(int vadFrequency,
                               int vadSampleLength,
                               int vadAggressiveness,
                               int bufferSize,
                               std::string filteredAudioPortOutName):
                               m_vadFrequency(vadFrequency),
                               m_vadSampleLength(vadSampleLength),
                               m_vadAggressiveness(vadAggressiveness),
                               m_bufferSize(bufferSize),
                               m_filteredAudioPortOutName(filteredAudioPortOutName){
}


void AudioProcessor::addSound(yarp::sig::Sound&& sound)  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_soundToProcess.push_back(std::move(sound));
}


bool AudioProcessor::threadInit(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_fvadObject = fvad_new();
    if (!m_fvadObject)
    {
        yCError(VADAUDIOPROCESSOR) << "Failed to created VAD object";
        return false;
    }
    /*
     * Changes the VAD operating ("aggressiveness") mode of a VAD instance.
     *
     * A more aggressive (higher mode) VAD is more restrictive in reporting speech.
     * Put in other words the probability of being speech when the VAD returns 1 is
     * increased with increasing mode. As a consequence also the missed detection
     * rate goes up.
     *
     * Valid modes are 0 ("quality"), 1 ("low bitrate"), 2 ("aggressive"), and 3
     * ("very aggressive"). The default mode is 0.
     *
     * Returns 0 on success, or -1 if the specified mode is invalid.
     */

    yCDebug(VADAUDIOPROCESSOR) << "m_vadAggressiveness" << m_vadAggressiveness;
    fvad_set_mode(m_fvadObject, m_vadAggressiveness);

    /*
     * Sets the input sample rate in Hz for a VAD instance.
     *
     * Valid values are 8000, 16000, 32000 and 48000. The default is 8000. Note
     * that internally all processing will be done 8000 Hz; input data in higher
     * sample rates will just be downsampled first.
     *
     * Returns 0 on success, or -1 if the passed value is invalid.
     */

    if (fvad_set_sample_rate(m_fvadObject, m_vadFrequency))
    {
        yCError(VADAUDIOPROCESSOR) << "Unsupported input frequency.";
        return false;
    }
    if (!m_filteredAudioOutputPort.open(m_filteredAudioPortOutName)){
        yCError(VADAUDIOPROCESSOR) << "cannot open port" << m_filteredAudioPortOutName;
        return false;
    }

    m_microphoneOpen = true;
    return true;
}


[[noreturn]] void AudioProcessor::run() {
    while(true) {
        if(m_soundToProcess.empty()) {
            usleep(1000.0);
        } else {
            processAudio(m_soundToProcess.front());
        }
    }
}


void AudioProcessor::threadRelease() {
    fvad_free(m_fvadObject);
    m_filteredAudioOutputPort.close();
    m_fvadObject = nullptr;
}


void AudioProcessor::processAudio(yarp::sig::Sound& inputSound){
    std::cout << "processAudio" << std::endl;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (inputSound.getFrequency() < m_vadFrequency)
    {
        yCError(VADAUDIOPROCESSOR) << "The frequency needs to be at least" << m_vadFrequency;
    }
    double sampleSecondsLength = (double) inputSound.getSamples() / inputSound.getFrequency();
    if (int(sampleSecondsLength*1000)  % m_vadSampleLength == 0 ) {
        if (m_microphoneOpen) {
            int numberOfSamplesPerPacket = m_vadSampleLength * (m_vadFrequency / 1000);
            int packetNumber = (int)(sampleSecondsLength*1000) / m_vadSampleLength;
            for (int packet = 0; packet < packetNumber ; packet++) {
                auto copiedSound = createVector();
                for (size_t index = (packet * numberOfSamplesPerPacket); index < (packet + 1) * numberOfSamplesPerPacket; index++) {
                    copiedSound->push_back(inputSound.get(index));
                }
                if (m_microphoneOpen) {
                    processPacket(copiedSound);
                }
            }
        }
    } else {
        yCDebug(VADAUDIOPROCESSOR) << "cannot split samples into packets. " << sampleSecondsLength;
    }
    m_soundToProcess.pop_front();
}

std::shared_ptr<std::vector<int16_t>> AudioProcessor::createVector() {
    return std::make_shared<std::vector<int16_t>>();
}

void AudioProcessor::processPacket(std::shared_ptr<std::vector<int16_t>> copiedSound) {
    /*
    * Calculates a VAD decision for an audio frame.
    *
    * `frame` is an array of `length` signed 16-bit samples. Only frames with a
    * length of 10, 20 or 30 ms are supported, so for example at 8 kHz, `length`
    * must be either 80, 160 or 240.
    *
    * Returns              : 1 - (active voice),
    *                        0 - (non-active Voice),
    *                       -1 - (invalid frame length).
    */
    // the vector is const so it doesn't modify copied sound
    int isTalking = fvad_process(m_fvadObject, copiedSound->data(), copiedSound->size());
    //better use cout since it doesn't pass through the network and it is a callback

    if (isTalking < 0)
    {
        yCWarning(VADAUDIOPROCESSOR) << "Invalid frame length.";
        if (m_soundDetected) {
            sendSound();
        }
        else {
            clearVector();
        }
    } else if (isTalking == 0) {
        // not talking, but taking padding in front and at the end of buffersize
        if(m_soundDetected) {
            if (m_paddingCurrentSize < m_bufferSize) {
                m_paddingCurrentSize++;
                m_soundToSend.push_back(copiedSound);
            }
            else {
                sendSound();
            }
        }
        else {
            if (m_soundToSend.size() == m_bufferSize) {
                m_soundToSend.pop_front();
            }

            m_soundToSend.push_back(copiedSound);
        }
    } else {
        m_soundToSend.push_back(copiedSound);
        if (!m_soundDetected){
            yCDebug(VADAUDIOPROCESSOR) << "started detecting voice activity";
        }
        m_soundDetected = true;
        m_paddingCurrentSize = 0;
    }
}


void AudioProcessor::openMicrophone() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_microphoneOpen = true;
}


void AudioProcessor::closeMicrophone() {
    m_microphoneOpen = false;
}


void AudioProcessor::sendSound() {
    m_soundDetected = false;
    m_paddingCurrentSize = 0;
    int numberOfSamplesPerPacket = m_vadSampleLength * (m_vadFrequency / 1000);

    yarp::sig::Sound& soundToSend = m_filteredAudioOutputPort.prepare();
    yCDebug(VADAUDIOPROCESSOR) << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> sound detected, sending on the port";

    soundToSend.resize(m_soundToSend.size() * numberOfSamplesPerPacket);
    soundToSend.setFrequency(m_vadFrequency);
    for (size_t packet = 0; packet < m_soundToSend.size(); packet++)
    {
        for(size_t index = 0; index < numberOfSamplesPerPacket; index++) {
            soundToSend.set((*(m_soundToSend[packet]))[index], (packet * numberOfSamplesPerPacket) + index);
        }
    }
    m_filteredAudioOutputPort.write();
    clearVector();
}


void AudioProcessor::clearVector() {
    m_soundToSend.clear();
}
