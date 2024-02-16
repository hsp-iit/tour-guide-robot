#include "AudioCallback.h"

#include <iostream>
#include <vector>
#include <dlfcn.h>


AudioCallback::AudioCallback(const std::string audioOutName,
                            const std::string accessKey,
                            const std::string modelPath,
                            const std::string keywordPath,
                            const float sensitivity) {
    auto keywords = keywordPath.c_str(); // can be an array of c_str

    m_audioOut.open(audioOutName);
    
    pv_status_t porcupine_status = pv_porcupine_init(accessKey.c_str(), modelPath.c_str(), 1, &keywords, &sensitivity, &m_porcupine);
    auto model_status = pv_status_to_string(porcupine_status);

    m_frameSize = pv_porcupine_frame_length();
    m_currentAudioSliceBuffer.resize(m_frameSize);

    std::cout << "Model status: " << model_status << std::endl;
}

void AudioCallback::onRead(yarp::sig::Sound &soundReceived) {
    if (m_currentlyStreaming)
    {
        yarp::sig::Sound& soundToSend = m_audioOut.prepare();
        soundToSend = soundReceived;
        m_audioOut.write();
    }
    else {
        processFrame(soundReceived);
    }    
}

void AudioCallback::processFrame(yarp::sig::Sound &soundReceived) {
    const size_t num_samples = soundReceived.getSamples();
    
    int remainingSamplesBufferIdx = 0;

    for (size_t i = 0; i < num_samples; i++) {
        // accumulate all remaining samples for sending after keyword detected
        if (m_currentlyStreaming) {
            m_remainingSamplesBuffer.at(remainingSamplesBufferIdx) = soundReceived.get(i);
            ++remainingSamplesBufferIdx;
            continue;
        }
        
        m_currentAudioSliceBuffer.at(m_sampleCounter) = soundReceived.get(i);
        ++m_sampleCounter;
        
        if (m_sampleCounter == m_frameSize) {
            m_currentlyStreaming = processSliceOfFrame(num_samples, i, remainingSamplesBufferIdx);
            m_sampleCounter = 0; // start filling audio slice buffer from start again
        }
    }

    // send any remaining slices after the wake word detected
    if (m_currentlyStreaming && m_remainingSamplesBuffer.size() > 0) {
        sendRemainingSamples();
    }
}

bool AudioCallback::processSliceOfFrame(const size_t &numSamplesInFrame, int currentSampleIdx, int &remainingSamplesBufferIdx) {
    int32_t keyword_index = -1;
    pv_status_t status = pv_porcupine_process(m_porcupine, m_currentAudioSliceBuffer.data(), &keyword_index);

    auto porcupine_status = pv_status_to_string(status);

    bool keyWordDetected = false;
    if (keyword_index != -1) {
        std::cout <<  "keyword detected!!!!!!!!!!!" << std::endl;
        keyWordDetected = true;

        // resize buffer to contain a few previous slices of audio + have space for all remaining samples in the current audioframe
        remainingSamplesBufferIdx = m_frameSize * m_previousAudioBuffer.size();
        m_remainingSamplesBuffer.resize((m_frameSize * m_previousAudioBuffer.size()) + numSamplesInFrame - currentSampleIdx - 1, 999);
        for (size_t v = 0; v < m_previousAudioBuffer.size(); v++)
        {
            auto vec = m_previousAudioBuffer[v];
            std::copy(vec.begin(), vec.end(), m_remainingSamplesBuffer.begin() + (v * m_frameSize));
        }
        std::cout << m_remainingSamplesBuffer.size() << std::endl;
    }
    else {
        std::cout <<  "no keyword :(" << std::endl;
    }

    m_previousAudioBuffer.push_back(m_currentAudioSliceBuffer);
    if (m_previousAudioBuffer.size() > 5)
    {
        m_previousAudioBuffer.pop_front();
    }

    return keyWordDetected;
}

void AudioCallback::sendRemainingSamples() {
    yarp::sig::Sound& soundToSend = m_audioOut.prepare();
    soundToSend.setFrequency(16000);
    soundToSend.resize(m_remainingSamplesBuffer.size());
    for (size_t i = 0; i < m_remainingSamplesBuffer.size(); i++)
    {
        soundToSend.set(m_remainingSamplesBuffer.at(i), i);
    }
    m_audioOut.write();
}