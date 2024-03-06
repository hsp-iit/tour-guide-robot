#include "AudioCallback.h"

#include <iostream>
#include <vector>

YARP_LOG_COMPONENT(WAKEWORDDETECTOR, "tour_guide_robot.speech.wakeWordDetection.AudioCallback", yarp::os::Log::TraceType);


AudioCallback::AudioCallback(const std::string audioOutName,
                            const std::string accessKey,
                            const std::string modelPath,
                            const std::string keywordPath,
                            const float sensitivity) {
    m_audioOut.open(audioOutName);
    
    const char *keywords = keywordPath.c_str();
    pv_status_t porcupine_status = pv_porcupine_init(accessKey.c_str(), modelPath.c_str(), 1, &keywords, &sensitivity, &m_porcupine);
    std::string model_status = pv_status_to_string(porcupine_status);

    m_frameSize = pv_porcupine_frame_length();
    m_currentAudioSliceBuffer.resize(m_frameSize);

    if (porcupine_status != PV_STATUS_SUCCESS) {
        yCError(WAKEWORDDETECTOR) << "Model failed ot initialise with " << model_status;
        
        char **message_stack = NULL;
        int32_t message_stack_depth = 0;
        pv_status_t error_status = pv_get_error_stack(&message_stack, &message_stack_depth);
        if (error_status != PV_STATUS_SUCCESS) {
            yCError(WAKEWORDDETECTOR) <<  "Unable to get Porcupine error state with", pv_status_to_string(error_status);
            exit(1);
        }

        if (message_stack_depth > 0) {
            printPorcupineErrorMessage(message_stack, message_stack_depth);
        } else {
            yCError(WAKEWORDDETECTOR) << "No error stack received";
        }

        pv_free_error_stack(message_stack);
        exit(1);
    }

    yCDebug(WAKEWORDDETECTOR) << "Model Loaded successfully";
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
            m_sampleCounter = 0;
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
        yCDebug(WAKEWORDDETECTOR) <<  "keyword detected!!!!!!!!!!!";
        keyWordDetected = true;

        // resize buffer to contain a few previous slices of audio + have space for all remaining samples in the current audioframe
        remainingSamplesBufferIdx = m_frameSize * m_previousAudioBuffer.size();
        m_remainingSamplesBuffer.resize((m_frameSize * m_previousAudioBuffer.size()) + numSamplesInFrame - currentSampleIdx - 1, 999);
        for (size_t v = 0; v < m_previousAudioBuffer.size(); v++)
        {
            auto vec = m_previousAudioBuffer[v];
            std::copy(vec.begin(), vec.end(), m_remainingSamplesBuffer.begin() + (v * m_frameSize));
        }
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
    soundToSend.setFrequency(FREQUENCY);
    soundToSend.resize(m_remainingSamplesBuffer.size());
    for (size_t i = 0; i < m_remainingSamplesBuffer.size(); i++)
    {
        soundToSend.set(m_remainingSamplesBuffer.at(i), i);
    }
    m_audioOut.write();
}

void AudioCallback::printPorcupineErrorMessage(char **messageStack, int32_t messageStackDepth) {
    for (int32_t i = 0; i < messageStackDepth; i++) {
        yCError(WAKEWORDDETECTOR) << messageStack[i];
    }
}