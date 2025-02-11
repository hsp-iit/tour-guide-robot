// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <cstdlib>

#include <iostream>
#include <cstdint>
#include <stdexcept>
#include "Detector.h"

YARP_LOG_COMPONENT(VADAUDIOPROCESSOR, "behavior_tour_robot.voiceActivationDetection.AudioProcessor", yarp::os::Log::TraceType)

Detector::Detector(int vadFrequency,
                    int gapAllowance,
                    bool saveGap,
                    float threshold,
                    int vadSavePriorToDetection,
                    const std::string modelPath,
                    std::string filteredAudioPortOutName,
                    std::string wakeWordClientPort):
                    m_vadFrequency(vadFrequency),
                    m_vadGapAllowance(gapAllowance),
                    m_vadSaveGap(saveGap),
                    m_vadThreshold(threshold),
                    m_vadSavePriorToDetection(vadSavePriorToDetection),
                    m_vadNumSamples(vadFrequency == 16000 ? 512 :
                                    vadFrequency == 8000 ? 256 :
                                    throw std::runtime_error("Unsupported sample rate " + std::to_string(vadFrequency)
                                                            + ", must be 8kHz or 16kHz")),
                    m_context((vadFrequency == 16000) ? 64 : 32, 0),
                    m_currentSoundBufferNorm(m_vadNumSamples, 0),
                    m_currentSoundBuffer(m_vadNumSamples, 0),
                    m_fillCount(0) {
    
    init_onnx_model(modelPath);

    m_input.resize(m_context.size() + m_vadNumSamples);
    m_input_node_dims[0] = 1;
    m_input_node_dims[1] = m_context.size() + m_currentSoundBuffer.size();

    m_state.resize(m_size_state);
    m_sr.resize(1);
    m_sr[0] = m_vadFrequency;

    reset_states();

    if (!m_rpcClientPort.open(wakeWordClientPort)){
        yCError(VADAUDIOPROCESSOR) << "cannot open port" << wakeWordClientPort;
    }
    m_rpcClient.yarp().attachAsClient(m_rpcClientPort);

    if (!m_filteredAudioOutputPort.open(filteredAudioPortOutName)){
        yCError(VADAUDIOPROCESSOR) << "cannot open port" << wakeWordClientPort;
    }
}

void Detector::init_engine_threads(int inter_threads, int intra_threads) {
    m_session_options.SetIntraOpNumThreads(intra_threads);
    m_session_options.SetInterOpNumThreads(inter_threads);
    m_session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
};

void Detector::init_onnx_model(const std::string& model_path) {
    init_engine_threads(1, 1);
    m_session = std::make_shared<Ort::Session>(m_env, model_path.c_str(), m_session_options);
};

void Detector::reset_states() {
    std::memset(m_state.data(), 0.0f, m_state.size() * sizeof(float));
};

void Detector::predict(const std::vector<float> &data) {
    // Create ort tensors
    std::copy(m_context.begin(), m_context.end(), m_input.begin());
    std::copy(m_currentSoundBuffer.begin(), m_currentSoundBuffer.end(), m_input.begin() + m_context.size()); 
    Ort::Value input_ort = Ort::Value::CreateTensor<float>(
        m_memory_info, m_input.data(), m_input.size(), m_input_node_dims, 2);
    Ort::Value state_ort = Ort::Value::CreateTensor<float>(
        m_memory_info, m_state.data(), m_state.size(), m_state_node_dims, 3);
    Ort::Value sr_ort = Ort::Value::CreateTensor<int64_t>(
        m_memory_info, m_sr.data(), m_sr.size(), m_sr_node_dims, 1);

    // Clear and add inputs
    m_ort_inputs.clear();
    m_ort_inputs.emplace_back(std::move(input_ort));
    m_ort_inputs.emplace_back(std::move(state_ort));
    m_ort_inputs.emplace_back(std::move(sr_ort));

    // Infer
    m_ort_outputs = m_session->Run(
        Ort::RunOptions{nullptr},
        m_input_node_names.data(), m_ort_inputs.data(), m_ort_inputs.size(),
        m_output_node_names.data(), m_output_node_names.size());

    // Output probability & update h,c recursively
    float speech_prob = m_ort_outputs[0].GetTensorMutableData<float>()[0];
    float *stateN = m_ort_outputs[1].GetTensorMutableData<float>();
    std::memcpy(m_state.data(), stateN, m_size_state * sizeof(float));

    bool isTalking = speech_prob > m_vadThreshold;
    if (isTalking) { 
        yCDebug(VADAUDIOPROCESSOR) << "Voice detected adding to send buffer";
        m_soundDetected = true;
        m_soundToSend.push_back(m_currentSoundBuffer);
        m_gapCounter = 0;
    } else {
        if (m_soundDetected)
        {
            ++m_gapCounter;
            if (m_gapCounter > m_vadGapAllowance)
            {
                yCDebug(VADAUDIOPROCESSOR) << "End of of speech";
                sendSound();
                m_soundToSend.clear();
                m_soundDetected = false;
                m_rpcClient.stop();
                reset_states();
            }
            else if (m_vadSaveGap)
            {
                m_soundToSend.push_back(m_currentSoundBuffer);
            }
        }
        else if (m_vadSavePriorToDetection > 0)
        {
            m_soundToSend.push_back(m_currentSoundBuffer);
            if (m_soundToSend.size() > m_vadSavePriorToDetection)
            {
                m_soundToSend.pop_front();
            }
        }
        
    }

    // copy last part into context for next input
    std::copy(
        m_currentSoundBuffer.end() - m_context.size(),
        m_currentSoundBuffer.end(),  
        m_context.begin()                     
    );
};


void Detector::onRead(yarp::sig::Sound& soundReceived) {
    size_t num_samples = soundReceived.getSamples();

    for (size_t i = 0; i < num_samples; i++)
    {
        m_currentSoundBuffer.at(m_fillCount) = soundReceived.get(i);
        m_currentSoundBufferNorm.at(m_fillCount) = static_cast<float>(soundReceived.get(i)) / INT16_MAX;
        ++m_fillCount;
        if (m_fillCount == m_currentSoundBuffer.size()) {
            predict(m_currentSoundBufferNorm);
            m_fillCount = 0;
        }
    } 
    
}


void Detector::sendSound() {
    int packetsWithSound = m_soundToSend.size();

    yarp::sig::Sound& soundToSend = m_filteredAudioOutputPort.prepare();
    yCDebug(VADAUDIOPROCESSOR) << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> sending recorded voice sound";


    int totalPackets = packetsWithSound < m_minSoundSize ? m_minSoundSize : packetsWithSound;
    int numSamples = m_currentSoundBuffer.size() * totalPackets;
    soundToSend.resize(numSamples);
    soundToSend.setFrequency(m_vadFrequency);
    for (size_t p = 0; p < packetsWithSound; ++p)
    {
        for (size_t i = 0; i < m_currentSoundBuffer.size(); i++)
        {
            soundToSend.set(m_soundToSend[p].at(i), i + (p * m_currentSoundBuffer.size()));
        }
    }

    // padding to minimum size
    for (size_t i = packetsWithSound * m_currentSoundBuffer.size(); i < numSamples; i++)
    {
        soundToSend.set(0, i);
    }
    
    m_filteredAudioOutputPort.write();
}

