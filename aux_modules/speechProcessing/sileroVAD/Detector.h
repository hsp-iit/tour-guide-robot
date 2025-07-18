// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause


#ifndef DETECTOR_H
#define DETECTOR_H

#include <yarp/sig/Sound.h>
#include <yarp/os/TypedReaderCallback.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>

#include <functional>
#include <cmath>
#include <deque>
#include <memory>

#include <mutex>
#include "onnxruntime_cxx_api.h"
#include <yarp/os/BufferedPort.h>
#include "WakeMsgs.h"

class Detector: public yarp::os::TypedReaderCallback<yarp::sig::Sound> {
public:
    Detector(int vadFrequency,
            int gapAllowance,
            bool saveGap,
            float threshold,
            int vadSavePriorToDetection,
            const std::string modelPath,
            std::string filteredAudioPortOutName,
            std::string wakeWordClientPort,
            int8_t vadReenableKeyword = 0);
    using TypedReaderCallback<yarp::sig::Sound>::onRead;
    void onRead(yarp::sig::Sound& soundReceived) override;
    float m_vadThreshold;
    int m_vadGapAllowance;

private:
    const int m_vadFrequency;
    const bool m_vadSaveGap;
    const int m_vadNumSamples;
    const int m_vadSavePriorToDetection;

    std::deque<std::vector<int16_t>> m_soundToSend;
    std::vector<float> m_currentSoundBufferNorm; // Normalised for silero model
    std::vector<int16_t> m_currentSoundBuffer; // Original for downstream processing/synthesis
    std::vector<float> m_context;
    int m_fillCount; // keep track of up to what index the buffer is full
    int m_vadReenableKeyword; // If 1, reenable the keyword after each audio clip
    bool m_soundDetected{false};
    std::string m_filteredAudioPortOutName;
    yarp::os::BufferedPort<yarp::sig::Sound> m_filteredAudioOutputPort; /** The output port for sending the filtered audio. **/
    std::deque<yarp::sig::Sound> m_soundToProcess;

    int m_gapCounter = 0;
    int m_minSoundSize = 0; // how many extra packets to pad, can help with transcription

    yarp::os::RpcClient m_rpcClientPort;
    WakeMsgs m_rpcClient;

    // Onnx model
    Ort::Env m_env;
    Ort::SessionOptions m_session_options;
    std::shared_ptr<Ort::Session> m_session = nullptr;
    Ort::AllocatorWithDefaultOptions m_allocator;
    Ort::MemoryInfo m_memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

    // Model Inputs
    std::vector<Ort::Value> m_ort_inputs;

    std::vector<const char *> m_input_node_names = {"input", "state", "sr"};
    std::vector<float> m_input;
    unsigned int m_size_state = 2 * 1 * 128; // It's FIXED.
    std::vector<float> m_state;
    std::vector<int64_t> m_sr;

    int64_t m_input_node_dims[2] = {};
    const int64_t m_state_node_dims[3] = {2, 1, 128};
    const int64_t m_sr_node_dims[1] = {1};

    // Model Outputs
    std::vector<Ort::Value> m_ort_outputs;
    std::vector<const char *> m_output_node_names = {"output", "stateN"};



    void processPacket();
    void sendSound();
    void init_engine_threads(int inter_threads, int intra_threads);
    void init_onnx_model(const std::string& model_path);
    void reset_states();
    void predict(const std::vector<float> &data);
};

#endif