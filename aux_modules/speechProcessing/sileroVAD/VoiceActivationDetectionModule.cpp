// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <iostream>
#include "VoiceActivationDetectionModule.h"

YARP_LOG_COMPONENT(VADAUDIOPROCESSORCREATOR, "behavior_tour_robot.voiceActivationDetection.AudioProcessorCreator", yarp::os::Log::TraceType)

bool VoiceActivationDetectionModule::configure(yarp::os::ResourceFinder &rf)
{
    std::string filteredAudioPortOutName = rf.check("filtered_audio_output_port_name",
                                                    yarp::os::Value("/vad/audio:o"),
                                                    "The name of the output port for the filtered audio.")
                                               .asString();

    std::string audioPortIn = rf.check("audio_input_port_name", yarp::os::Value("/vad/audio:i"),
                                       "The name of the input port for the audio.")
                                  .asString();

    std::string speechTimestampPortOutName = rf.check("speech_start_timestamp_output_port_name",
                                                yarp::os::Value("/vad/speech_start_timestamp:o"),
                                                "The name of the output port for speech start timestamps.")
                                            .asString();

    std::string wakeWordClientPort = rf.check("wake_word_client_port_name", yarp::os::Value("/vad/rpc:o"),
                                            "Name of rpc port to inform wake detector when audio clip is done")
                                       .asString();

    std::string vadServerPort = rf.check("vad_server_port_name", yarp::os::Value("/vad/rpc:i"),
                                                  "Name of the input port for  synchronization rpc port.")
                                             .asString();
    int8_t vadReenableKeyword = rf.check("vad_reenable_keyword", yarp::os::Value(0),
                                                  "If 1, reenable the keyword after each audio clip.")
                                             .asInt8();



    if (!rf.check("vad_frequency", "vad_frequency"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_frequency' parameter of " << VAD_FREQUENCY_DEFAULT;
    }
    else
    {
        m_vadFrequency = rf.find("vad_frequency").asInt32();
    }

    if (!rf.check("vad_threshold", "vad_threshold"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_threshold' parameter of " << VAD_THRESHOLD;
    }
    else
    {
        m_vadThreshold = rf.find("vad_threshold").asFloat32();
    }

    if (!rf.check("vad_gap_allowance", "vad_gap_allowance"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_gap_allowance' parameter of " << VAD_GAP_ALLOWANCE_DEFAULT;
    }
    else
    {
        m_vadGapAllowance = rf.find("vad_gap_allowance").asInt32();
    }

    if (!rf.check("vad_save_gap", "vad_save_gap"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_save_gap' parameter of " << VAD_SAVE_GAP;
    }
    else
    {
        m_vadSaveGap = rf.find("vad_save_gap").asBool();
    }

    if (!rf.check("vad_save_prior_to_detection", "vad_save_prior_to_detection"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_save_prior_to_detection' parameter of " << VAD_SAVE_PRIOR_TO_DETECTION;
    }
    else
    {
        m_vadSavePriorToDetection = rf.find("vad_save_prior_to_detection").asInt32();
    }

    if (!rf.check("vad_speech_prob_ema_alpha", "vad_speech_prob_ema_alpha"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_speech_prob_ema_alpha' parameter of " << VAD_SPEECH_PROB_EMA_ALPHA;
    }
    else
    {
        m_vadSpeechProbEmaAlpha = rf.find("vad_speech_prob_ema_alpha").asFloat32();
    }

    if (!rf.check("vad_stop_threshold_margin", "vad_stop_threshold_margin"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_stop_threshold_margin' parameter of " << VAD_STOP_THRESHOLD_MARGIN;
    }
    else
    {
        m_vadStopThresholdMargin = rf.find("vad_stop_threshold_margin").asFloat32();
    }

    if (!rf.check("model_path", "model_path"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'model_path' parameter of " << MODEL_PATH;
    }
    else
    {
        m_modelPath = rf.find("model_path").asString();
    }

    if (!m_audioPort.open(audioPortIn))
    {
        yCError(VADAUDIOPROCESSORCREATOR) << "cannot open port " << audioPortIn;
        return false;
    }


    m_audioProcessor = std::make_shared<Detector>(m_vadFrequency,
                                                    m_vadGapAllowance,
                                                    m_vadSaveGap,
                                                    m_vadThreshold,
                                                    m_vadSavePriorToDetection,
                                                    m_modelPath,
                                                    filteredAudioPortOutName,
                                                    speechTimestampPortOutName,
                                                    wakeWordClientPort,
                                                    m_vadSpeechProbEmaAlpha,
                                                    m_vadStopThresholdMargin,
                                                    vadReenableKeyword);

    m_audioPort.useCallback(*m_audioProcessor);

    if (!m_rpcPort.open(vadServerPort))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Cannot open port " << vadServerPort;
        return false;
    }
    m_rpcServer = std::make_unique<SileroVADServer>(m_audioProcessor);

    m_rpcServer->yarp().attachAsServer(m_rpcPort);

    yCInfo(VADAUDIOPROCESSORCREATOR) << "Started";
    return true;
}

bool VoiceActivationDetectionModule::close()
{
    m_audioPort.close();
    yCInfo(VADAUDIOPROCESSORCREATOR) << "Closing";
    return true;
}

bool VoiceActivationDetectionModule::updateModule()
{
    return true;
}