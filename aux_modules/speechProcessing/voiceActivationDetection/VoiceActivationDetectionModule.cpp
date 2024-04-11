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

    std::string wakeWordClientPort = rf.check("wake_wrod_client_port_name", yarp::os::Value("/vad/rpc:o"),
                                            "Name of rpc port to inform wake detector when audio clip is done")
                                       .asString();

    std::string vadServerPort = rf.check("vad_server_port_name", yarp::os::Value("/vad/rpc:i"),
                                                  "Name of the input port for  synchronization rpc port.")
                                             .asString();



    if (!rf.check("vad_frequency", "vad_frequency"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_frequency' parameter of " << VAD_FREQUENCY_DEFAULT;
    }
    else
    {
        m_vadFrequency = rf.find("vad_frequency").asInt32();
    }

    if (!rf.check("vad_aggressiveness", "vad_aggressiveness"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_aggressiveness' parameter of " << VAD_AGGRESSIVENESS_DEFAULT;
    }
    else
    {
        m_vadAggressiveness = rf.find("vad_aggressiveness").asInt32();
    }

    if (!rf.check("vad_sample_length", "vad_sample_length"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_sample_length' parameter of " << VAD_SAMPLE_LENGTH_DEFAULT;
    }
    else
    {
        m_vadSampleLength = rf.find("vad_sample_length").asInt32();
    }

    if (!rf.check("vad_gap_allowance", "vad_gap_allowance"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_gap_allowance' parameter of " << VAD_GAP_ALLOWANCE_DEFAULT;
    }
    else
    {
        m_vadGapAllowance = rf.find("vad_gap_allowance").asInt32();
    }

    if (!rf.check("vad_min_sound_out_size", "vad_min_sound_out_size"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'vad_min_sound_out_size' parameter of " << VAD_MIN_SOUND_OUT_SIZE_DEFAULT;
    }
    else
    {
        m_minSoundOutSize = rf.find("vad_min_sound_out_size").asInt32();
    }

    if (!m_audioPort.open(audioPortIn))
    {
        yCError(VADAUDIOPROCESSORCREATOR) << "cannot open port " << audioPortIn;
        return false;
    }

    
    m_audioProcessor = std::make_unique<Detector>(m_vadFrequency,
                                                    m_vadSampleLength,
                                                    m_vadAggressiveness,
                                                    m_vadGapAllowance,
                                                    m_minSoundOutSize,
                                                    filteredAudioPortOutName,
                                                    wakeWordClientPort);

    m_audioPort.useCallback(*m_audioProcessor);
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