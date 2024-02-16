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

    if (!rf.check("buffer_size", "buffer_size"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'buffer_size' parameter of " << m_bufferSize;
    }
    else
    {
        m_bufferSize = rf.find("buffer_size").asInt32();
    }

    if (!m_audioPort.open(audioPortIn))
    {
        yCError(VADAUDIOPROCESSORCREATOR) << "cannot open port " << audioPortIn;
        return false;
    }

    
    m_rpcPort.open("/vad/rpc:i");

    m_audioProcessor = std::make_shared<Detector>(m_vadFrequency,
                                                    m_vadSampleLength,
                                                    m_vadAggressiveness,
                                                    m_bufferSize,
                                                    filteredAudioPortOutName,
                                                    wakeWordClientPort);

    m_rpc = std::make_unique<VADServer>(m_audioProcessor);
    m_rpc->yarp().attachAsServer(m_rpcPort);

    m_audioPort.useCallback(*m_audioProcessor);
    yCInfo(VADAUDIOPROCESSORCREATOR) << "Started";
    return true;
}

double VoiceActivationDetectionModule::getPeriod()
{
    return m_period;
}

bool VoiceActivationDetectionModule::close()
{
    m_audioPort.close();
    m_rpcPort.close();
    yCInfo(VADAUDIOPROCESSORCREATOR) << "Closing";
    return true;
}

bool VoiceActivationDetectionModule::updateModule()
{
    return true;
}