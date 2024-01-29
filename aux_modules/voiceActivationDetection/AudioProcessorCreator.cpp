// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <iostream>
#include "AudioProcessorCreator.h"

YARP_LOG_COMPONENT(VADAUDIOPROCESSORCREATOR, "behavior_tour_robot.voiceActivationDetection.AudioProcessorCreator", yarp::os::Log::TraceType)

bool AudioProcessorCreator::configure(yarp::os::ResourceFinder &rf)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    std::string filteredAudioPortOutName = rf.check("filtered_audio_output_port_name",
                                                    yarp::os::Value("/vad/audio:o"),
                                                    "The name of the output port for the filtered audio.")
                                               .asString();

    std::string audioPortIn = rf.check("audio_input_port_name", yarp::os::Value("/vad/audio:i"),
                                       "The name of the input port for the audio.")
                                  .asString();

    std::string microphonePortIn = rf.check("microphone_status_port", yarp::os::Value("/vad/microphone/status:i"),
                                            "The name of the input port for the microphone status.")
                                       .asString();

    std::string synchronizationRpcPort = rf.check("synchronization_rpc_port", yarp::os::Value("/vad/sync:o"),
                                                  "The name of the input port for the synchronization rpc port.")
                                             .asString();


    // I think this is not needed
    if (!rf.check("period", "refresh period of the rf module"))
    {
        yCDebug(VADAUDIOPROCESSORCREATOR) << "Using default 'period' parameter of " << PERIOD_DEFAULT << "s";
    }
    else
    {
        m_period = rf.find("period").asFloat64();
    }

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
        yCError(VADAUDIOPROCESSORCREATOR) << "cannot open port" << audioPortIn;
        return false;
    }

    m_audioProcessor = std::make_shared<AudioProcessor>(m_vadFrequency,
                                                        m_vadSampleLength,
                                                        m_vadAggressiveness,
                                                        m_bufferSize,
                                                        filteredAudioPortOutName);

    m_audioCallback = std::make_unique<AudioCallback>(m_audioProcessor);
    m_audioPort.useCallback(*m_audioCallback);
    m_audioProcessor->start();
    yCInfo(VADAUDIOPROCESSORCREATOR) << "Started";
    return true;
}

double AudioProcessorCreator::getPeriod()
{
    return m_period;
}

bool AudioProcessorCreator::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pHeadSynchronizerClient.close();
    m_microphoneStatusPort.close();
    m_audioPort.close();
    yCInfo(VADAUDIOPROCESSORCREATOR) << "Closing";
    return true;
}

bool AudioProcessorCreator::updateModule()
{
    return true;
}