/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "yarp/sig/Sound.h"

#include "SpeechToTextComponent.hpp"

using namespace std::chrono_literals;

bool SpeechToTextComponent::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    // --------------------- SPEECH TRANSCRIPTION NWC----------------------------
    bool okCheck = rf.check("SPEECHTRANSCRIPTION-CLIENT");
    std::string device = "speechTranscription_nwc_yarp";
    std::string local = "/SpeechToTextComponent/speechClient";
    std::string remote = "/speechTranscription_nws";

    if (okCheck)
    {
        yarp::os::Searchable &speech_config = rf.findGroup("SPEECHTRANSCRIPTION-CLIENT");
        if (speech_config.check("device"))
        {
            device = speech_config.find("device").asString();
        }
        if (speech_config.check("local-suffix"))
        {
            local = "/SpeechToTextComponent" + speech_config.find("local-suffix").asString();
        }
        if (speech_config.check("remote"))
        {
            remote = speech_config.find("remote").asString();
        }
    }

    yarp::os::Property prop;
    prop.put("device", device);
    prop.put("local", local);
    prop.put("remote", remote);

    m_speechTranscrPoly.open(prop);
    if (!m_speechTranscrPoly.isValid())
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Error opening speech synthesizer Client PolyDriver. Check parameters";
        return false;
    }
    m_speechTranscrPoly.view(m_iSpeechTranscr);
    if (!m_iSpeechTranscr)
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Error opening iSpeechSynth interface. Device not available";
        return false;
    }

    // --------------------- Input and output ports ----------------------------
    okCheck = rf.check("SPEECH_TO_TEXT_COMPONENT");
    std::string audioInputPort = "/SpeechToTextComponent/audio:i";
    std::string transcriptionOutputPort = "/SpeechToTextComponent/text:o";

    if (okCheck)
    {
        yarp::os::Searchable &speakersConfig = rf.findGroup("SPEECH_TO_TEXT_COMPONENT");
        if (speakersConfig.check("audioInputPort"))
        {
            audioInputPort = "/SpeechToTextComponent" + speakersConfig.find("audioInputPort").asString();
        }
        if (speakersConfig.check("transcriptionOutputPort"))
        {
            transcriptionOutputPort = "/SpeechToTextComponent" + speakersConfig.find("transcriptionOutputPort").asString();
        }
    }

    if(!m_audioInputPort.open(audioInputPort))
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Unable to open port: " << audioInputPort;
        return false;
    }
    m_audioInputPort.useCallback(*this);

    if(!m_transcriptionOutputPort.open(transcriptionOutputPort))
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Unable to open port: " << transcriptionOutputPort;
        return false;
    }
    // --------------------- AUDIO RECORDER NWC----------------------------
    okCheck = rf.check("MICROPHONE");
    device = "audioRecorder_nwc_yarp";
    local = "/SpeechToTextComponent/audio";
    remote = "/audioRecorder_nws";

    if (okCheck)
    {
        yarp::os::Searchable &mic_config = rf.findGroup("MICROPHONE");
        if (mic_config.check("device"))
        {
            device = mic_config.find("device").asString();
        }
        if (mic_config.check("local-suffix"))
        {
            local = "/SpeechToTextComponent" + mic_config.find("local-suffix").asString();
        }
        if (mic_config.check("remote"))
        {
            remote = mic_config.find("remote").asString();
        }
    }

    yarp::os::Property audioRecorder_prop;
    audioRecorder_prop.put("device", device);
    audioRecorder_prop.put("local", local);
    audioRecorder_prop.put("remote", remote);

    m_audioRecorderPoly.open(audioRecorder_prop);
    if (!m_audioRecorderPoly.isValid())
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Error opening audioRecorder Client PolyDriver. Check parameters";
        return false;
    }
    m_audioRecorderPoly.view(m_iAudioGrabberSound);
    if (!m_iAudioGrabberSound)
    {
        yError() << "[SpeechToTextComponent::ConfigureYARP] Error opening audioRecorderSound interface. Device not available";
        return false;
    }

    return true;
}

bool SpeechToTextComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("SpeechToTextComponentNode");

    RCLCPP_INFO(m_node->get_logger(), "Started node");
    return true;
}

bool SpeechToTextComponent::close()
{
    rclcpp::shutdown();
    return true;
}

void SpeechToTextComponent::spin()
{
    rclcpp::spin(m_node);
}

void SpeechToTextComponent::onRead(yarp::sig::Sound &msg)
{
    bool isRecording;
    if(!m_iAudioGrabberSound->isRecording(isRecording))
    {
        yError() << "[SpeechToTextComponent::onRead] Error checking if audio is recording";
        return;
    }
    if(isRecording)
    {
        m_iAudioGrabberSound->stopRecording();
        yInfo() << "[SpeechToTextComponent::onRead] Stopping the recording";
    }
    if (m_iSpeechTranscr)
    {
        yarp::os::Bottle& outputText = m_transcriptionOutputPort.prepare();
        outputText.clear();
        std::string transcriptionText;
        double confidence;
        if(!m_iSpeechTranscr->transcribe(msg, transcriptionText, confidence))

        {
            yError() << "[SpeechToTextComponent::onRead] Error transcribing audio";
            return;
        }
        yInfo() << "[SpeechToTextComponent::onRead] Transcription: " << transcriptionText << " with confidence: " << confidence;
        outputText.addString(transcriptionText);
        m_transcriptionOutputPort.write();
    }
    else
    {
        yError() << "[SpeechToTextComponent::onRead] Error opening iSpeechSynth interface. Device not available";
    }
}
