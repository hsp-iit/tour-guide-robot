// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include "SpeechMicController.h"
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

YARP_LOG_COMPONENT(SPEECH_MIC_CONTROLLER, "tour_guide_robot.speechProcessing.SpeechMicController")


/* ------------------------------------------ SoundCatcher class ----------------------------------------------- */
bool SoundCatcher::configure(yarp::os::ResourceFinder& rf)
{
    std::string audiorecorderRPCPortName = "/soundCatcher/microphone:rpc";
    if(rf.check("rpc_microphone_port")) {audiorecorderRPCPortName = rf.find("rpc_microphone_port").asString();}

    if(!m_audiorecorderRPCPort.open(audiorecorderRPCPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to open RPC port to microphone");
        return false;
    }

     return true;
}

void SoundCatcher::onRead(yarp::sig::Sound& soundReceived) {
    std::lock_guard<std::mutex> lock(m_mutex);
    size_t num_samples = soundReceived.getSamples();
    yarp::os::Bottle red_rec{"isRecording_RPC"};
    yarp::os::Bottle reply;
    yarp::os::Bottle req_stop{"stopRecording_RPC"};
    reply.clear();
    m_audiorecorderRPCPort.write(red_rec, reply);
    // yCInfo(SPEECH_MIC_CONTROLLER, "isReconrding_RPC reply: %s", reply.toString().c_str());
    if(reply.get(1).asString() == "ok")
    {
        yCInfo(SPEECH_MIC_CONTROLLER, "Microphone is recording, stopping it");
        reply.clear();
        m_audiorecorderRPCPort.write(req_stop,reply);
    }
    else
    {
        yCInfo(SPEECH_MIC_CONTROLLER, "Microphone is not recording, no need to stop it");
    }

}

SoundCatcher::~SoundCatcher()
{
    m_audiorecorderRPCPort.close();
}


/* ------------------------------------------ BufferCatcher class ----------------------------------------------- */
bool BufferCatcher::configure(yarp::os::ResourceFinder& rf)
{
    std::string audiorecorderRPCPortName = "/bufferCatcher/microphone:rpc";
    if(rf.check("rpc_microphone_port")) {audiorecorderRPCPortName = rf.find("rpc_microphone_port").asString();}

    if(!m_audiorecorderRPCPort.open(audiorecorderRPCPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to open RPC port to microphone");
        return false;
    }

     return true;
}

void BufferCatcher::onRead(yarp::sig::AudioPlayerStatus& status) {
    std::lock_guard<std::mutex> lock(m_mutex);
    // yCInfo(SPEECH_MIC_CONTROLLER, "Received audio player status: %s", status.toString().c_str());
    if(status.current_buffer_size > 0 && !m_switchFlipped)
    {
        yCInfo(SPEECH_MIC_CONTROLLER, "Audio is playing, stopping microphone");
        yarp::os::Bottle red_rec{"isRecording_RPC"};
        yarp::os::Bottle reply;
        yarp::os::Bottle req_stop{"stopRecording_RPC"};
        reply.clear();
        m_audiorecorderRPCPort.write(red_rec, reply);
        yCInfo(SPEECH_MIC_CONTROLLER, "isReconrding_RPC reply: %s", reply.toString().c_str());
        if(reply.get(1).asString() == "ok")
        {
            yCInfo(SPEECH_MIC_CONTROLLER, "Microphone is recording, stopping it");
            reply.clear();
            m_audiorecorderRPCPort.write(req_stop,reply);
            if (!reply.isNull() && reply.get(0).asString() == "nack")
            {
                yCError(SPEECH_MIC_CONTROLLER, "BufferCatcher::onRead. Orchestrator returned NACK when restarting microphone.");
                return;
            }
            else if(reply.isNull())
            {
                yCError(SPEECH_MIC_CONTROLLER, "BufferCatcher::onRead. No reply received when restarting microphone.");
                return;
            }
        }
        else
        {
            yCInfo(SPEECH_MIC_CONTROLLER, "Microphone is not recording, no need to stop it");
        }
        m_switchFlipped = true;
    }
    else if(status.current_buffer_size == 0 && m_switchFlipped)
    {
        yCInfo(SPEECH_MIC_CONTROLLER, "Audio finished playing, restarting microphone");
        yarp::os::Bottle req_start{"startRecording_RPC"};
        yarp::os::Bottle reply;
        m_audiorecorderRPCPort.write(req_start,reply);
        if (!reply.isNull() && reply.get(0).asString() == "nack")
        {
            yCError(SPEECH_MIC_CONTROLLER, "BufferCatcher::onRead. Orchestrator returned NACK when restarting microphone.");
            return;
        }
        else if(reply.isNull())
        {
            yCError(SPEECH_MIC_CONTROLLER, "BufferCatcher::onRead. No reply received when restarting microphone.");
            return;
        }
        m_switchFlipped = false;
    }
}

BufferCatcher::~BufferCatcher()
{
    m_audiorecorderRPCPort.close();
}


/* ------------------------------------------ TranscriptionCatcher class ----------------------------------------------- */
bool TranscriptionCatecher::configure(yarp::os::ResourceFinder& rf)
{
    std::string audiorecorderRPCPortName = "/transcriptionCatcher/microphone:rpc";
    if(rf.check("rpc_microphone_port")) {audiorecorderRPCPortName = rf.find("rpc_microphone_port").asString();}

    if(!m_audiorecorderRPCPort.open(audiorecorderRPCPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to open RPC port to microphone");
        return false;
    }

     return true;
}

void TranscriptionCatecher::onRead(yarp::os::Bottle& transcription) {
    std::lock_guard<std::mutex> lock(m_mutex);
    // yCInfo(SPEECH_MIC_CONTROLLER, "Received transcription: %s", transcription.toString().c_str());
    if(transcription.get(0).asString() == "")
    {
        yCInfo(SPEECH_MIC_CONTROLLER, "Empty transcription received, ignoring");
        yarp::os::Bottle req_start{"startRecording_RPC"};
        yarp::os::Bottle reply;
        m_audiorecorderRPCPort.write(req_start,reply);
        if (!reply.isNull() && reply.get(0).asString() == "nack")
        {
            yCError(SPEECH_MIC_CONTROLLER, "BufferCatcher::onRead. Orchestrator returned NACK when restarting microphone.");
            return;
        }
        else if(reply.isNull())
        {
            yCError(SPEECH_MIC_CONTROLLER, "BufferCatcher::onRead. No reply received when restarting microphone.");
            return;
        }
    }

    return;
}

TranscriptionCatecher::~TranscriptionCatecher()
{
    m_audiorecorderRPCPort.close();
}


/* ------------------------------------------ SpeechMicController class ----------------------------------------------- */
bool SpeechMicController::configure(yarp::os::ResourceFinder& rf)
{
    yCInfo(SPEECH_MIC_CONTROLLER, "Configuring SpeechMicController module");
    if(!m_soundCatcher.configure(rf))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to configure SoundCatcher");
        return false;
    }
    if(!m_bufferCatcher.configure(rf))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to configure BufferCatcher");
        return false;
    }

    if(!m_transcriptionCatcher.configure(rf))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to configure TranscriptionCatcher");
        return false;
    }

    std::string soundPortName = "/speechMicController/sound:i";
    if(rf.check("sound_port")) {soundPortName = rf.find("sound_port").asString();}

    if(!m_soundPort.open(soundPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to open sound port");
        return false;
    }
    m_soundPort.useCallback(m_soundCatcher);

    std::string audioPlayPortName = "/speechMicController/audioPlayStatus:i";
    if(rf.check("audio_play_status_port")) {audioPlayPortName = rf.find("audio_play_status_port").asString();}

    if(!m_audioPlayPort.open(audioPlayPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to open audio player status port");
        return false;
    }
    m_audioPlayPort.useCallback(m_bufferCatcher);

    std::string transcriptionPortName = "/speechMicController/transcription:i";
    if(rf.check("transcription_port")) {transcriptionPortName = rf.find("transcription_port").asString();}
    if(!m_transcriptionPort.open(transcriptionPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to open transcription port");
        return false;
    }
    m_transcriptionPort.useCallback(m_transcriptionCatcher);

    return true;
}

bool SpeechMicController::close()
{
    yCInfo(SPEECH_MIC_CONTROLLER, "Closing SpeechMicController module");
    m_soundPort.close();
    m_audioPlayPort.close();
    return true;
}

double SpeechMicController::getPeriod()
{
    return 1.0;
}

bool SpeechMicController::updateModule()
{
    return true;
}