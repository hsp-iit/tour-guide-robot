// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include "SpeechMicController.h"
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <thread>

YARP_LOG_COMPONENT(SPEECH_MIC_CONTROLLER, "tour_guide_robot.speechProcessing.SpeechMicController")


/* ------------------------------------------ SoundCatcher class ----------------------------------------------- */
bool SoundCatcher::configure(yarp::os::ResourceFinder& rf)
{
    std::string localRPCPortName = "/soundCatcher/microphone:rpc";
    if(rf.check("rpc_microphone_sound_local_port")) {localRPCPortName = rf.find("rpc_microphone_sound_local_port").asString();}

    std::string remoteRPCPortName = "/audioRecorder_nws/rpc";
    if(rf.check("rpc_microphone_remote_port")) {remoteRPCPortName = rf.find("rpc_microphone_remote_port").asString();}
    else if(rf.check("rpc_microphone_port")) {remoteRPCPortName = rf.find("rpc_microphone_port").asString();}
    if(rf.check("sound_stop_cooldown_ms")) {m_stopCooldownMs = rf.find("sound_stop_cooldown_ms").asInt32();}

    if(!m_audiorecorderRPCPort.open(localRPCPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to open RPC port to microphone");
        return false;
    }

    if(!yarp::os::Network::connect(localRPCPortName, remoteRPCPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to connect RPC port %s -> %s", localRPCPortName.c_str(), remoteRPCPortName.c_str());
        return false;
    }

    yCInfo(SPEECH_MIC_CONTROLLER, "Configured SoundCatcher RPC %s -> %s", localRPCPortName.c_str(), remoteRPCPortName.c_str());
    yCInfo(SPEECH_MIC_CONTROLLER, "SoundCatcher stop cooldown=%d ms", m_stopCooldownMs);

     return true;
}

void SoundCatcher::onRead(yarp::sig::Sound& soundReceived) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (soundReceived.getSamples() == 0)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastStopCommandTime).count();
    if (elapsedMs < m_stopCooldownMs)
    {
        return;
    }

    yarp::os::Bottle req_stop{"stopRecording_RPC"};
    if (!m_audiorecorderRPCPort.write(req_stop))
    {
        yCError(SPEECH_MIC_CONTROLLER, "SoundCatcher::onRead. Failed to send stopRecording_RPC.");
        return;
    }
    m_lastStopCommandTime = now;
    yCInfo(SPEECH_MIC_CONTROLLER, "Sound detected on speaker path, requested microphone stop");

}

SoundCatcher::~SoundCatcher()
{
    m_audiorecorderRPCPort.close();
}


/* ------------------------------------------ BufferCatcher class ----------------------------------------------- */
bool BufferCatcher::configure(yarp::os::ResourceFinder& rf)
{
    std::string localRPCPortName = "/bufferCatcher/microphone:rpc";
    if(rf.check("rpc_microphone_buffer_local_port")) {localRPCPortName = rf.find("rpc_microphone_buffer_local_port").asString();}

    std::string remoteRPCPortName = "/audioRecorder_nws/rpc";
    if(rf.check("rpc_microphone_remote_port")) {remoteRPCPortName = rf.find("rpc_microphone_remote_port").asString();}
    else if(rf.check("rpc_microphone_port")) {remoteRPCPortName = rf.find("rpc_microphone_port").asString();}

    if(rf.check("resume_delay_ms")) {m_resumeDelayMs = rf.find("resume_delay_ms").asInt32();}
    if(rf.check("resume_when_buffer_leq")) {m_resumeWhenBufferLEQ = rf.find("resume_when_buffer_leq").asInt32();}
    if(rf.check("min_idle_status_before_resume")) {m_minIdleStatusesBeforeResume = rf.find("min_idle_status_before_resume").asInt32();}
    if(rf.check("force_resume_timeout_ms")) {m_forceResumeTimeoutMs = rf.find("force_resume_timeout_ms").asInt32();}

    if(!m_audiorecorderRPCPort.open(localRPCPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to open RPC port to microphone");
        return false;
    }

    if(!yarp::os::Network::connect(localRPCPortName, remoteRPCPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to connect RPC port %s -> %s", localRPCPortName.c_str(), remoteRPCPortName.c_str());
        return false;
    }

    yCInfo(SPEECH_MIC_CONTROLLER, "Configured BufferCatcher RPC %s -> %s", localRPCPortName.c_str(), remoteRPCPortName.c_str());
    yCInfo(SPEECH_MIC_CONTROLLER, "BufferCatcher resume_when_buffer_leq=%d min_idle_status_before_resume=%d resume_delay_ms=%d",
           m_resumeWhenBufferLEQ,
           m_minIdleStatusesBeforeResume,
           m_resumeDelayMs);
    yCInfo(SPEECH_MIC_CONTROLLER, "BufferCatcher force_resume_timeout_ms=%d", m_forceResumeTimeoutMs);

     return true;
}

bool BufferCatcher::restartMicrophoneLocked(const char* reason)
{
    yCInfo(SPEECH_MIC_CONTROLLER, "Restarting microphone (%s)", reason);
    yarp::os::Bottle req_start{"startRecording_RPC"};
    if (!m_audiorecorderRPCPort.write(req_start))
    {
        yCError(SPEECH_MIC_CONTROLLER, "BufferCatcher::restartMicrophoneLocked. Failed to send startRecording_RPC.");
        return false;
    }

    m_switchFlipped = false;
    m_idleStatusesCounter = 0;
    m_resumePending = false;
    return true;
}

void BufferCatcher::onRead(yarp::sig::AudioPlayerStatus& status) {
    std::lock_guard<std::mutex> lock(m_mutex);
    yCInfo(SPEECH_MIC_CONTROLLER, "Received audio player status: %s", status.toString().c_str());
    const bool isPlaying = status.current_buffer_size > m_resumeWhenBufferLEQ;

    if(isPlaying && !m_switchFlipped)
    {
        yCInfo(SPEECH_MIC_CONTROLLER, "Audio is playing, stopping microphone");
        yarp::os::Bottle req_stop{"stopRecording_RPC"};
        if (!m_audiorecorderRPCPort.write(req_stop))
        {
            yCError(SPEECH_MIC_CONTROLLER, "BufferCatcher::onRead. Failed to send stopRecording_RPC.");
            return;
        }
        m_switchFlipped = true;
        m_idleStatusesCounter = 0;
        m_resumePending = false;
        m_lastPlayingStatusTime = std::chrono::steady_clock::now();
    }
    else if(!isPlaying && m_switchFlipped)
    {
        ++m_idleStatusesCounter;
        if (m_idleStatusesCounter < m_minIdleStatusesBeforeResume)
        {
            return;
        }

        if (!m_resumePending)
        {
            yCInfo(SPEECH_MIC_CONTROLLER, "Audio finished by status, scheduling microphone restart in %d ms", m_resumeDelayMs);
            m_resumePending = true;
            m_resumeAtTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_resumeDelayMs);
        }
    }
    else if (isPlaying)
    {
        m_idleStatusesCounter = 0;
        m_resumePending = false;
        m_lastPlayingStatusTime = std::chrono::steady_clock::now();
    }
}

void BufferCatcher::checkAndForceResumeIfTimedOut()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_switchFlipped)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (m_resumePending && now >= m_resumeAtTime)
    {
        (void)restartMicrophoneLocked("playback finished by status");
        return;
    }

    if (m_forceResumeTimeoutMs <= 0)
    {
        return;
    }

    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastPlayingStatusTime).count();
    if (elapsedMs >= m_forceResumeTimeoutMs)
    {
        yCInfo(SPEECH_MIC_CONTROLLER,
               "No idle status received for %lld ms after mic stop. Forcing microphone restart.",
               static_cast<long long>(elapsedMs));
        (void)restartMicrophoneLocked("force resume timeout");
    }
}

BufferCatcher::~BufferCatcher()
{
    m_audiorecorderRPCPort.close();
}


/* ------------------------------------------ TranscriptionCatcher class ----------------------------------------------- */
bool TranscriptionCatecher::configure(yarp::os::ResourceFinder& rf)
{
    std::string localRPCPortName = "/transcriptionCatcher/microphone:rpc";
    if(rf.check("rpc_microphone_transcription_local_port")) {localRPCPortName = rf.find("rpc_microphone_transcription_local_port").asString();}

    std::string remoteRPCPortName = "/audioRecorder_nws/rpc";
    if(rf.check("rpc_microphone_remote_port")) {remoteRPCPortName = rf.find("rpc_microphone_remote_port").asString();}
    else if(rf.check("rpc_microphone_port")) {remoteRPCPortName = rf.find("rpc_microphone_port").asString();}

    if(!m_audiorecorderRPCPort.open(localRPCPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to open RPC port to microphone");
        return false;
    }

    if(!yarp::os::Network::connect(localRPCPortName, remoteRPCPortName))
    {
        yCError(SPEECH_MIC_CONTROLLER, "Unable to connect RPC port %s -> %s", localRPCPortName.c_str(), remoteRPCPortName.c_str());
        return false;
    }

    yCInfo(SPEECH_MIC_CONTROLLER, "Configured TranscriptionCatcher RPC %s -> %s", localRPCPortName.c_str(), remoteRPCPortName.c_str());

     return true;
}

void TranscriptionCatecher::onRead(yarp::os::Bottle& transcription) {
    std::lock_guard<std::mutex> lock(m_mutex);
    yCInfo(SPEECH_MIC_CONTROLLER, "Received transcription: %s", transcription.toString().c_str());
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
    m_bufferCatcher.checkAndForceResumeIfTimedOut();
    return true;
}