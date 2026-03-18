/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once

#include <mutex>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Sound.h>
#include <yarp/os/TypedReaderCallback.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/AudioPlayerStatus.h>

class SoundCatcher : public yarp::os::TypedReaderCallback<yarp::sig::Sound>
{
public:
    SoundCatcher() = default;
    ~SoundCatcher() override;
    bool configure(yarp::os::ResourceFinder& rf);
    void onRead(yarp::sig::Sound& sound) override;
private:
    std::mutex m_mutex;
    yarp::os::RpcClient m_audiorecorderRPCPort;
};

class BufferCatcher : public yarp::os::TypedReaderCallback<yarp::sig::AudioPlayerStatus>
{
public:
    BufferCatcher() = default;
    ~BufferCatcher() override;;
    bool configure(yarp::os::ResourceFinder& rf);
    void onRead(yarp::sig::AudioPlayerStatus& status) override;
private:
    std::mutex          m_mutex;
    yarp::os::RpcClient m_audiorecorderRPCPort;
    bool                m_switchFlipped{false};
};

class TranscriptionCatecher : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    TranscriptionCatecher() = default;
    ~TranscriptionCatecher() override;
    bool configure(yarp::os::ResourceFinder& rf);
    void onRead(yarp::os::Bottle& transcription) override;
private:
    std::mutex          m_mutex;
    yarp::os::RpcClient m_audiorecorderRPCPort;
};


class SpeechMicController : public yarp::os::RFModule
{
public:
    SpeechMicController() = default;
    ~SpeechMicController() override = default;
    bool configure(yarp::os::ResourceFinder& rf) override;
    bool close() override;
    double getPeriod() override;
    bool updateModule() override;
private:
    SoundCatcher m_soundCatcher;
    BufferCatcher m_bufferCatcher;
    TranscriptionCatecher m_transcriptionCatcher;
    yarp::os::BufferedPort<yarp::sig::Sound> m_soundPort;
    yarp::os::BufferedPort<yarp::sig::AudioPlayerStatus> m_audioPlayPort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_transcriptionPort;
};
