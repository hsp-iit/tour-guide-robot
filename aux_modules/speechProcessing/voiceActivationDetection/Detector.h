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
#include <fvad.h>
#include <yarp/os/BufferedPort.h>
#include "WakeMsgs.h"

class Detector: public yarp::os::TypedReaderCallback<yarp::sig::Sound> {
public:
    Detector(int vadFrequency,
                   int vadSampleLength,
                   int vadAggressiveness,
                   int gapAllowance,
                   int minSoundSize,
                   std::string filteredAudioPortOutName,
                   std::string wakeWordClientPort);
    using TypedReaderCallback<yarp::sig::Sound>::onRead;
    void onRead(yarp::sig::Sound& soundReceived) override;

private:
    int m_vadFrequency;
    int m_vadSampleLength;
    int m_vadAggressiveness;
    Fvad * m_fvadObject {nullptr}; /** The voice activity detection object. **/
    std::deque<std::vector<int16_t>> m_soundToSend; /** Internal sound buffer. **/
    std::vector<int16_t> m_currentSoundBuffer;
    int m_fillCount; // keep track of up to what index the buffer is full
    bool m_soundDetected{false};
    std::string m_filteredAudioPortOutName;
    yarp::os::BufferedPort<yarp::sig::Sound> m_filteredAudioOutputPort; /** The output port for sending the filtered audio. **/
    std::deque<yarp::sig::Sound> m_soundToProcess;
    int m_gapAllowance = 34;
    int m_gapCounter = 0;
    int m_minSoundSize = 0; // how many extra packets to pad, can help with transcription

    yarp::os::RpcClient m_rpcClientPort;
    WakeMsgs m_rpcClient;

    void processPacket();
    void sendSound();
};

#endif
