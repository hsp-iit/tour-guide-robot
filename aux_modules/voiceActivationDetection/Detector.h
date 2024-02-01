// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause


#ifndef BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSOR_H
#define BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSOR_H

#include <yarp/sig/Sound.h>
#include <yarp/os/TypedReaderCallback.h>
#include <yarp/os/LogStream.h>

#include <functional>
#include <cmath>
#include <deque>
#include <memory>

#include <mutex>
#include <fvad.h>
#include <yarp/os/BufferedPort.h>

class Detector: public yarp::os::TypedReaderCallback<yarp::sig::Sound> {
public:
    Detector(int vadFrequency,
                   int vadSampleLength,
                   int vadAggressiveness,
                   int bufferSize,
                   std::string filteredAudioPortOutName);
    using TypedReaderCallback<yarp::sig::Sound>::onRead;
    void onRead(yarp::sig::Sound& soundReceived) override;
    bool m_runInference = false;


private:
    int m_vadFrequency;
    int m_vadSampleLength;
    int m_vadAggressiveness;
    Fvad * m_fvadObject {nullptr}; /** The voice activity detection object. **/
    std::deque<std::shared_ptr<std::vector<int16_t>>> m_soundToSend; /** Internal sound buffer. **/
    std::vector<int16_t> m_soundBuffer;
    int m_fillCount; // keep tracks of up to what index the buffer is full
    std::mutex m_mutex; /** Internal mutex. **/
    int m_bufferSize;
    int m_paddingCurrentSize{0};
    bool m_soundDetected{false};
    std::string m_filteredAudioPortOutName;
    yarp::os::BufferedPort<yarp::sig::Sound> m_filteredAudioOutputPort; /** The output port for sending the filtered audio. **/
    bool m_microphoneOpen{false};
    std::deque<yarp::sig::Sound> m_soundToProcess;

    void processPacket(std::vector<int16_t> copiedSound);
    void sendSound();
    void clearVector();
    void closeMicrophone();
    std::shared_ptr<std::vector<int16_t>> createVector();
};

#endif //BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSOR_H
