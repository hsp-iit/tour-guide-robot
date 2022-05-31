// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BEHAVIOR_TOUR_ROBOT_AUDIOCALLBACK_H
#define BEHAVIOR_TOUR_ROBOT_AUDIOCALLBACK_H


#include <yarp/os/TypedReaderCallback.h>
#include <yarp/sig/Sound.h>
#include <mutex>
#include <Interfaces/IAudioProcessorFeeder.h>
#include <memory>

class AudioCallback: public yarp::os::TypedReaderCallback<yarp::sig::Sound> {
public:
    AudioCallback(std::shared_ptr<IAudioProcessorFeeder> iAudioProcessorFeeder);
    using TypedReaderCallback<yarp::sig::Sound>::onRead;
    void onRead(yarp::sig::Sound& soundReceived) override;


private:
    std::mutex m_mutex; /** Internal mutex. **/
    std::shared_ptr<IAudioProcessorFeeder> m_iAudioProcessorFeeder;
};

#endif //BEHAVIOR_TOUR_ROBOT_AUDIOCALLBACK_H
