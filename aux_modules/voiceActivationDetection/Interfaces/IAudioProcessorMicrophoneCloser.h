// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BEHAVIOR_TOUR_ROBOT_IAUDIOPROCESSORMICROPHONECLOSER_H
#define BEHAVIOR_TOUR_ROBOT_IAUDIOPROCESSORMICROPHONECLOSER_H


class IAudioProcessorMicrophoneCloser{
public:

    /**
     *  This function is used by the AudioProcessorCreator and is called by the AudioProcessor
     *  to synchronize the closing of the microphone
     */
    virtual void closeMicrophone() = 0;
};

#endif //BEHAVIOR_TOUR_ROBOT_IAUDIOPROCESSORMICROPHONECLOSER_H
