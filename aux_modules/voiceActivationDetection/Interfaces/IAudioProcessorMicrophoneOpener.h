// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BEHAVIOR_TOUR_ROBOT_IAUDIOPROCESSORMICROPHONEOPENER_H
#define BEHAVIOR_TOUR_ROBOT_IAUDIOPROCESSORMICROPHONEOPENER_H


class IAudioProcessorMicrophoneOpener{
public:

    /**
     *  This function is used by the AudioProcessor and is called by the MicrophoneStatusCallback
     *  to reopen the microphone
     */
    virtual void openMicrophone() = 0;
};

#endif //BEHAVIOR_TOUR_ROBOT_IAUDIOPROCESSORMICROPHONEOPENER_H
