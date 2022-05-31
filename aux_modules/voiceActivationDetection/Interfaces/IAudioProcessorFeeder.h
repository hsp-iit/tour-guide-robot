// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BEHAVIOR_TOUR_ROBOT_IAUDIOPROCESSORFEEDER_H
#define BEHAVIOR_TOUR_ROBOT_IAUDIOPROCESSORFEEDER_H

class IAudioProcessorFeeder{
public:

    virtual void addSound(yarp::sig::Sound&& sound) = 0;

};

#endif //BEHAVIOR_TOUR_ROBOT_IAUDIOPROCESSORFEEDER_H
