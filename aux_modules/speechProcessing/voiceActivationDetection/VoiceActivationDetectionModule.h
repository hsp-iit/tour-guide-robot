// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSORCREATOR_H
#define BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSORCREATOR_H

#include <yarp/os/RFModule.h>
#include <yarp/dev/AudioRecorderStatus.h>
#include "Detector.h"

// Other frequencies do not seem to work well

class VoiceActivationDetectionModule : public yarp::os::RFModule
{
private:
    static constexpr int VAD_FREQUENCY_DEFAULT = 16000;
    static constexpr int VAD_SAMPLE_LENGTH_DEFAULT = 20; // millisecond
    static constexpr int VAD_AGGRESSIVENESS_DEFAULT = 1;
    static constexpr int VAD_GAP_ALLOWANCE_DEFAULT = 29;
    static constexpr int VAD_MIN_SOUND_OUT_SIZE_DEFAULT = 60;

    int m_vadFrequency{VAD_FREQUENCY_DEFAULT};
    int m_vadSampleLength{VAD_SAMPLE_LENGTH_DEFAULT};
    int m_vadAggressiveness{VAD_AGGRESSIVENESS_DEFAULT};
    int m_vadGapAllowance{VAD_GAP_ALLOWANCE_DEFAULT};
    int m_minSoundOutSize{VAD_MIN_SOUND_OUT_SIZE_DEFAULT};
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioPort;            /** The input port for receiving the microphone input. **/
    double m_period{0.032};                                 /** The module period. **/
    std::shared_ptr<Detector> m_audioProcessor;
    std::mutex m_mutex; /** Internal mutex. **/

public:
    bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool close() override;
    bool updateModule() override;
};

#endif // BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSORCREATOR_H
