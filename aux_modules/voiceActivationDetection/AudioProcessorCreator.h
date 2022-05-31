// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSORCREATOR_H
#define BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSORCREATOR_H

#include <yarp/os/RFModule.h>
#include <yarp/dev/AudioRecorderStatus.h>
#include "AudioProcessor.h"
#include "Interfaces/IAudioProcessorMicrophoneCloser.h"
#include "AudioCallback.h"
#include "MicrophoneStatusCallback.h"
#include "headSynchronizerRPC.h"

// Other frequencies do not seem to work well

class AudioProcessorCreator : public yarp::os::RFModule
{
private:
    static constexpr int VAD_FREQUENCY_DEFAULT = 16000;
    static constexpr int VAD_SAMPLE_LENGTH_DEFAULT = 20; // millisecond
    static constexpr int VAD_AGGRESSIVENESS_DEFAULT = 3;
    static constexpr int PERIOD_DEFAULT = 1;

    int m_vadFrequency{VAD_FREQUENCY_DEFAULT};
    int m_vadSampleLength{VAD_SAMPLE_LENGTH_DEFAULT};
    int m_vadAggressiveness{VAD_AGGRESSIVENESS_DEFAULT};
    yarp::os::BufferedPort<yarp::os::Bottle> m_microphoneStatusPort; /** The input port for receiving the microphone status. **/
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioPort;            /** The input port for receiving the microphone input. **/
    double m_period{PERIOD_DEFAULT};                                 /** The module period. **/
    int m_bufferSize{8};
    std::unique_ptr<AudioCallback> m_audioCallback;
    std::shared_ptr<AudioProcessor> m_audioProcessor;
    std::mutex m_mutex; /** Internal mutex. **/
    std::shared_ptr<MicrophoneStatusCallback> m_microphoneStatusCallback;

    std::string m_headSynchronizerClientName;
    yarp::os::Port m_pHeadSynchronizerClient;
    headSynchronizerRPC m_headSynchronizer;

public:
    bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool close() override;
    bool updateModule() override;
};

#endif // BEHAVIOR_TOUR_ROBOT_AUDIOPROCESSORCREATOR_H
