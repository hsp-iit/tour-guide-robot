// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef VAD_MODULE_H
#define VAD_MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/sig/AudioRecorderStatus.h>
#include "Detector.h"

#include "SileroVADServer.h"


class VoiceActivationDetectionModule : public yarp::os::RFModule
{
private:
    static constexpr int VAD_FREQUENCY_DEFAULT = 16000;
    static constexpr float VAD_THRESHOLD = 0.9;
    static constexpr bool VAD_SAVE_GAP = true;
    static constexpr int VAD_GAP_ALLOWANCE_DEFAULT = 18; // In packets of 32 ms
    static constexpr int VAD_SAVE_PRIOR_TO_DETECTION = 15; // In packets of 32 ms
    static constexpr float VAD_SPEECH_PROB_EMA_ALPHA = 0.35f;
    static constexpr float VAD_STOP_THRESHOLD_MARGIN = 0.15f;
    const std::string MODEL_PATH = "/usr/local/src/robot/silero-vad/src/silero_vad/data/silero_vad.onnx";

    std::unique_ptr<SileroVADServer> m_rpcServer;
    yarp::os::RpcServer m_rpcPort;

    int m_vadFrequency{VAD_FREQUENCY_DEFAULT};
    float m_vadThreshold{VAD_THRESHOLD};
    int m_vadGapAllowance{VAD_GAP_ALLOWANCE_DEFAULT};
    bool m_vadSaveGap{VAD_SAVE_GAP};
    int m_vadSavePriorToDetection{VAD_SAVE_PRIOR_TO_DETECTION};
    float m_vadSpeechProbEmaAlpha{VAD_SPEECH_PROB_EMA_ALPHA};
    float m_vadStopThresholdMargin{VAD_STOP_THRESHOLD_MARGIN};
    std::string m_modelPath = MODEL_PATH;
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioPort;            /** The input port for receiving the microphone input. **/
    std::shared_ptr<Detector> m_audioProcessor;

public:
    bool configure(yarp::os::ResourceFinder &rf) override;
    bool close() override;
    bool updateModule() override;
};

#endif
