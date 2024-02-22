#ifndef AUDIOCALLBACK_H
#define AUDIOCALLBACK_H

#include <yarp/os/TypedReaderCallback.h>
#include <yarp/sig/Sound.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>

#include <memory>
#include <deque>
#include <string>

#include "pv_porcupine.h"

const int FREQUENCY = 16000; // Porcupine model expects a frequency of 16000

class AudioCallback: public yarp::os::TypedReaderCallback<yarp::sig::Sound> {
public:
    AudioCallback(const std::string audioOutName,
                    const std::string accessKey,
                    const std::string modelPath,
                    const std::string keywordPath,
                    const float sensitivity);

    using TypedReaderCallback<yarp::sig::Sound>::onRead;
    void onRead(yarp::sig::Sound& soundReceived) override;
    bool m_currentlyStreaming = false; // stream until VAD reports that its done detecting voice


private:    
    pv_porcupine_t *m_porcupine = NULL;

    int m_frameSize;
    std::vector<int16_t> m_currentAudioSliceBuffer;
    int m_sampleCounter = 0;

    std::deque<std::vector<int16_t>> m_previousAudioBuffer; // store previous audio samples to compensate for detector latency

    bool m_sendRemainingSamples = false;
    std::vector<int16_t> m_remainingSamplesBuffer; // once wake word is detected gather all the following samples to send

    yarp::os::BufferedPort<yarp::sig::Sound> m_audioOut;

    void processFrame(yarp::sig::Sound &soundReceived);
    bool processSliceOfFrame(const size_t &num_samples, int currentSampleIdx, int &m_remainingSamplesBufferIdx);
    void sendRemainingSamples();
    void printPorcupineErrorMessage(char **message_stack, int32_t message_stack_depth);
};

#endif
