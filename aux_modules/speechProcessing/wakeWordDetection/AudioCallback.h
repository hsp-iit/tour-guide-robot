#ifndef AUDIOCALLBACK_H
#define AUDIOCALLBACK_H

#include <yarp/os/TypedReaderCallback.h>
#include <yarp/sig/Sound.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>

#include <memory>
#include <deque>
#include <string>

#include "VADMsgs.h"
#include "pv_porcupine.h"

class AudioCallback: public yarp::os::TypedReaderCallback<yarp::sig::Sound> {
public:
    AudioCallback(std::string &rpcPortName);
    using TypedReaderCallback<yarp::sig::Sound>::onRead;
    void onRead(yarp::sig::Sound& soundReceived) override;
    bool m_currentlyStreaming = false; // stream until VAD reports that its done detecting voice


private:
    yarp::os::RpcClient m_rpcClientPort;
    VADMsgs m_rpcClient;
    
    pv_porcupine_t *m_porcupine = NULL;

    int m_frameSize;
    std::vector<int16_t> m_curreAudioFrame;
    int m_sampleCounter = 0;

    std::deque<std::vector<int16_t>> back;

    std::string m_audioOutName = "/wake/audio:o";
    yarp::os::BufferedPort<yarp::sig::Sound> m_audioOut; // pass audio to VAD from here to avoid out of sync errors

    void processFrame(yarp::sig::Sound &soundReceived);
};

#endif
