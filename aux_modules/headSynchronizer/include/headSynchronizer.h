#ifndef BEHAVIOR_TOUR_ROBOT_HEAD_SYNCHRONIZER_H
#define BEHAVIOR_TOUR_ROBOT_HEAD_SYNCHRONIZER_H

#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include "yarp/dev/GenericVocabs.h"
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <headSynchronizerRPC.h>
#include <iostream>
#include <vector>
#include <mutex>
#include <yarp/os/LogComponent.h>
#include <yarp/sig/AudioRecorderStatus.h>
#include <yarp/sig/AudioPlayerStatus.h>

class StatusCallback;
class HeadSynchronizer : public yarp::os::RFModule, public headSynchronizerRPC
{
private:
    StatusCallback *m_statusCallback;
    std::string m_name;
    std::mutex m_mutex;
    double m_period;
    bool m_isSpeaking;
    bool m_isError;
    std::vector<std::string> m_textBuffer;

    std::string m_statusInputName;
    std::string m_synthesisOutputName;
    std::string m_eyeContactName;
    std::string m_rpcName;
    std::string m_faceOutputName;
    std::string m_microphoneStatusName;
    std::string m_microphoneOutputName;
    std::string m_playerStatusName;
    std::string m_playerOutputName;
    std::string m_headSynchronizerThriftPortName;

    yarp::os::BufferedPort<yarp::os::Bottle> m_pStatusInput;
    yarp::os::BufferedPort<yarp::sig::AudioRecorderStatus> m_pMicrophoneStatus;
    yarp::os::BufferedPort<yarp::sig::AudioPlayerStatus> m_pPlayerStatus;

    yarp::os::Port m_headSynchronizerThriftPort;
    yarp::os::Port m_pSynthesisOutput;
    yarp::os::Port m_pEyeContact;
    yarp::os::Port m_pMicrophoneOutput;
    yarp::os::Port m_pPlayerOutput;
    yarp::os::Port m_pFaceOutput;
    yarp::os::RpcServer m_pRPC;

    bool writeToPort(const std::string &s, yarp::os::Port &port);
    bool writeToPort(const std::string &s, yarp::os::Port &port, yarp::os::Bottle &res);
    bool isAudioPlaying();

public:
    HeadSynchronizer(const std::string &name);

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool interruptModule();
    virtual bool updateModule();

    virtual bool say(const std::string &s);
    virtual bool pauseSpeaking();
    virtual bool continueSpeaking();
    virtual bool reset();
    virtual bool isSpeaking();
    virtual bool isHearing();
    virtual bool startHearing();
    virtual bool stopHearing();
    virtual bool sadFaceWarning();
    virtual bool sadFaceError();
    virtual bool busyFaceError();
    virtual bool happyFace();
    virtual bool busyFace();

    bool changeEmotion(int i);
    bool colorEars(int r, int g, int b);
    bool colorMouth(int r, int g, int b);
    bool getIsError();
};

class StatusCallback : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    StatusCallback(HeadSynchronizer *headSynchronizer);
    using yarp::os::TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle &b) override;

private:
    HeadSynchronizer *m_headSynchronizer;
};

#endif // BEHAVIOR_TOUR_ROBOT_HEAD_SYNCHRONIZER_H
