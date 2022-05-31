#ifndef FACE_EXPRESSION_IMAGE_HPP
#define FACE_EXPRESSION_IMAGE_HPP
#define VOCAB_AUDIO_START       yarp::os::createVocab32('a','s','t','a')
#define VOCAB_AUDIO_STOP        yarp::os::createVocab32('a','s','t','o')
#define VOCAB_TALK_START        yarp::os::createVocab32('t','s','t','a')
#define VOCAB_TALK_STOP         yarp::os::createVocab32('t','s','t','o')
#define VOCAB_BLINK             yarp::os::createVocab32('b','l','i','n')
#define VOCAB_RESET             yarp::os::createVocab32('r','s','t')
#define VOCAB_BLACK_RESET       yarp::os::createVocab32('b','l','c','k')


#include <mutex>
#include <string>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>
#include "drawingThread.hpp"
#include "earsThread.hpp"
#include "mouthThread.hpp"
#include "eyesThread.hpp"



class FaceExpressionImageModule: public yarp::os::RFModule
{
private:
    DrawingThread*           m_thread_output = nullptr;
    MouthThread*             m_thread_mouth = nullptr;
    EyesThread*              m_thread_eyes = nullptr;
    EarsThread*              m_thread_ears = nullptr;

    std::mutex                                                 m_mutex;
    yarp::os::Port                                             m_rpcPort;
    cv::Mat                                                    m_image;

public:
    FaceExpressionImageModule();
    bool configure(yarp::os::ResourceFinder &rf) override;
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) override;
    bool close() override;
    bool interruptModule() override;
    double getPeriod() override;
    bool   updateModule() override;

private:
    bool respondString(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool respondVocab(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

private:
    bool start_blinking(bool val);
    bool start_talking(bool val);
    bool start_listening(bool val);
    bool reset_default();
    bool black();

};

#endif
