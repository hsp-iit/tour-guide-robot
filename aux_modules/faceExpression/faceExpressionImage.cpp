#include <cmath>
#include <string>
#include "faceExpressionImage.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>
#include <yarp/math/Rand.h>


#define BLINK_STEP_NUM  10

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::math;

FaceExpressionImageModule::FaceExpressionImageModule()
{
    m_image.create(FACE_HEIGHT, FACE_WIDTH, CV_8UC3);
}

bool FaceExpressionImageModule::configure(ResourceFinder &rf)
{
    // Open ports
    if(!m_rpcPort.open("/" + getName() + "/rpc") )
    {
        yError() << "Cannot open RPC port";
        return false;
    }

    if(!attach(m_rpcPort))
    {
        yError() << "RFModule cannot read from RPC port (" << m_rpcPort.getName() << ")";
    }

    m_thread_output = new DrawingThread(rf, getName(), 0.033, m_image, m_mutex);
    m_thread_eyes   = new EyesThread(rf, getName(), 0.020, m_image, m_mutex);
    m_thread_ears   = new EarsThread(rf, getName(), 0.020, m_image, m_mutex);
    m_thread_mouth  = new MouthThread(rf, getName(), 0.020, m_image, m_mutex);

    if (m_thread_output)
        if (!m_thread_output->start())
            return false;
    if (m_thread_eyes)
        if (!m_thread_eyes->start())
            return false;
    if (m_thread_ears)
        if (!m_thread_ears->start())
            return false;
    if (m_thread_mouth)
        if (!m_thread_mouth->start())
            return false;

    yInfo() << "FaceExpressionImage module started";
    return true;
}

bool FaceExpressionImageModule::respondString(const Bottle& command, Bottle& reply)
{
    string cmd = command.get(0).asString();
    yDebug() << "Received command " << command.toString();
    reply.clear();

    if (cmd == "help")
    {
        reply.addVocab32(Vocab32::encode("many"));
        reply.addString("start_ears 0/1");
        reply.addString("start_mouth 0/1");
        reply.addString("start_blinking 0/1");
        reply.addString("enable_draw_eyes 0/1");
        reply.addString("enable_draw_ears 0/1");
        reply.addString("enable_draw_mouth 0/1");
        reply.addString("color_mouth 255 255 255");
        reply.addString("color_ears  255 255 255");
        reply.addString("reset_to_default");
        reply.addString("emotion 0/1/2");
        reply.addString("black_screen");
        return true;
    }
    else if (cmd == "start_ears")
    {
        bool value = command.get(1).asBool();
        start_listening(value);
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "start_mouth")
    {
        bool value = command.get(1).asBool();
        start_talking(value);
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "start_blinking")
    {
        bool value = command.get(1).asBool();
        start_blinking(value);
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "enable_draw_ears")
    {
        bool value = command.get(1).asBool();
        if (m_thread_ears)
        {
            m_thread_ears->enableDrawing(value);
        }
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "enable_draw_mouth")
    {
        bool value = command.get(1).asBool();
        if (m_thread_mouth)
        {
            m_thread_mouth->enableDrawing(value);
        }
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "enable_draw_eyes")
    {
        bool value = command.get(1).asBool();
        if (m_thread_eyes)
        {
            m_thread_eyes->enableDrawing(value);
        }
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "emotion")
    {
        int value = command.get(1).asFloat32();
        if (m_thread_mouth)
        {
            m_thread_mouth->setExpression(value);
        }
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "reset_to_default")
    {
        reset_default();
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "black_screen")
    {
        black();
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "color_ears")
    {
        float vr= command.get(1).asFloat32();
        float vg= command.get(2).asFloat32();
        float vb= command.get(3).asFloat32();
        if (m_thread_ears) m_thread_ears->setColor(vr,vg,vb);
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    else if (cmd == "color_mouth")
    {
        float vr = command.get(1).asFloat32();
        float vg = command.get(2).asFloat32();
        float vb = command.get(3).asFloat32();
        if (m_thread_mouth) m_thread_mouth->setColor(vr, vg, vb);
        reply.addVocab32(yarp::os::Vocab32::encode("ok"));
        return true;
    }
    return false;
}

bool FaceExpressionImageModule::start_blinking(bool val)
{
    if (m_thread_eyes)
    {
        m_thread_eyes->activateBlink(val);
        if (val == false)
        {
            m_thread_eyes->resetToDefault(false);
        }
    }
    return true;
}

bool FaceExpressionImageModule::start_talking(bool val)
{
    if (m_thread_mouth)
    {
        m_thread_mouth->activateTalk(val);
        if (val == false)
        {
            m_thread_mouth->resetToDefault();
        }
    }
    return true;
}

bool FaceExpressionImageModule::start_listening(bool val)
{
    if (m_thread_ears)
    {
        m_thread_ears->activateBars(val);
        if (val == false)
        {
            m_thread_ears->resetToDefault();
        }
    }
    return true;
}

bool FaceExpressionImageModule::reset_default()
{
    if (m_thread_eyes)  m_thread_eyes->enableDrawing(true);
    if (m_thread_mouth) m_thread_mouth->enableDrawing(true);
    if (m_thread_ears)  m_thread_ears->enableDrawing(true);
    if (m_thread_eyes)  m_thread_eyes->activateBlink(true); //or false
    if (m_thread_mouth) m_thread_mouth->activateTalk(false);
    if (m_thread_ears)  m_thread_ears->activateBars(false);
    if (m_thread_eyes)  m_thread_eyes->resetToDefault();
    if (m_thread_mouth) m_thread_mouth->resetToDefault();
    if (m_thread_ears)  m_thread_ears->resetToDefault();
    return true;
}

bool FaceExpressionImageModule::black()
{
    if (m_thread_eyes)  m_thread_eyes->enableDrawing(false);
    if (m_thread_mouth) m_thread_mouth->enableDrawing(false);
    if (m_thread_ears)  m_thread_ears->enableDrawing(false);
    return true;
}

bool FaceExpressionImageModule::respondVocab(const Bottle& command, Bottle& reply)
{
    int cmd = command.get(0).asVocab32();

    yTrace() << "Received command " << command.toString();
    switch (cmd)
    {
    case VOCAB_AUDIO_START:
    {
        start_listening(true);
    }
    break;

    case VOCAB_AUDIO_STOP:
    {
        start_listening(false);
    }
    break;

    case VOCAB_TALK_START:
    {
        start_talking(true);
    }
    break;

    case VOCAB_TALK_STOP:
    {
        start_talking(false);
    }
    break;

    case VOCAB_BLINK:
    {
        start_blinking(true);
    }
    break;

    case VOCAB_RESET:
    {
        reset_default();
    }
    break;

    case VOCAB_BLACK_RESET:
    {
        black();
    }
    break;

    default:
    {
        yError() << "Unknown command " << command.toString();
    }
    break;
    }
    reply.clear();
    reply.addVocab32(yarp::os::Vocab32::encode("ok"));
    return true;
}

bool FaceExpressionImageModule::respond(const Bottle& command, Bottle& reply)
{
    if (command.get(0).isString())
    {
        return respondString(command,reply);
    }
    else if (command.get(0).isVocab32())
    {
        return respondVocab(command, reply);
    }
    yDebug() << "Error";
    reply.clear();
    reply.addVocab32(yarp::os::Vocab32::encode("ok"));
    return true;
}

bool FaceExpressionImageModule::interruptModule()
{
    return true;
}

bool FaceExpressionImageModule::close()
{
    m_rpcPort.close();
    return true;
}

double FaceExpressionImageModule::getPeriod()
{
    return 0.1;
}

bool FaceExpressionImageModule::updateModule()
{
    return true;
}
