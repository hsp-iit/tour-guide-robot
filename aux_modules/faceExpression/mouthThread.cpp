#include <cmath>
#include <string>
#include "mouthThread.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>
#include <yarp/math/Rand.h>

#define BLINK_STEP_NUM 10
#define FACE_WIDTH 80
#define FACE_HEIGHT 32

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::math;

MouthThread::MouthThread(ResourceFinder &_rf, string _moduleName, double _period, cv::Mat &_image, std::mutex &_mutex) : PeriodicThread(_period),
                                                                                                                         m_face(_image),
                                                                                                                         m_drawing_mutex(_mutex),
                                                                                                                         m_rf(_rf),
                                                                                                                         m_moduleName(_moduleName)
{
}

void MouthThread::afterStart(bool s) {}

void MouthThread::activateTalk(bool activate)
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);
    m_doTalk = activate;
}

bool MouthThread::threadInit()
{
    if (m_audioPlayPort.open("/" + m_moduleName + "/mouthAudioData:i") == false)
    {
        yError() << "Cannot open port";
        return false;
    }

    m_blackMouth.create(m_mouth_h, m_mouth_w, CV_8UC3);
    m_blackMouth.setTo(Scalar(0, 0, 0));

    m_defaultPlainMouth.create(m_mouth_h, m_mouth_w, CV_8UC3);
    m_defaultPlainMouth.setTo(Scalar(0, 0, 0));

    resetToDefault();

    return true;
}

void MouthThread::run()
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);

    // get the status
    yarp::sig::AudioPlayerStatus *Pstatus = m_audioPlayPort.read(false);
    if (Pstatus)
    {
        m_audioIsPlaying = Pstatus->current_buffer_size > 0;
    }

    if (m_audioIsPlaying == false)
    {
        showExpression();
        return;
    }

    // draw the mouth
    if (m_doTalk || m_audioIsPlaying)
    {
        updateTalk();
    }
}

void MouthThread::updateTalk()
{
    lock_guard<mutex> faceguard(m_drawing_mutex);
    clearWithBlack();
    if (m_drawEnable == false)
    {
        return;
    }

    // draw the mouth
    int pixels = FACE_HEIGHT >> 1;
    int y = FACE_HEIGHT - 2;
    for (int x = (FACE_WIDTH - pixels) >> 1; x < (FACE_WIDTH + pixels) >> 1; x++)
    {
        int y_ = y + int(round(Rand::scalar(-1, 1)));
        m_face.at<cv::Vec3b>(y_, x)[0] = m_mouthCurrentColor[0];
        m_face.at<cv::Vec3b>(y_, x)[1] = m_mouthCurrentColor[1];
        m_face.at<cv::Vec3b>(y_, x)[2] = m_mouthCurrentColor[2];
    }
}

void MouthThread::showExpression()
{
    lock_guard<mutex> faceguard(m_drawing_mutex);
    clearWithBlack();
    if (m_drawEnable == false)
    {
        return;
    }

    // draw the mouth
    int pixels = FACE_HEIGHT >> 1;
    double y = FACE_HEIGHT - 2;
    for (int x = (FACE_WIDTH - pixels) >> 1; x < (FACE_WIDTH + pixels) >> 1; x++)
    {
        int y_;
        if (emotion == 0)
        {
            // sad
            y_ = (int)(y + sin(0.32 * (x - (FACE_WIDTH - pixels))) * 2);
        }
        else if (emotion == 1)
        {
            // happy
            y_ = (int)(y + sin(0.32 * (x - (FACE_WIDTH - pixels)) - 135.1) * 2);
        }
        else if (emotion == 2)
        {
            // thinking
            y_ = y;
        }

        m_face.at<cv::Vec3b>(y_, x)[0] = m_mouthCurrentColor[0];
        m_face.at<cv::Vec3b>(y_, x)[1] = m_mouthCurrentColor[1];
        m_face.at<cv::Vec3b>(y_, x)[2] = m_mouthCurrentColor[2];
    }
}

void MouthThread::threadRelease()
{
    m_audioPlayPort.close();
}

void MouthThread::resetToDefault()
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);
    m_mouthCurrentColor = m_mouthDefaultColor;
    m_defaultPlainMouth.setTo(Scalar(0, 0, 0));
    m_defaultPlainMouth(Rect(0, 0, m_mouth_w, m_mouth_h)).copyTo(m_face(cv::Rect(FACE_WIDTH / 2 - m_mouth_w / 2, FACE_HEIGHT - m_mouth_h, m_mouth_w, m_mouth_h)));
}

void MouthThread::setColor(float vr, float vg, float vb)
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);
    m_mouthCurrentColor = Scalar(vr, vg, vb);
    m_defaultPlainMouth.setTo(m_mouthCurrentColor);
}

void MouthThread::clearWithBlack()
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);
    // lock_guard<mutex> faceguard(m_drawing_mutex);
    m_blackMouth(Rect(0, 0, m_mouth_w, m_mouth_h)).copyTo(m_face(cv::Rect(FACE_WIDTH / 2 - m_mouth_w / 2, FACE_HEIGHT - m_mouth_h, m_mouth_w, m_mouth_h)));
}

void MouthThread::enableDrawing(bool val)
{
    if (val)
    {
        m_drawEnable = true;
    }
    else
    {
        m_drawEnable = false;
        clearWithBlack();
    }
}

void MouthThread::setExpression(int e)
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);

    emotion = e;
}
