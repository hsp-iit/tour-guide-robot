#include <cmath>
#include <string>
#include "earsThread.hpp"
#include "utils.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>
#include <yarp/math/Rand.h>

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::math;

EarsThread::EarsThread(ResourceFinder& _rf, string _moduleName, double _period, cv::Mat& _image, std::mutex& _mutex) :
    PeriodicThread(_period),
    m_face(_image),
    m_drawing_mutex(_mutex),
    m_rf(_rf),
    m_moduleName(_moduleName)
{
}

void EarsThread::afterStart(bool s) { }

void EarsThread::activateBars(bool activate)
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);
    m_doBars = activate;
}

bool EarsThread::threadInit()
{
    // Check for optional params
    earBarL0_x = m_rf.check("earBar0_x", Value(1), "horizontal offset from left border of outer ear bar, in pixels from upper left corner of the image, starting from 0").asInt32();
    earBarR0_x = FACE_WIDTH - 1 - earBarL0_x;

    earBarL0_y = m_rf.check("earBar0_y", Value(6), "vertical offset from bottom border of outer ear bar, in pixels from upper left corner of the image, starting from 0").asInt32();
    earBarR0_y = earBarL0_y;

    earBar0_minLen = m_rf.check("earBar0_minLen", Value(3), "minimum length  of outer ear bar").asInt32();
    earBar0_maxLen = m_rf.check("earBar0_maxLen", Value(18), "maximum length  of outer ear bar").asInt32();
    earBar0_len = m_rf.check("earBar0_len", Value(10), "starting length of outer ear bar").asInt32();

    earBarL1_x = m_rf.check("earBar1_x", Value(3), "horizontal offset from left border of inner ear bar, in pixels from upper left corner of the image, starting from 0").asInt32();
    earBarR1_x = FACE_WIDTH - 1 - earBarL1_x;

    earBarL1_y = m_rf.check("earBar1_y", Value(6), "vertical offset from bottom border of inner ear bar, in pixels from upper left corner of the image, starting from 0").asInt32();
    earBarR1_y = earBarL1_y;

    earBar1_minLen = m_rf.check("earBar1_minLen", Value(4), "minimum length  of inner ear bar").asInt32();
    earBar1_maxLen = m_rf.check("earBar1_maxLen", Value(19), "maximum length  of inner ear bar").asInt32();
    earBar1_len = m_rf.check("earBar1_len", Value(11), "starting length of inner ear bar").asInt32();

    if (m_audioRecPort.open("/"+ m_moduleName+"/earsAudioData:i")==false)
    {
        yError() << "Cannot open port";
        return false;
    }

   if (m_audioStatusPort.open("/"+ m_moduleName+"/earsAudioStatus:i")==false)
    {
        yError() << "Cannot open port";
        return false;
    }

    // Create a Mat for maximum bar size
    m_earBar.create(earBar1_maxLen, barWidth, CV_8UC3);
    m_earBar.setTo(m_earsDefaultColor);

    m_blackBar.create(32, barWidth, CV_8UC3);
    m_blackBar.setTo(Scalar(0, 0, 0));

    this->resetToDefault();
    return true;
}

void EarsThread::run()
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);

    if (m_drawEnable == false)
    {
        clearWithBlack();
        return;
    }

    float percentage = 0.5;

    yarp::sig::AudioRecorderStatus *rec_status = m_audioStatusPort.read(false);
    if (rec_status)
    {
        m_micIsEnabled = rec_status->enabled; //&& rec_status->current_buffer_size > 0;
        //yInfo() << rec_status->current_buffer_size;
    //    yInfo() << m_micIsEnabled;
    }

    yarp::sig::Sound* data_audio = m_audioRecPort.read(false);
    if(m_doBars)
    {
        if(data_audio){
            auto vec= data_audio->getChannel(0);
      	    short int max_val = *std::max_element(vec.begin(),vec.end());
//            max_val = 30000;
            percentage = fabs((float)max_val / 32800);
            updateBars(percentage);
            yInfo() << percentage;
        }
    }
    else
    {
        updateBars(0.5);
    }

}

bool EarsThread::updateBars(float percentage)
{
    lock_guard<mutex> faceguard(m_drawing_mutex);

    earBar0_len = earBar0_minLen + (earBar0_maxLen - earBar0_minLen) *  percentage;
    earBar1_len = earBar1_minLen + (earBar1_maxLen - earBar1_minLen) *  percentage;

    if(m_micIsEnabled==false){
        m_earBar.setTo(Scalar(0,0,255));
        percentage=0.5;
    }
    else
    {
       m_earBar.setTo(m_earsDefaultColor);
    }
    if(percentage>0.85){
        m_earBar.setTo(Scalar(255,0,0));
    }


    // Reset bars to black
    m_blackBar(Rect(0, 0, barWidth, 32)).copyTo  (m_face(cv::Rect(earBarL0_x,  0, barWidth, 32)));
    m_blackBar(Rect(0, 0, barWidth, 32)).copyTo  (m_face(cv::Rect(earBarL1_x,  0, barWidth, 32)));
    m_blackBar(Rect(0, 0, barWidth, 32)).copyTo  (m_face(cv::Rect(earBarR0_x, 0, barWidth, 32)));
    m_blackBar(Rect(0, 0, barWidth, 32)).copyTo  (m_face(cv::Rect(earBarR1_x, 0, barWidth, 32)));

    // Left side
    m_earBar(Rect(0, 0, barWidth, earBar0_len)).copyTo  (m_face(cv::Rect(earBarL0_x, FACE_HEIGHT - earBarL0_y - earBar0_len, barWidth, earBar0_len)));
    m_earBar(Rect(0, 0, barWidth, earBar1_len)).copyTo  (m_face(cv::Rect(earBarL1_x, FACE_HEIGHT - earBarL1_y - earBar1_len, barWidth, earBar1_len)));

    // Right side
    m_earBar(Rect(0, 0, barWidth, earBar0_len)).copyTo  (m_face(cv::Rect(earBarR0_x, FACE_HEIGHT - earBarR0_y - earBar0_len, barWidth, earBar0_len)));
    m_earBar(Rect(0, 0, barWidth, earBar1_len)).copyTo  (m_face(cv::Rect(earBarR1_x, FACE_HEIGHT - earBarR1_y - earBar1_len, barWidth, earBar1_len)));

    return true;
}

void EarsThread::threadRelease()
{
    m_audioRecPort.close();
}

void EarsThread::resetToDefault()
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);
    m_doBars = true;
    m_earBar.setTo(m_earsDefaultColor);
    updateBars(0.5);
}

void EarsThread::setColor(float vr, float vg, float vb)
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);
    m_earsCurrentColor = Scalar(vr, vg, vb);
    m_earBar.setTo(m_earsCurrentColor);
}

void EarsThread::clearWithBlack()
{
    lock_guard<recursive_mutex> lg(m_methods_mutex);
    lock_guard<mutex> faceguard(m_drawing_mutex);
    m_blackBar(Rect(0, 0, barWidth, 32)).copyTo(m_face(cv::Rect(earBarL0_x, 0, barWidth, 32)));
    m_blackBar(Rect(0, 0, barWidth, 32)).copyTo(m_face(cv::Rect(earBarL1_x, 0, barWidth, 32)));
    m_blackBar(Rect(0, 0, barWidth, 32)).copyTo(m_face(cv::Rect(earBarR0_x, 0, barWidth, 32)));
    m_blackBar(Rect(0, 0, barWidth, 32)).copyTo(m_face(cv::Rect(earBarR1_x, 0, barWidth, 32)));
}

void EarsThread::enableDrawing(bool val)
{
    if (val)
    {
        m_drawEnable = true;
    }
    else
    {
        m_drawEnable=false;
        clearWithBlack();
    }
}
