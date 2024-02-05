#include <cmath>
#include <string>
#include "utils.hpp"
#include "eyesThread.hpp"

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

EyesThread::EyesThread(ResourceFinder& _rf, std::string _moduleName, double _period, cv::Mat& _image, std::mutex& _mutex) :
    PeriodicThread(_period),
    m_face(_image),
    m_drawing_mutex(_mutex),
    m_rf(_rf),
    m_moduleName(_moduleName)
{
    //indexes
    indexes[0] = 0;
    indexes[1] = 1;
    indexes[2] = 2;
    indexes[3] = 3;
    indexes[4] = 4;
    indexes[5] = 5;
    indexes[6] = 4;
    indexes[7] = 3;
    indexes[8] = 2;
    indexes[9] = 1;
    indexes[10] = 0;

    //delays
    delays[0] = 0.045;
    delays[1] = 0.045;
    delays[2] = 0.045;
    delays[3] = 0.015;
    delays[4] = 0.015;
    delays[5] = 0.015;
    delays[6] = 0.065;
    delays[7] = 0.065;
    delays[8] = 0.065;
    delays[9] = 0.065;
    delays[10] = 0.065;
}

void EyesThread::afterStart(bool s) { }

void EyesThread::activateBlink(bool activate)
{
    lock_guard<mutex> lg(recursive_mutex);
    m_doBlink = activate;
}

bool EyesThread::threadInit()
{
    if (!getPath(m_rf, m_imagePath)) return false;

    //load eyes
    bool ok = true;
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_1.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_2.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_3.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_4.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_5.bmp").c_str(), cv::IMREAD_COLOR));
    blinkEye.push_back(cv::imread(std::string(m_imagePath + "/blink_6.bmp").c_str(), cv::IMREAD_COLOR));
    for (int i = 0; i < blinkEye.size(); i++)
    {
        if (blinkEye[i].empty())
        {
            yError() << "Image number " << i << "required for blink sequence was not found.";
            ok = false;
        }
    }
    // Load nose
    noseBar = cv::imread(std::string(m_imagePath + "/noseBar.bmp").c_str(), cv::IMREAD_COLOR);
    noseBar0_len = noseBar.cols;

    if (noseBar.empty())
    {
        yError() << "Image required for nose was not found.";
        ok = false;
    }

    // Quit if something wrong!!
    if (!ok)
    {
        yError() << "I am searching in path: " << m_imagePath << ", use option --path to change the path";
        return false;
    }

    // Reset image to black
    faceBlack.create(FACE_HEIGHT, FACE_WIDTH, CV_8UC3);
    faceBlack.setTo(cv::Scalar(0, 0, 0));

    faceRest.create(FACE_HEIGHT, FACE_WIDTH, CV_8UC3);
    faceRest.setTo(cv::Scalar(0,0,0));

    // Copy eyes
    ((Mat)blinkEye[indexes[0]]).copyTo(faceRest(cv::Rect(leftEye_x,  leftEye_y,  eyeWidth, eyeHeight)));
    cv::Mat flippedEye;
    cv::flip(blinkEye[indexes[index]], flippedEye, 1); // flipping the right eye on its vertical axis
    flippedEye.copyTo(faceRest(cv::Rect(rightEye_x, rightEye_y, eyeWidth, eyeHeight)));
    // Add nose
    noseBar.copyTo(faceRest(cv::Rect(noseBar0_x, noseBar0_y, noseBar.cols, noseBar.rows)));

    this->resetToDefault();
    return true;
}

void EyesThread::run()
{
    lock_guard<mutex> lg(recursive_mutex);

    if (m_drawEnable == false)
    {
        clearWithBlack();
        return;
    }

    if (m_doBlink ==false)
    {
        updateBlink(0);
        return;
    }

    bool  doubleBlink = false;
    bool  executeBlink = false;
    float blinks_per_minute = 15;
    float extra_blinks_per_minute = ((float)rand() / (float)RAND_MAX) * 6;
    float blinks_per_second = (blinks_per_minute+ extra_blinks_per_minute)/60;
    float period_sec = 1/ blinks_per_second;

    if (yarp::os::Time::now()-m_last_blink >= period_sec)
    {
        executeBlink = true;
        float r = ((float)rand() / (float)RAND_MAX) * 100;
        if (r < 30) //30% of probability of double blink
        {
            doubleBlink = true;
        }
        else
        {
            doubleBlink = false;
        }
        m_last_blink = yarp::os::Time::now();
    }

    do
    {
        if(executeBlink)
        {
            index++;
            updateBlink(index);

            if(index >= BLINK_STEP_NUM)
            {
                index = 0;
                if (doubleBlink==true)
                {
                    doubleBlink = false;
                }
                else
                {
                    executeBlink = false;
                }
            }
        }

        if(executeBlink)
        {
            yarp::os::Time::delay(delays[index]);
        }
    }
    while(executeBlink);
}

bool EyesThread::updateBlink(int index)
{
    lock_guard<mutex> faceguard(m_drawing_mutex);

    // Copy eyes
    ((Mat)blinkEye[indexes[index]]).copyTo(m_face(cv::Rect(leftEye_x,  leftEye_y,  eyeWidth, eyeHeight)));
    cv::Mat flippedEye;
    cv::flip(blinkEye[indexes[index]], flippedEye, 1); // flipping the right eye on its vertical axis
    flippedEye.copyTo(m_face(cv::Rect(rightEye_x, rightEye_y, eyeWidth, eyeHeight)));

    // Add nose
    noseBar.copyTo(m_face(cv::Rect(noseBar0_x, noseBar0_y, noseBar.cols, noseBar.rows)));

    return true;
}

void EyesThread::resetToDefault(bool _blink)
{
    lock_guard<mutex> lg(recursive_mutex);
    lock_guard<mutex> faceguard(m_drawing_mutex);
    m_doBlink=_blink;
    m_last_blink = yarp::os::Time::now();
    faceRest.copyTo(m_face);
}

void EyesThread::threadRelease()
{
}

void EyesThread::clearWithBlack()
{
    lock_guard<mutex> lg(recursive_mutex);
    lock_guard<mutex> faceguard(m_drawing_mutex);
    faceBlack.copyTo(m_face(cv::Rect(0, 0, FACE_WIDTH, FACE_HEIGHT)));
}

void EyesThread::enableDrawing(bool val)
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
