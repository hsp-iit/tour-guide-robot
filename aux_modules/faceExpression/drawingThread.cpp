#include <cmath>
#include <string>
#include "drawingThread.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <yarp/os/Time.h>
#include <yarp/os/Searchable.h>
#include <yarp/math/Rand.h>


#define BLINK_STEP_NUM  10
#define FACE_WIDTH      80
#define FACE_HEIGHT     32

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::math;

DrawingThread::DrawingThread(ResourceFinder& _rf, string _moduleName, double _period, cv::Mat& _image, std::mutex& _mutex):
               PeriodicThread(_period),
               m_image(_image),
               m_mutex(_mutex),
               m_rf(_rf),
               m_moduleName(_moduleName)
{
}

void DrawingThread::afterStart(bool s) { }

bool DrawingThread::threadInit()
{
    if (m_imageOutPort.open("/"+ m_moduleName + "/image:o") == false)
    {
        yError() << "Cannot open port";
        return false;
    }
    return true;
}

void DrawingThread::run()
{
    lock_guard<mutex> lg(m_mutex);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = m_imageOutPort.prepare();
    img.setExternal(m_image.data, FACE_WIDTH, FACE_HEIGHT);
    m_imageOutPort.writeStrict();
}

void DrawingThread::threadRelease()
{
    m_imageOutPort.close();
}

void DrawingThread::blackReset()
{
    lock_guard<mutex> lg(m_mutex);
    m_image.setTo((Scalar(0, 0, 0)));
}
