#include <opencv2/rgbd.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/opengl.hpp>

#include <iostream>
#include <fstream>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN 1
#define NOMINMAX 1
#include <windows.h>
#endif

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "AntTweakBar.h"
#include "bx/fpumath.h"
#include "tiny_obj_loader.h"

using namespace std;
using namespace cv;
using namespace cv::rgbd;

#define BILATERAL_FILTER 0// if 1 then bilateral filter will be used for the depth

Mat model;
Vec3f dirU, dirV, dirW;
Vec4f quat;
Vec3f trans;

int main(int argc, char** argv)
{
    vector<Mat> Rts;

    // Open Kinect sensor
    cv::VideoCapture capture(cv::CAP_OPENNI2);
    if (!capture.isOpened())
    {
        printf("Could not open OpenNI-capable sensor\n");
        return -1;
    }
    capture.set(cv::CAP_PROP_OPENNI_REGISTRATION, 1);
    double focal_length = capture.get(cv::CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
    printf("Focal length = %f\n", focal_length);

    const int W = capture.get(CAP_PROP_FRAME_WIDTH);
    const int H = capture.get(CAP_PROP_FRAME_HEIGHT);
    int kWindowSize = 5;
    float cx = W / 2.f + 0.5f;
    float cy = H / 2.f + 0.5f;

    Mat K = (Mat_<double>(3, 3) << focal_length, 0, cx, 0, focal_length, cy, 0, 0, 1);
    Mat Kinv = K.inv();

    Ptr<OdometryFrame> frame_prev = Ptr<OdometryFrame>(new OdometryFrame()),
        frame_curr = Ptr<OdometryFrame>(new OdometryFrame());
    Ptr<Odometry> odometry = Odometry::create("RgbdOdometry"); // ICPOdometry, RgbdICPOdometry
    if (odometry.empty())
    {
        cout << "Can not create Odometry algorithm. Check the passed odometry name." << endl;
        return -1;
    }
    odometry->setCameraMatrix(K);

    namedWindow("kinfu_lite", WINDOW_OPENGL);
    resizeWindow("kinfu_lite", 800, 600);
    setOpenGlDrawCallback("kinfu_lite", [](void* userdata)
    {
        // Clear frame buffer
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Mat_<double> m = model;
        dirU[0] = m(0, 0);
        dirU[1] = m(1, 0);
        dirU[2] = m(2, 0);
        dirV[0] = m(0, 1);
        dirV[1] = m(1, 1);
        dirV[2] = m(2, 1);
        dirW[0] = m(0, 2);
        dirW[1] = m(1, 2);
        dirW[2] = m(2, 2);
        trans[0] = m(0, 3);
        trans[1] = m(1, 3);
        trans[2] = m(2, 3);

        // quat
        Mat R = model(Rect(0, 0, 3, 3)), rvec;
        Rodrigues(R, rvec);
        double alpha = norm(rvec);
        if (alpha > DBL_MIN)
            rvec = rvec / alpha;

        double cos_alpha2 = std::cos(0.5 * alpha);
        double sin_alpha2 = std::sin(0.5 * alpha);

        rvec *= sin_alpha2;

        quat[0] = rvec.at<double>(0);
        quat[1] = rvec.at<double>(1);
        quat[2] = rvec.at<double>(2);
        quat[3] = cos_alpha2;

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0, (double)640 / 480, 0.1, 100.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0, 0, 3, 0, 0, 0, 0, 1, 0);
        glMultMatrixf((float*)model.ptr());

        // bug here

        Mat_<Vec2f> vertex(1, 4);
        vertex << Vec2f(-1, 1), Vec2f(-1, -1), Vec2f(1, -1), Vec2f(1, 1);
        Mat_<int> indices(1, 6);
        indices << 0, 1, 2, 2, 3, 0;
        ogl::Arrays arr;
        arr.setVertexArray(vertex);
        
        ogl::render(arr, indices, ogl::TRIANGLES);

        TwDraw();
    });

    // Initialize AntTweakBar
    TwInit(TW_OPENGL, NULL);
    TwWindowSize(800, 600);
    auto bar = TwNewBar("TweakBar");

    TwAddVarRO(bar, "U", TW_TYPE_QUAT4D, &quat,
        " label='Light direction' open help='Change the light direction.' ");

    bool isLooping = true;
    while (isLooping)
    {
        Mat image, depth;
        {
            capture.grab();
            capture.retrieve(depth, cv::CAP_OPENNI_DEPTH_MAP);
            capture.retrieve(image, cv::CAP_OPENNI_BGR_IMAGE);

            CV_Assert(depth.type() == CV_16UC1);

            // scale depth
            Mat depth_flt;
            depth.convertTo(depth_flt, CV_32FC1, 1.f / 5000.f);
#if !BILATERAL_FILTER
            depth_flt.setTo(std::numeric_limits<float>::quiet_NaN(), depth == 0);
            depth = depth_flt;
#else
            tm_bilateral_filter.start();
            depth = Mat(depth_flt.size(), CV_32FC1, Scalar(0));
            const double depth_sigma = 0.03;
            const double space_sigma = 4.5;  // in pixels
            Mat invalidDepthMask = depth_flt == 0.f;
            depth_flt.setTo(-5 * depth_sigma, invalidDepthMask);
            bilateralFilter(depth_flt, depth, -1, depth_sigma, space_sigma);
            depth.setTo(std::numeric_limits<float>::quiet_NaN(), invalidDepthMask);
            tm_bilateral_filter.stop();
            cout << "Time filter " << tm_bilateral_filter.getTimeSec() << endl;
#endif
        }

        {
            Mat gray;
            cvtColor(image, gray, COLOR_BGR2GRAY);
            frame_curr->image = gray;
            frame_curr->depth = depth;

            imshow("image", gray);
            imshow("depth", depth);

            Mat Rt;
            if (Rts.empty())
            {
                model = Mat::eye(4, 4, CV_64FC1);
                Rts.push_back(model);
            }
            else
            {
                if (!odometry->compute(frame_curr, frame_prev, Rt))
                {
                    Rt = Mat::eye(4, 4, CV_64FC1);
                }

                Mat& prevRt = *Rts.rbegin();
                cout << "Rt " << Rt << endl;

                model = prevRt * Rt;
                Rts.push_back(model);
            }

            if (!frame_prev.empty())
                frame_prev->release();
            std::swap(frame_prev, frame_curr);

            updateWindow("kinfu_lite");

            char key = (char)waitKey(10);
            if (key == 'q')
                break;

            switch (key)
            {
            case 'q':
                isLooping = false;
                break;
            default:
                ;
            }
        }
    }

    TwTerminate();

    return 0;
}
