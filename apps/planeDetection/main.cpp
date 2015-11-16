#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/cap_openni2.hpp>

using namespace cv;

int main(int argc, char * argv[])
{
    namedWindow("depth");
    namedWindow("color");
    namedWindow("normals");

    // Open Kinect sensor
    Ptr<CvCapture_OpenNI2> capture = new CvCapture_OpenNI2(0);
    if (!capture->isOpened())
    {
        printf("Could not open OpenNI-capable sensor\n");
        return -1;
    }
    capture->setProperty(CAP_PROP_OPENNI_REGISTRATION, 1);
    double focal_length = capture->getProperty(CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
    printf("Focal length = %f\n", focal_length);

    const int W = capture->getProperty(CAP_PROP_FRAME_WIDTH);
    const int H = capture->getProperty(CAP_PROP_FRAME_HEIGHT);
    int window_size = 5;
    float cx = W / 2.f + 0.5f;
    float cy = H / 2.f + 0.5f;

    Mat K = (Mat_<double>(3, 3) << focal_length, 0, cx, 0, focal_length, cy, 0, 0, 1);
    Mat Kinv = K.inv();

    Mat color, depth, normals;

    rgbd::RgbdPlane planeComputer;
    rgbd::RgbdNormals normalsComputer(H, W, CV_32F, K, window_size, rgbd::RgbdNormals::RGBD_NORMALS_METHOD_FALS);

    Mat plane_mask;
    std::vector<Vec4f> plane_coefficients;

    bool isLooping = true;
    while (isLooping)
    {
        capture->grabFrame();
        depth = capture->retrieveFrame(CAP_OPENNI_DEPTH_MAP);
        color = capture->retrieveFrame(CAP_OPENNI_BGR_IMAGE);

        Mat points3d;
        rgbd::depthTo3d(depth, K, points3d);
        normalsComputer(points3d, normals);
        planeComputer(points3d, normals, plane_mask, plane_coefficients);

        imshow("depth", depth);
        imshow("color", color);
        imshow("normals", normals);

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
    return 0;
}
