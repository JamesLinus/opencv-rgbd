#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char * argv[])
{
    cv::namedWindow("depth");
    cv::namedWindow("color");

    // Open Kinect sensor
    cv::VideoCapture capture(cv::CAP_OPENNI2);
    if (!capture.isOpened())
    {
        printf("Could not open OpenNI-capable sensor\n");
        return -1;
    }
    capture.set(cv::CAP_PROP_OPENNI_REGISTRATION, 1);
    double focal_length = capture.get(cv::CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
    //printf("Focal length = %f\n", focal_length);

    cv::Mat color, depth;
    bool isLooping = true;
    while (isLooping)
    {
        capture.grab();
        capture.retrieve(depth, cv::CAP_OPENNI_DEPTH_MAP);
        capture.retrieve(color, cv::CAP_OPENNI_BGR_IMAGE);

        cv::imshow("depth", depth);
        cv::imshow("color", color);

        char key = (char)cv::waitKey(10);
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
