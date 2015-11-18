#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/opengl.hpp>

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

using namespace cv;

struct RenderContext
{
    Point2i size;
    ogl::Arrays points;
    Mat plane_mask;
    std::vector<Vec4f> plane_coefficients;
};

void onRender(void* usr)
{
    RenderContext* context = (RenderContext*)usr;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, (double)context->size.x / context->size.y, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 0);
    //glScaled(2, 2, 2);
    glRotated(-180, 0, 0, 1);

    ogl::render(context->points);
}

int main(int argc, char * argv[])
{
    namedWindow("depth");
    namedWindow("cleanedDepth");
    namedWindow("color");
    namedWindow("normals");
    namedWindow("points", WINDOW_OPENGL);

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

    Mat color, depth, normals;
    Mat cleanedDepth;

    resizeWindow("points", W, H);

    RenderContext renderContext;
    renderContext.size = { W, H };
    setOpenGlDrawCallback("points", [](void* usr)
    {
        RenderContext* context = (RenderContext*)usr;

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60, (double)context->size.x / context->size.y, 0.1, 100.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 0);
        //glScaled(2, 2, 2);
        glRotated(-180, 0, 0, 1);

        ogl::render(context->points);
    }, (void*)&renderContext);

    rgbd::RgbdPlane planeComputer;
    rgbd::RgbdNormals normalsComputer(H, W, CV_32F, K, kWindowSize, rgbd::RgbdNormals::RGBD_NORMALS_METHOD_FALS);
    rgbd::DepthCleaner depthCleaner(CV_16UC1, kWindowSize);

    Vec3b kRandomColors[256];
    for (auto& clr : kRandomColors)
    {
        clr[0] = rand() % 256;
        clr[1] = rand() % 256;
        clr[2] = rand() % 256;
    }
    kRandomColors[255] = Vec3b(0, 0, 0);

    bool isLooping = true;
    while (isLooping)
    {
        capture.grab();
        capture.retrieve(depth, cv::CAP_OPENNI_DEPTH_MAP);
        capture.retrieve(color, cv::CAP_OPENNI_BGR_IMAGE);

        Mat points;
#if 1
        depthCleaner(depth, cleanedDepth);
#else
        cleanedDepth = depth;
#endif
        rgbd::depthTo3d(cleanedDepth, K, points);

        normalsComputer(points, normals);
        planeComputer(points, normals, renderContext.plane_mask, renderContext.plane_coefficients);

        Mat plane_mask_rgb;
        cvtColor(renderContext.plane_mask, plane_mask_rgb, COLOR_GRAY2RGB);
        for (int y = 0; y < plane_mask_rgb.rows; y++)
            for (int x = 0; x < plane_mask_rgb.cols; x++)
            {
                uchar label = renderContext.plane_mask.at<uchar>(y, x);
                auto& pixel = plane_mask_rgb.at<Vec3b>(y, x);
                pixel = kRandomColors[label];
            }

        renderContext.points.setVertexArray(points);
        //renderContext.points.setNormalArray(normals);
        renderContext.points.setColorArray(plane_mask_rgb);

        updateWindow("points");

        imshow("depth", depth);
        imshow("cleanedDepth", depth);
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
