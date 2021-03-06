#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/cuda.hpp>
//#include <opencv2/viz/vizcore.hpp>
#include <kfusion/kinfu.hpp>
#include <time.h>

using namespace kf;

struct KinFuApp
{
    //static void KeyboardCallback(const cv::viz::KeyboardEvent& event, void* pthis)
    //{
    //    KinFuApp& kinfu = *static_cast<KinFuApp*>(pthis);

    //    if(event.action != cv::viz::KeyboardEvent::KEY_DOWN)
    //        return;

    //    if(event.code == 't' || event.code == 'T')
    //        kinfu.take_cloud(*kinfu.kinfu_);

    //    if(event.code == 'i' || event.code == 'I')
    //        kinfu.iteractive_mode_ = !kinfu.iteractive_mode_;
    //}

    KinFuApp(cv::VideoCapture& source) : exit_(false), iteractive_mode_(false), capture_(source)
    {
        KinFuParams params = KinFuParams::default_params();
        kinfu_ = KinFu::Ptr( new KinFu(params) );

        capture_.set(cv::CAP_PROP_OPENNI_REGISTRATION, 1);

        //cv::viz::WCube cube(cv::Vec3d::all(0), cv::Vec3d(params.volume_size), true, cv::viz::Color::apricot());
        //viz.showWidget("cube", cube, params.volume_pose);
        //viz.showWidget("coor", cv::viz::WCoordinateSystem(0.1));
        //viz.registerKeyboardCallback(KeyboardCallback, this);
    }

    void show_depth(const cv::Mat& depth)
    {
        cv::Mat display;
        //cv::normalize(depth, display, 0, 255, cv::NORM_MINMAX, CV_8U);
        depth.convertTo(display, CV_8U, 255.0/4000);
        cv::imshow("Depth", display);
    }

    void show_raycasted(KinFu& kinfu)
    {
        const int mode = 3;
        if (iteractive_mode_)
        {
            //kinfu.renderImage(view_device_, viz.getViewerPose(), mode);
        }
        else
            kinfu.renderImage(view_device_, mode);

        view_host_.create(view_device_.rows(), view_device_.cols(), CV_8UC4);
        view_device_.download(view_host_.ptr<void>(), view_host_.step);
        cv::imshow("Scene", view_host_);
    }

    void take_cloud(KinFu& kinfu)
    {
        cuda::Array<Point> cloud = kinfu.tsdf().fetchPoints(cloud_buffer);
        cv::Mat cloud_host(1, (int)cloud.size(), CV_32FC4);
        cloud.download(cloud_host.ptr<Point>());

        char meshFileName[256];
        time_t now = time(NULL);
        strftime(meshFileName, 100, "points-%Y-%m-%d_%H-%M-%S.asc", localtime(&now));
        printf("Writing cloud points to %s\n", meshFileName);
        FILE *f = fopen(meshFileName, "w+");
        if (f != NULL)
        {
            for (uint i = 0; i < cloud.size(); i++)
            {
                auto& point = cloud_host.at<Point>(i);
                fprintf(f, "%f %f %f\n", point.x, point.y, point.z);
            }
            fclose(f);
        }
        //viz.showWidget("cloud", cv::viz::WCloud(cloud_host));
        //viz.showWidget("cloud", cv::viz::WPaintedCloud(cloud_host));
    }

    bool execute()
    {
        KinFu& kinfu = *kinfu_;
        cv::Mat depth, color;
        double time_ms = 0;
        bool has_image = false;

        for (int i = 0; !exit_ /*&& !viz.wasStopped()*/; ++i)
        {
            bool has_frame = capture_.grab();
            if (!has_frame)
                return std::cout << "Can't grab" << std::endl, false;

            capture_.retrieve(depth, cv::CAP_OPENNI_DEPTH_MAP);
            capture_.retrieve(color, cv::CAP_OPENNI_BGR_IMAGE);

            depth_device_.upload(depth.data, depth.step, depth.rows, depth.cols);
            color_device_.upload(color.data, color.step, color.rows, color.cols);

            {
                SampledScopeTime fps(time_ms); (void)fps;
                has_image = kinfu(depth_device_, color_device_);
            }

            if (has_image)
                show_raycasted(kinfu);

            show_depth(depth);
            //cv::imshow("Image", image);

            //if (!iteractive_mode_)
            //    viz.setViewerPose(kinfu.getCameraPose());

            int key = cv::waitKey(3);

            switch(key)
            {
            case 't': case 'T' : take_cloud(kinfu); break;
            case 'i': case 'I' : iteractive_mode_ = !iteractive_mode_; break;
            case 27: case 32: exit_ = true; break;
            }

            //exit_ = exit_ || i > 100;
            //viz.spinOnce(3, true);
        }
        return true;
    }

    bool exit_, iteractive_mode_;
    cv::VideoCapture& capture_;
    KinFu::Ptr kinfu_;
    //cv::viz::Viz3d viz;

    cv::Mat view_host_;
    cuda::Image view_device_;
    cuda::Depth depth_device_;
    cuda::Array2D<PixelRGB> color_device_;
    cuda::Array<Point> cloud_buffer;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool checkIfPreFermiGPU(int device)
{
    if (device < 0)
        cudaSafeCall(cudaGetDevice(&device));

    cudaDeviceProp prop;
    cudaSafeCall(cudaGetDeviceProperties(&prop, device));
    return prop.major < 2; // CC == 1.x
}

int main (int argc, char* argv[])
{
    int device = 0;
    cv::cuda::setDevice (device);
    cv::cuda::printShortCudaDeviceInfo(device);

    //cv::cuda::GpuMat mat(10, 10, CV_8UC3);

    if (checkIfPreFermiGPU(device))
        return std::cout << std::endl << "Kinfu is not supported for pre-Fermi GPU architectures, and not built for them by default. Exiting..." << std::endl, 1;

    cv::VideoCapture capture(cv::CAP_OPENNI2);
    if (!capture.isOpened())
    {
        printf("Could not open OpenNI-capable sensor\n");
        return -1;
    }
    //capture.open("d:/onis/20111013-224932.oni");
    //capture.open("d:/onis/reg20111229-180846.oni");
    //capture.open("d:/onis/white1.oni");
    //capture.open("/media/Main/onis/20111013-224932.oni");
    //capture.open("20111013-225218.oni");
    //capture.open("d:/onis/20111013-224551.oni");
    //capture.open("d:/onis/20111013-224719.oni");

    KinFuApp app (capture);

    // executing
    try { app.execute (); }
    catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; }
    catch (const std::exception& /*e*/) { std::cout << "Exception" << std::endl; }

    return 0;
}
