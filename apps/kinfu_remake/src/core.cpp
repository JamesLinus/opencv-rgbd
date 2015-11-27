#include <kfusion/kinfu.hpp>

#include <cuda.h>
#include <cstdio>
#include <iostream>

namespace 
{
    template <class T> inline void getCudaAttribute(T *attribute, CUdevice_attribute device_attribute, int device)
    {
        *attribute = T();
        CUresult error = cuDeviceGetAttribute( attribute, device_attribute, device );
        if( CUDA_SUCCESS == error ) 
            return;        

        printf("Driver API error = %04d\n", error);
        CV_Error(cv::Error::GpuApiCallError, "driver API error");
    }

    inline int convertSMVer2Cores(int major, int minor)
    {
        // Defines for GPU Architecture types (using the SM version to determine the # of cores per SM
        typedef struct {
            int SM; // 0xMm (hexidecimal notation), M = SM Major version, and m = SM minor version
            int Cores;
        } SMtoCores;

        SMtoCores gpuArchCoresPerSM[] =  { { 0x10,  8 }, { 0x11,  8 }, { 0x12,  8 }, { 0x13,  8 }, { 0x20, 32 }, { 0x21, 48 }, {0x30, 192}, {0x35, 192}, { -1, -1 }  };

        int index = 0;
        while (gpuArchCoresPerSM[index].SM != -1) 
        {
            if (gpuArchCoresPerSM[index].SM == ((major << 4) + minor) ) 
                return gpuArchCoresPerSM[index].Cores;
            index++;
        }
        printf("\nCan't determine number of cores. Unknown SM version %d.%d!\n", major, minor);
        return 0;
    }
}

kf::SampledScopeTime::SampledScopeTime(double& time_ms) : time_ms_(time_ms)
{
    start = (double)cv::getTickCount();
}
kf::SampledScopeTime::~SampledScopeTime()
{
    static int i_ = 0;
    time_ms_ += getTime ();
    if (i_ % EACH == 0 && i_)
    {
        std::cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )" << std::endl;
        time_ms_ = 0.0;
    }
    ++i_;
}

double kf::SampledScopeTime::getTime()
{
    return ((double)cv::getTickCount() - start)*1000.0/cv::getTickFrequency();
}

kf::ScopeTime::ScopeTime(const char *name_) : name(name_)
{
    start = (double)cv::getTickCount();
}

kf::ScopeTime::~ScopeTime()
{
    double time_ms =  ((double)cv::getTickCount() - start)*1000.0/cv::getTickFrequency();
    std::cout << "Time(" << name << ") = " << time_ms << "ms" << std::endl;
}
