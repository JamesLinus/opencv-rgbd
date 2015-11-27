#include <kfusion/kinfu.hpp>

#include <cuda.h>
#include <cstdio>
#include <iostream>

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
