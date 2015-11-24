#pragma once

#include <kfusion/types.hpp>
#include <kfusion/kinfu.hpp>
#include <kfusion/cuda/tsdf_volume.hpp>
#include <kfusion/cuda/imgproc.hpp>
#include <kfusion/cuda/projective_icp.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
#include "internal.hpp"
#include <iostream>
#include "vector_functions.h"

namespace kf
{
    template<typename D, typename S>
    inline D device_cast(const S& source)
    {
        return *reinterpret_cast<const D*>(source.val);
    }

    template<>
    inline impl::Aff3f device_cast<impl::Aff3f, cv::Affine3f>(const cv::Affine3f& source)
    {
        impl::Aff3f aff;
        cv::Matx33f R = source.rotation();
        cv::Vec3f t = source.translation();
        aff.R = device_cast<impl::Mat3f>(R);
        aff.t = device_cast<impl::Vec3f>(t);
        return aff;
    }
}
