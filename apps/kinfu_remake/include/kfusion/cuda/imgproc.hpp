#pragma once

#include <kfusion/types.hpp>

namespace kf
{
    namespace cuda
    {
        KF_EXPORTS void depthBilateralFilter(const Depth& in, Depth& out, int ksz, float sigma_spatial, float sigma_depth);

        KF_EXPORTS void depthTruncation(Depth& depth, float threshold);

        KF_EXPORTS void depthBuildPyramid(const Depth& depth, Depth& pyramid, float sigma_depth);

        KF_EXPORTS void computeNormalsAndMaskDepth(const Intr& intr, Depth& depth, Normals& normals);

        KF_EXPORTS void computePointNormals(const Intr& intr, const Depth& depth, Points& points, Normals& normals);

        KF_EXPORTS void computeDists(const Depth& depth, Dists& dists, const Intr& intr);

        KF_EXPORTS void resizeDepthNormals(const Depth& depth, const Normals& normals, Depth& depth_out, Normals& normals_out);

        KF_EXPORTS void resizePointsNormals(const Points& points, const Normals& normals, Points& points_out, Normals& normals_out);

        KF_EXPORTS void waitAllDefaultStream();

        KF_EXPORTS void renderTangentColors(const Normals& normals, Image& image);

        KF_EXPORTS void renderImage(const Depth& depth, const Normals& normals, const Intr& intr, const cv::Vec3f& light_pose, Image& image);

        KF_EXPORTS void renderImage(const Points& points, const Normals& normals, const Intr& intr, const cv::Vec3f& light_pose, Image& image);
    }
}
