#pragma once

#include <kfusion/types.hpp>

namespace kf
{
    namespace cuda
    {
        void depthBilateralFilter(const Depth& in, Depth& out, int ksz, float sigma_spatial, float sigma_depth);

        void depthTruncation(Depth& depth, float threshold);

        void depthBuildPyramid(const Depth& depth, Depth& pyramid, float sigma_depth);

        void computeNormalsAndMaskDepth(const Intr& intr, Depth& depth, Normals& normals);

        void computePointNormals(const Intr& intr, const Depth& depth, Points& points, Normals& normals);

        void computeDists(const Depth& depth, Dists& dists, const Intr& intr);

        void resizeDepthNormals(const Depth& depth, const Normals& normals, Depth& depth_out, Normals& normals_out);

        void resizePointsNormals(const Points& points, const Normals& normals, Points& points_out, Normals& normals_out);

        void waitAllDefaultStream();

        void renderTangentColors(const Normals& normals, Image& image);

        void renderImage(const Depth& depth, const Normals& normals, const Intr& intr, const cv::Vec3f& light_pose, Image& image);

        void renderImage(const Points& points, const Normals& normals, const Intr& intr, const cv::Vec3f& light_pose, Image& image);
    }
}
