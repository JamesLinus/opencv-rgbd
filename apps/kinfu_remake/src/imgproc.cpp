#include "precomp.hpp"

void kf::cuda::depthBilateralFilter(const Depth& in, Depth& out, int kernel_size, float sigma_spatial, float sigma_depth)
{ 
    out.create(in.rows(), in.cols());
    impl::bilateralFilter(in, out, kernel_size, sigma_spatial, sigma_depth);
}

void kf::cuda::depthTruncation(Depth& depth, float threshold)
{ impl::truncateDepth(depth, threshold); }

void kf::cuda::depthBuildPyramid(const Depth& depth, Depth& pyramid, float sigma_depth)
{ 
    pyramid.create (depth.rows () / 2, depth.cols () / 2);
    impl::depthPyr(depth, pyramid, sigma_depth);
}

void kf::cuda::waitAllDefaultStream()
{ cudaSafeCall(cudaDeviceSynchronize() ); }

void kf::cuda::computeNormalsAndMaskDepth(const Intr& intr, Depth& depth, Normals& normals)
{
    normals.create(depth.rows(), depth.cols());

    impl::Reprojector reproj(intr.fx, intr.fy, intr.cx, intr.cy);

    impl::Normals& n = (impl::Normals&)normals;
    impl::computeNormalsAndMaskDepth(reproj, depth, n);
}

void kf::cuda::computePointNormals(const Intr& intr, const Depth& depth, Points& points, Normals& normals)
{
    points.create(depth.rows(), depth.cols());
    normals.create(depth.rows(), depth.cols());

    impl::Reprojector reproj(intr.fx, intr.fy, intr.cx, intr.cy);

    impl::Points& p = (impl::Points&)points;
    impl::Normals& n = (impl::Normals&)normals;
    impl::computePointNormals(reproj, depth, p, n);
}


void kf::cuda::computeDists(const Depth& depth, Dists& dists, const Intr& intr)
{
    dists.create(depth.rows(), depth.cols());
    impl::compute_dists(depth, dists, make_float2(intr.fx, intr.fy), make_float2(intr.cx, intr.cy));
}

void kf::cuda::resizeDepthNormals(const Depth& depth, const Normals& normals, Depth& depth_out, Normals& normals_out)
{
    depth_out.create (depth.rows()/2, depth.cols()/2);
    normals_out.create (normals.rows()/2, normals.cols()/2);

    impl::Normals& nsrc = (impl::Normals&)normals;
    impl::Normals& ndst = (impl::Normals&)normals_out;

    impl::resizeDepthNormals(depth, nsrc, depth_out, ndst);
}

void kf::cuda::resizePointsNormals(const Points& points, const Normals& normals, Points& points_out, Normals& normals_out)
{
    points_out.create (points.rows()/2, points.cols()/2);
    normals_out.create (normals.rows()/2, normals.cols()/2);

    impl::Points& pi = (impl::Points&)points;
    impl::Normals& ni= (impl::Normals&)normals;

    impl::Points& po = (impl::Points&)points_out;
    impl::Normals& no = (impl::Normals&)normals_out;

    impl::resizePointsNormals(pi, ni, po, no);
}


void kf::cuda::renderImage(const Depth& depth, const Normals& normals, const Intr& intr, const cv::Vec3f& light_pose, Image& image)
{
    image.create(depth.rows(), depth.cols());

    const impl::Depth& d = (const impl::Depth&)depth;
    const impl::Normals& n = (const impl::Normals&)normals;
    impl::Reprojector reproj(intr.fx, intr.fy, intr.cx, intr.fy);
    impl::Vec3f light = device_cast<impl::Vec3f>(light_pose);

    impl::Image& i = (impl::Image&)image;
    impl::renderImage(d, n, reproj, light, i);
    waitAllDefaultStream();
}

void kf::cuda::renderImage(const Points& points, const Normals& normals, const Intr& intr, const cv::Vec3f& light_pose, Image& image)
{
    image.create(points.rows(), points.cols());

    const impl::Points& p = (const impl::Points&)points;
    const impl::Normals& n = (const impl::Normals&)normals;
    impl::Reprojector reproj(intr.fx, intr.fy, intr.cx, intr.fy);
    impl::Vec3f light = device_cast<impl::Vec3f>(light_pose);

    impl::Image& i = (impl::Image&)image;
    impl::renderImage(p, n, reproj, light, i);
    waitAllDefaultStream();
}

void kf::cuda::renderTangentColors(const Normals& normals, Image& image)
{
    image.create(normals.rows(), normals.cols());
    const impl::Normals& n = (const impl::Normals&)normals;
    impl::Image& i = (impl::Image&)image;

    impl::renderTangentColors(n, i);
    waitAllDefaultStream();
}
