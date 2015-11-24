#include "precomp.hpp"

using namespace kf;
using namespace kf::cuda;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TsdfVolume::Entry

float kf::cuda::TsdfVolume::Entry::half2float(half)
{ throw "Not implemented"; }

kf::cuda::TsdfVolume::Entry::half kf::cuda::TsdfVolume::Entry::float2half(float value)
{ throw "Not implemented"; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TsdfVolume

kf::cuda::TsdfVolume::TsdfVolume(const cv::Vec3i& dims) : data_(), trunc_dist_(0.03f), max_weight_(128), dims_(dims),
    size_(cv::Vec3f::all(3.f)), pose_(cv::Affine3f::Identity()), gradient_delta_factor_(0.75f), raycast_step_factor_(0.75f)
{ create(dims_); }

kf::cuda::TsdfVolume::~TsdfVolume() {}

void kf::cuda::TsdfVolume::create(const cv::Vec3i& dims)
{
    dims_ = dims;
    int voxels_number = dims_[0] * dims_[1] * dims_[2];
    data_.create(voxels_number * sizeof(int));
    setTruncDist(trunc_dist_);
    clear();
}

cv::Vec3i kf::cuda::TsdfVolume::getDims() const
{ return dims_; }

cv::Vec3f kf::cuda::TsdfVolume::getVoxelSize() const
{
    return cv::Vec3f(size_[0]/dims_[0], size_[1]/dims_[1], size_[2]/dims_[2]);
}

const cuda::Memory kf::cuda::TsdfVolume::data() const { return data_; }
cuda::Memory kf::cuda::TsdfVolume::data() {  return data_; }
cv::Vec3f kf::cuda::TsdfVolume::getSize() const { return size_; }

void kf::cuda::TsdfVolume::setSize(const cv::Vec3f& size)
{ size_ = size; setTruncDist(trunc_dist_); }

float kf::cuda::TsdfVolume::getTruncDist() const { return trunc_dist_; }

void kf::cuda::TsdfVolume::setTruncDist(float distance)
{
    cv::Vec3f vsz = getVoxelSize();
    float max_coeff = std::max<float>(std::max<float>(vsz[0], vsz[1]), vsz[2]);
    trunc_dist_ = std::max (distance, 2.1f * max_coeff);
}

int kf::cuda::TsdfVolume::getMaxWeight() const { return max_weight_; }
void kf::cuda::TsdfVolume::setMaxWeight(int weight) { max_weight_ = weight; }
cv::Affine3f kf::cuda::TsdfVolume::getPose() const  { return pose_; }
void kf::cuda::TsdfVolume::setPose(const cv::Affine3f& pose) { pose_ = pose; }
float kf::cuda::TsdfVolume::getRaycastStepFactor() const { return raycast_step_factor_; }
void kf::cuda::TsdfVolume::setRaycastStepFactor(float factor) { raycast_step_factor_ = factor; }
float kf::cuda::TsdfVolume::getGradientDeltaFactor() const { return gradient_delta_factor_; }
void kf::cuda::TsdfVolume::setGradientDeltaFactor(float factor) { gradient_delta_factor_ = factor; }
void kf::cuda::TsdfVolume::swap(cuda::Memory& data) { data_.swap(data); }
void kf::cuda::TsdfVolume::applyAffine(const cv::Affine3f& affine) { pose_ = affine * pose_; }

void kf::cuda::TsdfVolume::clear()
{ 
    impl::Vec3i dims = device_cast<impl::Vec3i>(dims_);
    impl::Vec3f vsz  = device_cast<impl::Vec3f>(getVoxelSize());

    impl::TsdfVolume volume(data_.ptr<ushort2>(), dims, vsz, trunc_dist_, max_weight_);
    impl::clear_volume(volume);
}

void kf::cuda::TsdfVolume::integrate(const Dists& dists, const cv::Affine3f& camera_pose, const Intr& intr)
{
    cv::Affine3f vol2cam = camera_pose.inv() * pose_;

    impl::Projector proj(intr.fx, intr.fy, intr.cx, intr.cy);

    impl::Vec3i dims = device_cast<impl::Vec3i>(dims_);
    impl::Vec3f vsz  = device_cast<impl::Vec3f>(getVoxelSize());
    impl::Aff3f aff = device_cast<impl::Aff3f>(vol2cam);

    impl::TsdfVolume volume(data_.ptr<ushort2>(), dims, vsz, trunc_dist_, max_weight_);
    impl::integrate(dists, volume, aff, proj);
}

void kf::cuda::TsdfVolume::raycast(const cv::Affine3f& camera_pose, const Intr& intr, Depth& depth, Normals& normals)
{
    Array2D<impl::Normal>& n = (Array2D<impl::Normal>&)normals;

    cv::Affine3f cam2vol = pose_.inv() * camera_pose;

    impl::Aff3f aff = device_cast<impl::Aff3f>(cam2vol);
    impl::Mat3f Rinv = device_cast<impl::Mat3f>(cam2vol.rotation().inv(cv::DECOMP_SVD));

    impl::Reprojector reproj(intr.fx, intr.fy, intr.cx, intr.cy);

    impl::Vec3i dims = device_cast<impl::Vec3i>(dims_);
    impl::Vec3f vsz  = device_cast<impl::Vec3f>(getVoxelSize());

    impl::TsdfVolume volume(data_.ptr<ushort2>(), dims, vsz, trunc_dist_, max_weight_);
    impl::raycast(volume, aff, Rinv, reproj, depth, n, raycast_step_factor_, gradient_delta_factor_);

}

void kf::cuda::TsdfVolume::raycast(const cv::Affine3f& camera_pose, const Intr& intr, Points& points, Normals& normals)
{
    impl::Normals& n = (impl::Normals&)normals;
    impl::Points& p = (impl::Points&)points;

    cv::Affine3f cam2vol = pose_.inv() * camera_pose;

    impl::Aff3f aff = device_cast<impl::Aff3f>(cam2vol);
    impl::Mat3f Rinv = device_cast<impl::Mat3f>(cam2vol.rotation().inv(cv::DECOMP_SVD));

    impl::Reprojector reproj(intr.fx, intr.fy, intr.cx, intr.cy);

    impl::Vec3i dims = device_cast<impl::Vec3i>(dims_);
    impl::Vec3f vsz  = device_cast<impl::Vec3f>(getVoxelSize());

    impl::TsdfVolume volume(data_.ptr<ushort2>(), dims, vsz, trunc_dist_, max_weight_);
    impl::raycast(volume, aff, Rinv, reproj, p, n, raycast_step_factor_, gradient_delta_factor_);
}

Array<Point> kf::cuda::TsdfVolume::fetchPoints(Array<Point>& Points_buffer) const
{
    enum { DEFAULT_Points_BUFFER_SIZE = 10 * 1000 * 1000 };

    if (Points_buffer.empty ())
        Points_buffer.create (DEFAULT_Points_BUFFER_SIZE);

    Array<impl::Point>& b = (Array<impl::Point>&)Points_buffer;

    impl::Vec3i dims = device_cast<impl::Vec3i>(dims_);
    impl::Vec3f vsz  = device_cast<impl::Vec3f>(getVoxelSize());
    impl::Aff3f aff  = device_cast<impl::Aff3f>(pose_);

    impl::TsdfVolume volume((ushort2*)data_.ptr<ushort2>(), dims, vsz, trunc_dist_, max_weight_);
    size_t size = extractPoints(volume, aff, b);

    return Array<Point>((Point*)Points_buffer.ptr(), size);
}

void kf::cuda::TsdfVolume::fetchNormals(const Array<Point>& Points, Array<Normal>& normals) const
{
    normals.create(Points.size());
    Array<impl::Point>& c = (Array<impl::Point>&)Points;

    impl::Vec3i dims = device_cast<impl::Vec3i>(dims_);
    impl::Vec3f vsz  = device_cast<impl::Vec3f>(getVoxelSize());
    impl::Aff3f aff  = device_cast<impl::Aff3f>(pose_);
    impl::Mat3f Rinv = device_cast<impl::Mat3f>(pose_.rotation().inv(cv::DECOMP_SVD));

    impl::TsdfVolume volume((ushort2*)data_.ptr<ushort2>(), dims, vsz, trunc_dist_, max_weight_);
    impl::extractNormals(volume, c, aff, Rinv, gradient_delta_factor_, (float4*)normals.ptr());
}
