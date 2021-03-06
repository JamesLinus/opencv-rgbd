#pragma once

#include <kfusion/types.hpp>
#include <kfusion/cuda/tsdf_volume.hpp>
#include <kfusion/cuda/projective_icp.hpp>
#include <vector>
#include <string>

namespace kf
{
    struct KinFuParams
    {
        static KinFuParams default_params();

        int cols;  //pixels
        int rows;  //pixels

        Intr intr;  //Camera parameters

        cv::Vec3i volume_dims; //number of voxels
        cv::Vec3f volume_size; //meters
        cv::Affine3f volume_pose; //meters, inital pose

        float bilateral_sigma_depth;   //meters
        float bilateral_sigma_spatial;   //pixels
        int   bilateral_kernel_size;   //pixels

        float icp_truncate_depth_dist; //meters
        float icp_dist_thres;          //meters
        float icp_angle_thres;         //radians
        std::vector<int> icp_iter_num; //iterations for level index 0,1,..,3

        float tsdf_min_camera_movement; //meters, integrate only if exceedes
        float tsdf_trunc_dist;             //meters;
        int tsdf_max_weight;               //frames

        float raycast_step_factor;   // in voxel sizes
        float gradient_delta_factor; // in voxel sizes

        cv::Vec3f light_pose; //meters

    };

    class KinFu
    {
    public:        
        typedef cv::Ptr<KinFu> Ptr;

        KinFu(const KinFuParams& params);

        const KinFuParams& params() const;
        KinFuParams& params();

        const cuda::TsdfVolume& tsdf() const;
        cuda::TsdfVolume& tsdf();

        const cuda::ProjectiveICP& icp() const;
        cuda::ProjectiveICP& icp();

        void reset();

        bool operator()(const cuda::Depth& depth, const cuda::Array2D<PixelRGB>& color = cuda::Array2D<PixelRGB>());

        void renderImage(cuda::Image& image, int flags = 0);
        void renderImage(cuda::Image& image, const cv::Affine3f& pose, int flags = 0);

        const cv::Affine3f& getCameraPose (int time = -1) const;
    private:
        void allocate_buffers();

        int frame_counter_;
        KinFuParams params_;

        std::vector<cv::Affine3f> poses_;

        cuda::Dists dists_;
        cuda::Frame curr_, prev_;

        cuda::Points points_;
        cuda::Normals normals_;
        cuda::Depth depths_;

        cv::Ptr<cuda::TsdfVolume> volume_;
        cv::Ptr<cuda::ProjectiveICP> icp_;
    };
}
