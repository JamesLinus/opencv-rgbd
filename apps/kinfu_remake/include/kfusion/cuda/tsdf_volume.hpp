#pragma once

#include <kfusion/types.hpp>

namespace kf
{
    namespace cuda
    {
        class KF_EXPORTS TsdfVolume
        {
        public:
            TsdfVolume(const cv::Vec3i& dims);
            virtual ~TsdfVolume();

            void create(const cv::Vec3i& dims);

            cv::Vec3i getDims() const;
            cv::Vec3f getVoxelSize() const;

            const cuda::Memory data() const;
            cuda::Memory data();

            cv::Vec3f getSize() const;
            void setSize(const cv::Vec3f& size);

            float getTruncDist() const;
            void setTruncDist(float distance);

            int getMaxWeight() const;
            void setMaxWeight(int weight);

            cv::Affine3f getPose() const;
            void setPose(const cv::Affine3f& pose);

            float getRaycastStepFactor() const;
            void setRaycastStepFactor(float factor);

            float getGradientDeltaFactor() const;
            void setGradientDeltaFactor(float factor);

            cv::Vec3i getGridOrigin() const;
            void setGridOrigin(const cv::Vec3i& origin);

            virtual void clear();
            virtual void applyAffine(const cv::Affine3f& affine);
            virtual void integrate(const Dists& dists, const cv::Affine3f& camera_pose, const Intr& intr);
            virtual void raycast(const cv::Affine3f& camera_pose, const Intr& intr, Depth& depth, Normals& normals);
            virtual void raycast(const cv::Affine3f& camera_pose, const Intr& intr, Points& points, Normals& normals);

            void swap(cuda::Memory& data);

            Array<Point> fetchPoints(Array<Point>& cloud_buffer) const;
            void fetchNormals(const Array<Point>& cloud, Array<Normal>& normals) const;

            struct Entry
            {
                typedef unsigned short half;

                half tsdf;
                unsigned short weight;

                static float half2float(half value);
                static half float2half(float value);
            };
        private:
            cuda::Memory data_;

            float trunc_dist_;
            int max_weight_;
            cv::Vec3i dims_;
            cv::Vec3f size_;
            cv::Affine3f pose_;

            float gradient_delta_factor_;
            float raycast_step_factor_;
        };
    }
}
