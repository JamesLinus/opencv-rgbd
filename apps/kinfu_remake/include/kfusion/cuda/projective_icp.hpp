#pragma once

#include <kfusion/types.hpp>

namespace kf
{
    namespace cuda
    {
        class ProjectiveICP
        {
        public:
            enum { MAX_PYRAMID_LEVELS = 4 };

            ProjectiveICP();
            virtual ~ProjectiveICP();

            float getDistThreshold() const;
            void setDistThreshold(float distance);

            float getAngleThreshold() const;
            void setAngleThreshold(float angle);

            void setIterationsNum(const std::vector<int>& iters);
            int getUsedLevelsNum() const;

            virtual bool estimateTransform(cv::Affine3f& affine, const Intr& intr, const Frame& curr, const Frame& prev);

            /** The function takes masked depth, i.e. it assumes for performance reasons that
              * "if depth(y,x) is not zero, then normals(y,x) surely is not qnan" */
            virtual bool estimateTransform(cv::Affine3f& affine, const Intr& intr,
                const std::vector<Depth>& dcurr, const std::vector<Normals>& ncurr,
                const std::vector<Depth>& dprev, const std::vector<Normals>& nprev);
            virtual bool estimateTransform(cv::Affine3f& affine, const Intr& intr, 
                const std::vector<Points>& vcurr, const std::vector<Normals>& ncurr,
                const std::vector<Points>& vprev, const std::vector<Normals>& nprev);

            //static Vec3f rodrigues2(const Mat3f& matrix);
        private:
            std::vector<int> iters_;
            float angle_thres_;
            float dist_thres_;
            Array2D<float> buffer_;

            struct StreamHelper;
            cv::Ptr<StreamHelper> shelp_;
        };
    }
}
