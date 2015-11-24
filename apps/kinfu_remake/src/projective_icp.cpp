#include "precomp.hpp"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ComputeIcpHelper

kf::impl::ComputeIcpHelper::ComputeIcpHelper(float dist_thres, float angle_thres)
{
    min_cosine = cos(angle_thres);
    dist2_thres = dist_thres * dist_thres;
}

void kf::impl::ComputeIcpHelper::setLevelIntr(int level_index, float fx, float fy, float cx, float cy)
{
    int div = 1 << level_index;
    f = make_float2(fx/div, fy/div);
    c = make_float2(cx/div, cy/div);
    finv = make_float2(1.f/f.x, 1.f/f.y);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ProjectiveICP::StreamHelper

struct kf::cuda::ProjectiveICP::StreamHelper
{
    typedef impl::ComputeIcpHelper::PageLockHelper PageLockHelper;
    typedef cv::Matx66f Mat6f;
    typedef cv::Vec6f Vec6f;

    cudaStream_t stream;
    PageLockHelper locked_buffer;

    StreamHelper() { cudaSafeCall( cudaStreamCreate(&stream) ); }
    ~StreamHelper() { cudaSafeCall( cudaStreamDestroy(stream) ); }

    operator float*() { return locked_buffer.data; }
    operator cudaStream_t() { return stream; }

    Mat6f get(Vec6f& b)
    {
        cudaSafeCall( cudaStreamSynchronize(stream) );

        Mat6f A;
        float *data_A = A.val;
        float *data_b = b.val;

        int shift = 0;
        for (int i = 0; i < 6; ++i)   //rows
            for (int j = i; j < 7; ++j) // cols + b
            {
                float value = locked_buffer.data[shift++];
                if (j == 6)               // vector b
                    data_b[i] = value;
                else
                    data_A[j * 6 + i] = data_A[i * 6 + j] = value;
            }
        return A;
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ProjectiveICP

kf::cuda::ProjectiveICP::ProjectiveICP() : angle_thres_(deg2rad(20.f)), dist_thres_(0.1f)
{ 
    const int iters[] = {10, 5, 4, 0};
    std::vector<int> vector_iters(iters, iters + 4);
    setIterationsNum(vector_iters);
    impl::ComputeIcpHelper::allocate_buffer(buffer_);

    shelp_ = cv::makePtr<StreamHelper>();
}

kf::cuda::ProjectiveICP::~ProjectiveICP() {}

float kf::cuda::ProjectiveICP::getDistThreshold() const
{ return dist_thres_; }

void kf::cuda::ProjectiveICP::setDistThreshold(float distance)
{ dist_thres_ = distance; }

float kf::cuda::ProjectiveICP::getAngleThreshold() const
{ return angle_thres_; }

void kf::cuda::ProjectiveICP::setAngleThreshold(float angle)
{ angle_thres_ = angle; }

void kf::cuda::ProjectiveICP::setIterationsNum(const std::vector<int>& iters)
{
    if (iters.size() >= MAX_PYRAMID_LEVELS)
        iters_.assign(iters.begin(), iters.begin() + MAX_PYRAMID_LEVELS);
    else
    {
        iters_ = vector<int>(MAX_PYRAMID_LEVELS, 0);
        copy(iters.begin(), iters.end(),iters_.begin());
    }
}

int kf::cuda::ProjectiveICP::getUsedLevelsNum() const
{
    int i = MAX_PYRAMID_LEVELS - 1;
    for(; i >= 0 && !iters_[i]; --i);
    return i + 1;
}

bool kf::cuda::ProjectiveICP::estimateTransform(cv::Affine3f& /*affine*/, const Intr& /*intr*/, const Frame& /*curr*/, const Frame& /*prev*/)
{
//    bool has_depth = !curr.depth_pyr.empty() && !prev.depth_pyr.empty();
//    bool has_points = !curr.points_pyr.empty() && !prev.points_pyr.empty();

//    if (has_depth)
//        return estimateTransform(affine, intr, curr.depth_pyr, curr.normals_pyr, prev.depth_pyr, prev.normals_pyr);
//    else if(has_points)
//         return estimateTransform(affine, intr, curr.points_pyr, curr.normals_pyr, prev.points_pyr, prev.normals_pyr);
//    else
//        CV_Assert(!"Wrong parameters passed to estimateTransform");
    CV_Assert(!"Not implemented");
    return false;
}


bool kf::cuda::ProjectiveICP::estimateTransform(cv::Affine3f& affine, const Intr& intr, 
    const std::vector<Depth>& dcurr, const std::vector<Normals>& ncurr,
    const std::vector<Depth>& dprev, const std::vector<Normals>& nprev)
{
    const int LEVELS = getUsedLevelsNum();
    StreamHelper& sh = *shelp_;

    impl::ComputeIcpHelper helper(dist_thres_, angle_thres_);
    affine = cv::Affine3f::Identity();

    for(int level_index = LEVELS - 1; level_index >= 0; --level_index)
    {
        const impl::Normals& n = (const impl::Normals& )nprev[level_index];

        helper.rows = (float)n.rows();
        helper.cols = (float)n.cols();
        helper.setLevelIntr(level_index, intr.fx, intr.fy, intr.cx, intr.cy);
        helper.dcurr = dcurr[level_index];
        helper.ncurr = ncurr[level_index];

        for(int iter = 0; iter < iters_[level_index]; ++iter)
        {
            helper.aff = device_cast<impl::Aff3f>(affine);
            helper(dprev[level_index], n, buffer_, sh, sh);

            StreamHelper::Vec6f b;
            StreamHelper::Mat6f A  = sh.get(b);

            //checking nullspace
            double det = cv::determinant(A);

            if (fabs (det) < 1e-15 || cvIsNaN(det))
            {
                if (cvIsNaN(det)) cout << "qnan" << endl;
                return false;
            }

            StreamHelper::Vec6f r;
            cv::solve(A, b, r, cv::DECOMP_SVD);
            cv::Affine3f Tinc(cv::Vec3f(r.val), cv::Vec3f(r.val+3));
            affine = Tinc * affine;
        }
    }
    return true;
}

bool kf::cuda::ProjectiveICP::estimateTransform(cv::Affine3f& affine, const Intr& intr, 
    const std::vector<Points>& vcurr, const std::vector<Normals>& ncurr,
    const std::vector<Points>& vprev, const std::vector<Normals>& nprev)
{
    const int LEVELS = getUsedLevelsNum();
    StreamHelper& sh = *shelp_;

    impl::ComputeIcpHelper helper(dist_thres_, angle_thres_);
    affine = cv::Affine3f::Identity();

    for(int level_index = LEVELS - 1; level_index >= 0; --level_index)
    {
        const impl::Normals& n = (const impl::Normals& )nprev[level_index];
        const impl::Points& v = (const impl::Points& )vprev[level_index];

        helper.rows = (float)n.rows();
        helper.cols = (float)n.cols();
        helper.setLevelIntr(level_index, intr.fx, intr.fy, intr.cx, intr.cy);
        helper.vcurr = vcurr[level_index];
        helper.ncurr = ncurr[level_index];

        for(int iter = 0; iter < iters_[level_index]; ++iter)
        {
            helper.aff = device_cast<impl::Aff3f>(affine);
            helper(v, n, buffer_, sh, sh);

            StreamHelper::Vec6f b;
            StreamHelper::Mat6f A = sh.get(b);

            //checking nullspace
            double det = cv::determinant(A);

            if (fabs (det) < 1e-15 || cvIsNaN (det))
            {
                if (cvIsNaN (det)) cout << "qnan" << endl;
                return false;
            }

            StreamHelper::Vec6f r;
            cv::solve(A, b, r, cv::DECOMP_SVD);

            cv::Affine3f Tinc(cv::Vec3f(r.val), cv::Vec3f(r.val+3));
            affine = Tinc * affine;
        }
    }
    return true;
}

