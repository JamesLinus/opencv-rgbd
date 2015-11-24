#pragma once

#include <kfusion/cuda/device_array.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/affine.hpp>
//#include <opencv2/viz/vizcore.hpp>
#include <iosfwd>

struct CUevent_st;

namespace kf
{
    struct KF_EXPORTS Intr
    {
        float fx, fy, cx, cy;

        Intr ();
        Intr (float fx, float fy, float cx, float cy);
        Intr operator()(int level_index) const;
    };

    KF_EXPORTS std::ostream& operator << (std::ostream& os, const Intr& intr);

    struct Point
    {
        union
        {
            float data[4];
            struct { float x, y, z, w; };
        };
    };

    struct Normal
    {
        union
        {
            float data[4];
            struct { float x, y, z; };
        };
    };

    struct RGB
    {
        union
        {
            struct { unsigned char b, g, r; };
            int bgra;
        };
    };

    struct PixelRGB
    {
        unsigned char r, g, b;
    };

    namespace cuda
    {
        typedef cuda::Array2D<ushort> Depth;
        typedef cuda::Array2D<ushort> Dists;
        typedef cuda::Array2D<RGB> Image;
        typedef cuda::Array2D<Normal> Normals;
        typedef cuda::Array2D<Point> Points;

        struct Frame
        {
            bool use_points;

            std::vector<Depth> depth_pyr;
            std::vector<Points> points_pyr;
            std::vector<Normals> normals_pyr;
        };
    }

    inline float deg2rad (float alpha) { return alpha * 0.017453293f; }

    struct KF_EXPORTS ScopeTime
    {
        const char* name;
        double start;
        ScopeTime(const char *name);
        ~ScopeTime();
    };

    struct KF_EXPORTS SampledScopeTime
    {
    public:
        enum { EACH = 33 };
        SampledScopeTime(double& time_ms);
        ~SampledScopeTime();
    private:
        double getTime();
        SampledScopeTime(const SampledScopeTime&);
        SampledScopeTime& operator=(const SampledScopeTime&);

        double& time_ms_;
        double start;
    };

}
