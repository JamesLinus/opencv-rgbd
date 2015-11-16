/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
#pragma once
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"

#include "OpenNI.h"
#include "PS1080.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class CvCapture_OpenNI2
{
public:
    enum { DEVICE_DEFAULT=0, DEVICE_MS_KINECT=0, DEVICE_ASUS_XTION=1, DEVICE_MAX=1 };

    static const int INVALID_PIXEL_VAL = 0;
    static const int INVALID_COORDINATE_VAL = 0;

#ifdef HAVE_TBB
    static const int DEFAULT_MAX_BUFFER_SIZE = 8;
#else
    static const int DEFAULT_MAX_BUFFER_SIZE = 2;
#endif
    static const int DEFAULT_IS_CIRCLE_BUFFER = 0;
    static const int DEFAULT_MAX_TIME_DURATION = 20;

    CvCapture_OpenNI2(int index = 0);
    CvCapture_OpenNI2(const char * filename);
    ~CvCapture_OpenNI2();

    double getProperty(int propIdx) const;
    bool setProperty(int probIdx, double propVal);
    bool grabFrame();
    cv::Mat retrieveFrame(int outputType);

    bool isOpened() const;

protected:

    static const int outputMapsTypesCount = 7;

    static openni::VideoMode defaultColorOutputMode();
    static openni::VideoMode defaultDepthOutputMode();

    cv::Mat retrieveDepthMap();
    cv::Mat retrievePointCloudMap();
    cv::Mat retrieveDisparityMap();
    cv::Mat retrieveDisparityMap_32F();
    cv::Mat retrieveValidDepthMask();
    cv::Mat retrieveBGRImage();
    cv::Mat retrieveGrayImage();

    bool readCamerasParams();

    double getDepthGeneratorProperty(int propIdx) const;
    bool setDepthGeneratorProperty(int propIdx, double propVal);
    double getImageGeneratorProperty(int propIdx) const;
    bool setImageGeneratorProperty(int propIdx, double propVal);
    double getCommonProperty(int propIdx) const;
    bool setCommonProperty(int propIdx, double propVal);

    // OpenNI context
    openni::Device device;
    bool isContextOpened;
    openni::Recorder recorder;

    // Data generators with its metadata
    openni::VideoStream depth, color, **streams;
    openni::VideoFrameRef depthFrame, colorFrame;
    cv::Mat depthImage, colorImage;

    int maxBufferSize, maxTimeDuration; // for approx sync
    bool isCircleBuffer;
    //cv::Ptr<ApproximateSyncGrabber> approxSyncGrabber;

    // Cameras settings:
    // TODO find in OpenNI function to convert z->disparity and remove fields "baseline" and depthFocalLength_VGA
    // Distance between IR projector and IR camera (in meters)
    double baseline;
    // Focal length for the IR camera in VGA resolution (in pixels)
    int depthFocalLength_VGA;

    // The value for shadow (occluded pixels)
    int shadowValue;
    // The value for pixels without a valid disparity measurement
    int noSampleValue;

    int currentStream;

    int numStream;
    std::vector<cv::Mat> outputMaps;
};
