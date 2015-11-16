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
#include <opencv2/cap_openni2.hpp>

#if defined TBB_INTERFACE_VERSION && TBB_INTERFACE_VERSION < 5000
# undef HAVE_TBB
#endif

#include <queue>

#ifndef i386
#  define i386 0
#endif
#ifndef __arm__
#  define __arm__ 0
#endif
#ifndef _ARC
#  define _ARC 0
#endif
#ifndef __APPLE__
#  define __APPLE__ 0
#endif

#define CV_STREAM_TIMEOUT 2000

#define CV_DEPTH_STREAM 0
#define CV_COLOR_STREAM 1

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "OpenNI.h"
#include "PS1080.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CvCapture_OpenNI2::isOpened() const
{
    return isContextOpened;
}

openni::VideoMode CvCapture_OpenNI2::defaultColorOutputMode()
{
    openni::VideoMode mode;
    mode.setResolution(640, 480);
    mode.setFps(30);
    mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
    return mode;
}

openni::VideoMode CvCapture_OpenNI2::defaultDepthOutputMode()
{
    openni::VideoMode mode;
    mode.setResolution(640, 480);
    mode.setFps(30);
    mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    return mode;
}

CvCapture_OpenNI2::CvCapture_OpenNI2( int index )
{
    numStream = 2;
    const char* deviceURI = openni::ANY_DEVICE;
    openni::Status status;
    int deviceType = DEVICE_DEFAULT;

    noSampleValue = shadowValue = 0;

    isContextOpened = false;
    maxBufferSize = DEFAULT_MAX_BUFFER_SIZE;
    isCircleBuffer = DEFAULT_IS_CIRCLE_BUFFER;
    maxTimeDuration = DEFAULT_MAX_TIME_DURATION;

    if( index >= 10 )
    {
        deviceType = index / 10;
        index %= 10;
    }

    // Asus XTION and Occipital Structure Sensor do not have an image generator
    if (deviceType == DEVICE_ASUS_XTION)
        numStream = 1;

    if( deviceType > DEVICE_MAX )
        return;

    // Initialize and configure the context.
    status = openni::OpenNI::initialize();

    if (status != openni::STATUS_OK)
    {
        CV_Error(CV_StsError, cv::format("Failed to initialize:", openni::OpenNI::getExtendedError()));
        return;
    }

    status = device.open(deviceURI);
    if( status != openni::STATUS_OK )
    {
        CV_Error(CV_StsError, cv::format("OpenCVKinect: Device open failed see: %s\n", openni::OpenNI::getExtendedError()));
        openni::OpenNI::shutdown();
        return;
    }

    //device.setDepthColorSyncEnabled(true);


    status = depth.create(device, openni::SENSOR_DEPTH);
    if (status == openni::STATUS_OK)
    {
        if (depth.isValid())
        {
            CV_Assert(depth.setVideoMode(defaultDepthOutputMode()) == openni::STATUS_OK); // xn::DepthGenerator supports VGA only! (Jan 2011)
        }

        status = depth.start();
        if (status != openni::STATUS_OK)
        {
            CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::CvCapture_OpenNI2 : Couldn't start depth stream: %s\n", openni::OpenNI::getExtendedError()));
            depth.destroy();
            return;
        }
    }
    else
    {
        CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::CvCapture_OpenNI2 : Couldn't find depth stream:: %s\n", openni::OpenNI::getExtendedError()));
        return;
    }

    streams = new openni::VideoStream*[numStream];
    streams[CV_DEPTH_STREAM] = &depth;

    // create a color object
    status = color.create(device, openni::SENSOR_COLOR);
    if (status == openni::STATUS_OK)
    {
        // Set map output mode.
        if (color.isValid())
        {
            CV_Assert(color.setVideoMode(defaultColorOutputMode()) == openni::STATUS_OK);
        }
        status = color.start();
        if (status != openni::STATUS_OK)
        {
            CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::CvCapture_OpenNI2 : Couldn't start color stream: %s\n", openni::OpenNI::getExtendedError()));
            color.destroy();
            return;
        }
        streams[CV_COLOR_STREAM] = &color;
    }
    else if (numStream == 2)
    {
        CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::CvCapture_OpenNI2 : Couldn't find color stream: %s\n", openni::OpenNI::getExtendedError()));
        return;
    }

    if( !readCamerasParams() )
    {
        CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::CvCapture_OpenNI2 : Could not read cameras parameters\n"));
        return;
    }

//    if( deviceType == DEVICE_ASUS_XTION )
//    {
//        //ps/asus specific
//        imageGenerator.SetIntProperty("InputFormat", 1 /*XN_IO_IMAGE_FORMAT_YUV422*/);
//        imageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
//        depthGenerator.SetIntProperty("RegistrationType", 1 /*XN_PROCESSING_HARDWARE*/);
//    }


    outputMaps.resize( outputMapsTypesCount );

    isContextOpened = true;

    setProperty(cv::CAP_PROP_OPENNI_REGISTRATION, 1.0);
}

CvCapture_OpenNI2::CvCapture_OpenNI2(const char * filename)
{
    numStream = 2;
    openni::Status status;

    isContextOpened = false;
    maxBufferSize = DEFAULT_MAX_BUFFER_SIZE;
    isCircleBuffer = DEFAULT_IS_CIRCLE_BUFFER;
    maxTimeDuration = DEFAULT_MAX_TIME_DURATION;

    // Initialize and configure the context.
    status = openni::OpenNI::initialize();

    if (status != openni::STATUS_OK)
    {
        CV_Error(CV_StsError, cv::format("Failed to initialize:", openni::OpenNI::getExtendedError()));
        return;
    }

    // Open file
    status = device.open(filename);
    if( status != openni::STATUS_OK )
    {
        CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::CvCapture_OpenNI2 : Failed to open input file (%s): %s\n", filename, openni::OpenNI::getExtendedError()));
        return;
    }

    if( !readCamerasParams() )
    {
        CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::CvCapture_OpenNI2 : Could not read cameras parameters\n"));
        return;
    }

    outputMaps.resize( outputMapsTypesCount );

    isContextOpened = true;
}

CvCapture_OpenNI2::~CvCapture_OpenNI2()
{
    this->depthFrame.release();
    this->colorFrame.release();
    this->depth.stop();
    this->color.stop();
    openni::OpenNI::shutdown();
}

bool CvCapture_OpenNI2::readCamerasParams()
{
    double pixelSize = 0;
    if (depth.getProperty<double>(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE, &pixelSize) != openni::STATUS_OK)
    {
        CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::readCamerasParams : Could not read pixel size!\n"));
        return false;
    }

    // pixel size @ VGA = pixel size @ SXGA x 2
    pixelSize *= 2.0; // in mm

    // focal length of IR camera in pixels for VGA resolution
    int zeroPlanDistance; // in mm
    if (depth.getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE, &zeroPlanDistance) != openni::STATUS_OK)
    {
        CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::readCamerasParams : Could not read virtual plane distance!\n"));
        return false;
    }

    if (depth.getProperty<double>(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE, &baseline) != openni::STATUS_OK)
    {
        CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::readCamerasParams : Could not read base line!\n"));
        return false;
    }

    // baseline from cm -> mm
    baseline *= 10;

    // focal length from mm -> pixels (valid for 640x480)
    depthFocalLength_VGA = (int)((double)zeroPlanDistance / (double)pixelSize);

    return true;
}

double CvCapture_OpenNI2::getProperty( int propIdx ) const
{
    double propValue = 0;

    if( isOpened() )
    {
        int purePropIdx = propIdx & ~cv::CAP_OPENNI_GENERATORS_MASK;

        if( (propIdx & cv::CAP_OPENNI_GENERATORS_MASK) == cv::CAP_OPENNI_IMAGE_GENERATOR )
        {
            propValue = getImageGeneratorProperty( purePropIdx );
        }
        else if( (propIdx & cv::CAP_OPENNI_GENERATORS_MASK) == cv::CAP_OPENNI_DEPTH_GENERATOR )
        {
            propValue = getDepthGeneratorProperty( purePropIdx );
        }
        else
        {
            propValue = getCommonProperty( purePropIdx );
        }
    }

    return propValue;
}

bool CvCapture_OpenNI2::setProperty( int propIdx, double propValue )
{
    bool isSet = false;
    if( isOpened() )
    {
        int purePropIdx = propIdx & ~cv::CAP_OPENNI_GENERATORS_MASK;

        if( (propIdx & cv::CAP_OPENNI_GENERATORS_MASK) == cv::CAP_OPENNI_IMAGE_GENERATOR )
        {
            isSet = setImageGeneratorProperty( purePropIdx, propValue );
        }
        else if( (propIdx & cv::CAP_OPENNI_GENERATORS_MASK) == cv::CAP_OPENNI_DEPTH_GENERATOR )
        {
            isSet = setDepthGeneratorProperty( purePropIdx, propValue );
        }
        else
        {
            isSet = setCommonProperty( purePropIdx, propValue );
        }
    }

    return isSet;
}

double CvCapture_OpenNI2::getCommonProperty( int propIdx ) const
{
    double propValue = 0;

    switch( propIdx )
    {
    // There is a set of properties that correspond to depth generator by default
    // (is they are pass without particular generator flag). Two reasons of this:
    // 1) We can assume that depth generator is the main one for depth sensor.
    // 2) In the initial vertions of OpenNI integration to OpenCV the value of
    //    flag cv::CAP_OPENNI_DEPTH_GENERATOR was 0 (it isn't zero now).
    case cv::CAP_PROP_OPENNI_GENERATOR_PRESENT :
    case cv::CAP_PROP_FRAME_WIDTH :
    case cv::CAP_PROP_FRAME_HEIGHT :
    case cv::CAP_PROP_FPS :
    case cv::CAP_PROP_OPENNI_FRAME_MAX_DEPTH :
    case cv::CAP_PROP_OPENNI_BASELINE :
    case cv::CAP_PROP_OPENNI_FOCAL_LENGTH :
    case cv::CAP_PROP_OPENNI_REGISTRATION :
        propValue = getDepthGeneratorProperty( propIdx );
        break;
    case cv::CAP_PROP_OPENNI2_SYNC :
        propValue = const_cast<CvCapture_OpenNI2 *>(this)->device.getDepthColorSyncEnabled();
    case cv::CAP_PROP_OPENNI2_MIRROR:
    {
        bool isMirroring = color.getMirroringEnabled() && depth.getMirroringEnabled();
        propValue = isMirroring ? 1.0 : 0.0;
        break;
    }
    default :
        CV_Error( CV_StsBadArg, cv::format("Such parameter (propIdx=%d) isn't supported for getting.\n", propIdx) );
    }

    return propValue;
}

bool CvCapture_OpenNI2::setCommonProperty( int propIdx, double propValue )
{
    bool isSet = false;

    switch( propIdx )
    {
    case cv::CAP_PROP_OPENNI2_MIRROR:
    {
        bool mirror = propValue > 0.0 ? true : false;
        isSet = color.setMirroringEnabled(mirror) == openni::STATUS_OK;
        isSet = depth.setMirroringEnabled(mirror) == openni::STATUS_OK;
    }
        break;
    // There is a set of properties that correspond to depth generator by default
    // (is they are pass without particular generator flag).
    case cv::CAP_PROP_OPENNI_REGISTRATION:
        isSet = setDepthGeneratorProperty( propIdx, propValue );
        break;
    case cv::CAP_PROP_OPENNI2_SYNC:
        isSet = device.setDepthColorSyncEnabled(propValue > 0.0) == openni::STATUS_OK;
        break;
    default:
        CV_Error( CV_StsBadArg, cv::format("Such parameter (propIdx=%d) isn't supported for setting.\n", propIdx) );
    }

    return isSet;
}

double CvCapture_OpenNI2::getDepthGeneratorProperty( int propIdx ) const
{
    double propValue = 0;
    if( !depth.isValid() )
        return propValue;

    openni::VideoMode mode;

    switch( propIdx )
    {
    case cv::CAP_PROP_OPENNI_GENERATOR_PRESENT :
        CV_DbgAssert(depth.isValid());
        propValue = 1.;
        break;
    case cv::CAP_PROP_FRAME_WIDTH :
        propValue = depth.getVideoMode().getResolutionX();
        break;
    case cv::CAP_PROP_FRAME_HEIGHT :
            propValue = depth.getVideoMode().getResolutionY();
        break;
    case cv::CAP_PROP_FPS :
        mode = depth.getVideoMode();
        propValue = mode.getFps();
        break;
    case cv::CAP_PROP_OPENNI_FRAME_MAX_DEPTH :
        propValue = depth.getMaxPixelValue();
        break;
    case cv::CAP_PROP_OPENNI_BASELINE :
        propValue = baseline;
        break;
    case cv::CAP_PROP_OPENNI_FOCAL_LENGTH :
        propValue = (double)depthFocalLength_VGA;
        break;
    case cv::CAP_PROP_OPENNI_REGISTRATION :
        propValue = device.getImageRegistrationMode();
        break;
    case cv::CAP_PROP_POS_MSEC :
        propValue = (double)depthFrame.getTimestamp();
        break;
    case cv::CAP_PROP_POS_FRAMES :
        propValue = depthFrame.getFrameIndex();
        break;
    default :
        CV_Error( CV_StsBadArg, cv::format("Depth generator does not support such parameter (propIdx=%d) for getting.\n", propIdx) );
    }

    return propValue;
}

bool CvCapture_OpenNI2::setDepthGeneratorProperty( int propIdx, double propValue )
{
    bool isSet = false;

    CV_Assert( depth.isValid() );

    switch( propIdx )
    {
    case cv::CAP_PROP_OPENNI_REGISTRATION:
        {
            if( propValue < 1.0 ) // "on"
            {
                // if there isn't image generator (i.e. ASUS XtionPro doesn't have it)
                // then the property isn't avaliable
                if ( color.isValid() )
                {
                    openni::ImageRegistrationMode mode = propValue < 1.0 ? openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR : openni::IMAGE_REGISTRATION_OFF;
                    if( !device.getImageRegistrationMode() == mode )
                    {
                        if (device.isImageRegistrationModeSupported(mode))
                        {
                            openni::Status status = device.setImageRegistrationMode(mode);
                            if( status != openni::STATUS_OK )
                                CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::setDepthGeneratorProperty : %s\n", openni::OpenNI::getExtendedError()));
                            else
                                isSet = true;
                        }
                        else
                            CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::setDepthGeneratorProperty : Unsupported viewpoint.\n"));
                    }
                    else
                        isSet = true;
                }
            }
            else // "off"
            {
                openni::Status status = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
                if( status != openni::STATUS_OK )
                    CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::setDepthGeneratorProperty : %s\n", openni::OpenNI::getExtendedError()));
                else
                    isSet = true;
            }
        }
        break;
    default:
        CV_Error( CV_StsBadArg, cv::format("Depth generator does not support such parameter (propIdx=%d) for setting.\n", propIdx) );
    }

    return isSet;
}

double CvCapture_OpenNI2::getImageGeneratorProperty( int propIdx ) const
{
    double propValue = 0.;
    if( !color.isValid() )
        return propValue;

    openni::VideoMode mode;
    switch( propIdx )
    {
    case cv::CAP_PROP_OPENNI_GENERATOR_PRESENT :
        CV_DbgAssert( color.isValid() );
        propValue = 1.;
        break;
    case cv::CAP_PROP_FRAME_WIDTH :
            propValue = color.getVideoMode().getResolutionX();
        break;
    case cv::CAP_PROP_FRAME_HEIGHT :
            propValue = color.getVideoMode().getResolutionY();
        break;
    case cv::CAP_PROP_FPS :
            propValue = color.getVideoMode().getFps();
        break;
    case cv::CAP_PROP_POS_MSEC :
        propValue = (double)colorFrame.getTimestamp();
        break;
    case cv::CAP_PROP_POS_FRAMES :
        propValue = (double)colorFrame.getFrameIndex();
        break;
    default :
        CV_Error( CV_StsBadArg, cv::format("Image generator does not support such parameter (propIdx=%d) for getting.\n", propIdx) );
    }

    return propValue;
}

bool CvCapture_OpenNI2::setImageGeneratorProperty(int propIdx, double propValue)
{
    bool isSet = false;
        if( !color.isValid() )
            return isSet;

        switch( propIdx )
        {
        case cv::CAP_PROP_OPENNI_OUTPUT_MODE :
        {
            openni::VideoMode mode = color.getVideoMode();

            switch( cvRound(propValue) )
            {
            case cv::CAP_OPENNI_VGA_30HZ :
                mode.setResolution(640,480);
                mode.setFps(30);
                break;
            case cv::CAP_OPENNI_SXGA_15HZ :
                mode.setResolution(1280, 960);
                mode.setFps(15);
                break;
            case cv::CAP_OPENNI_SXGA_30HZ :
                mode.setResolution(1280, 960);
                mode.setFps(30);
                break;
            case cv::CAP_OPENNI_QVGA_30HZ :
                mode.setResolution(320, 240);
                mode.setFps(30);
                 break;
            case cv::CAP_OPENNI_QVGA_60HZ :
                mode.setResolution(320, 240);
                mode.setFps(60);
                 break;
            default :
                CV_Error( CV_StsBadArg, "Unsupported image generator output mode.\n");
            }

            openni::Status status = color.setVideoMode( mode );
            if( status != openni::STATUS_OK )
                CV_Error(CV_StsError, cv::format("CvCapture_OpenNI2::setImageGeneratorProperty : %s\n", openni::OpenNI::getExtendedError()));
            else
                isSet = true;
            break;
        }
        default:
            CV_Error( CV_StsBadArg, cv::format("Image generator does not support such parameter (propIdx=%d) for setting.\n", propIdx) );
        }

    return isSet;
}

bool CvCapture_OpenNI2::grabFrame()
{
    if( !isOpened() )
        return false;

    bool isGrabbed = false;

    openni::Status status = openni::OpenNI::waitForAnyStream(streams, numStream, &currentStream, CV_STREAM_TIMEOUT);
    if( status != openni::STATUS_OK )
        return false;

    if( depth.isValid() )
        depth.readFrame(&depthFrame);
    if (color.isValid())
        color.readFrame(&colorFrame);
    isGrabbed = true;

    return isGrabbed;
}

inline void getDepthMapFromMetaData(const openni::VideoFrameRef& depthMetaData, cv::Mat& depthMap, int noSampleValue, int shadowValue)
{
    depthMap.create(depthMetaData.getHeight(), depthMetaData.getWidth(), CV_16UC1);
    depthMap.data = (uchar*)depthMetaData.getData();

    cv::Mat badMask = (depthMap == (double)noSampleValue) | (depthMap == (double)shadowValue) | (depthMap == 0);

    // mask the pixels with invalid depth
    depthMap.setTo( cv::Scalar::all( CvCapture_OpenNI2::INVALID_PIXEL_VAL ), badMask );
}

cv::Mat CvCapture_OpenNI2::retrieveDepthMap()
{
    if( !depth.isValid() )
        return cv::Mat();

    getDepthMapFromMetaData( depthFrame, outputMaps[cv::CAP_OPENNI_DEPTH_MAP], noSampleValue, shadowValue );

    return outputMaps[cv::CAP_OPENNI_DEPTH_MAP];
}

cv::Mat CvCapture_OpenNI2::retrievePointCloudMap()
{
    if( !depthFrame.isValid() )
        return cv::Mat();

    cv::Mat depthImg;
    getDepthMapFromMetaData(depthFrame, depthImg, noSampleValue, shadowValue);

    const int badPoint = INVALID_PIXEL_VAL;
    const float badCoord = INVALID_COORDINATE_VAL;
    int cols = depthFrame.getWidth(), rows = depthFrame.getHeight();
    cv::Mat pointCloud_XYZ( rows, cols, CV_32FC3, cv::Scalar::all(badPoint) );

    float worldX, worldY, worldZ;
    for( int y = 0; y < rows; y++ )
    {
        for (int x = 0; x < cols; x++)
        {
            openni::CoordinateConverter::convertDepthToWorld(depth, x, y, depthImg.at<unsigned short>(y, x), &worldX, &worldY, &worldZ);

            if (depthImg.at<unsigned short>(y, x) == badPoint) // not valid
                pointCloud_XYZ.at<cv::Point3f>(y, x) = cv::Point3f(badCoord, badCoord, badCoord);
            else
            {
                pointCloud_XYZ.at<cv::Point3f>(y, x) = cv::Point3f(worldX*0.001f, worldY*0.001f, worldZ*0.001f); // from mm to meters
            }
        }
    }

    outputMaps[cv::CAP_OPENNI_POINT_CLOUD_MAP] = pointCloud_XYZ;

    return outputMaps[cv::CAP_OPENNI_POINT_CLOUD_MAP];
}

static void computeDisparity_32F( const openni::VideoFrameRef& depthMetaData, cv::Mat& disp, double baseline, int F, int noSampleValue, int shadowValue)
{
    cv::Mat depth;
    getDepthMapFromMetaData( depthMetaData, depth, noSampleValue, shadowValue );
    CV_Assert( depth.type() == CV_16UC1 );

    // disparity = baseline * F / z;

    float mult = (float)(baseline /*mm*/ * F /*pixels*/);

    disp.create( depth.size(), CV_32FC1);
    disp = cv::Scalar::all( CvCapture_OpenNI2::INVALID_PIXEL_VAL );
    for( int y = 0; y < disp.rows; y++ )
    {
        for( int x = 0; x < disp.cols; x++ )
        {
            unsigned short curDepth = depth.at<unsigned short>(y,x);
            if( curDepth != CvCapture_OpenNI2::INVALID_PIXEL_VAL )
                disp.at<float>(y,x) = mult / curDepth;
        }
    }
}

cv::Mat CvCapture_OpenNI2::retrieveDisparityMap()
{
    if (!depthFrame.isValid())
        return cv::Mat();

    cv::Mat disp32;
    computeDisparity_32F(depthFrame, disp32, baseline, depthFocalLength_VGA, noSampleValue, shadowValue);

    disp32.convertTo( outputMaps[cv::CAP_OPENNI_DISPARITY_MAP], CV_8UC1 );

    return outputMaps[cv::CAP_OPENNI_DISPARITY_MAP];
}

cv::Mat CvCapture_OpenNI2::retrieveDisparityMap_32F()
{
    if (!depthFrame.isValid())
        return cv::Mat();

    computeDisparity_32F(depthFrame, outputMaps[cv::CAP_OPENNI_DISPARITY_MAP_32F], baseline, depthFocalLength_VGA, noSampleValue, shadowValue);

    return outputMaps[cv::CAP_OPENNI_DISPARITY_MAP_32F];
}

cv::Mat CvCapture_OpenNI2::retrieveValidDepthMask()
{
    if (!depthFrame.isValid())
        return cv::Mat();

    cv::Mat d;
    getDepthMapFromMetaData(depthFrame, d, noSampleValue, shadowValue);

    outputMaps[cv::CAP_OPENNI_VALID_DEPTH_MASK] = d != CvCapture_OpenNI2::INVALID_PIXEL_VAL;

    return outputMaps[cv::CAP_OPENNI_VALID_DEPTH_MASK];
}

inline void getBGRImageFromMetaData( const openni::VideoFrameRef& imageMetaData, cv::Mat& bgrImage )
{
   cv::Mat bufferImage;
   if( imageMetaData.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_RGB888 )
        CV_Error( CV_StsUnsupportedFormat, "Unsupported format of grabbed image\n" );

   bgrImage.create(imageMetaData.getHeight(), imageMetaData.getWidth(), CV_8UC3);
   bufferImage.create(imageMetaData.getHeight(), imageMetaData.getWidth(), CV_8UC3);
   bufferImage.data = (uchar*)imageMetaData.getData();

   cv::cvtColor(bufferImage, bgrImage, cv::COLOR_RGB2BGR);
}

cv::Mat CvCapture_OpenNI2::retrieveBGRImage()
{
    if( !color.isValid() )
        return cv::Mat();

    getBGRImageFromMetaData( colorFrame, outputMaps[cv::CAP_OPENNI_BGR_IMAGE] );

    return outputMaps[cv::CAP_OPENNI_BGR_IMAGE];
}

cv::Mat CvCapture_OpenNI2::retrieveGrayImage()
{
    if (!colorFrame.isValid())
        return cv::Mat();

    CV_Assert(colorFrame.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_RGB888); // RGB

    cv::Mat rgbImage;
    getBGRImageFromMetaData(colorFrame, rgbImage);
    cv::cvtColor( rgbImage, outputMaps[cv::CAP_OPENNI_GRAY_IMAGE], CV_BGR2GRAY );

    return outputMaps[cv::CAP_OPENNI_GRAY_IMAGE];
}

cv::Mat CvCapture_OpenNI2::retrieveFrame( int outputType )
{
    cv::Mat image;
    CV_Assert( outputType < outputMapsTypesCount && outputType >= 0);

    if( outputType == cv::CAP_OPENNI_DEPTH_MAP )
    {
        image = retrieveDepthMap();
    }
    else if( outputType == cv::CAP_OPENNI_POINT_CLOUD_MAP )
    {
        image = retrievePointCloudMap();
    }
    else if( outputType == cv::CAP_OPENNI_DISPARITY_MAP )
    {
        image = retrieveDisparityMap();
    }
    else if( outputType == cv::CAP_OPENNI_DISPARITY_MAP_32F )
    {
        image = retrieveDisparityMap_32F();
    }
    else if( outputType == cv::CAP_OPENNI_VALID_DEPTH_MASK )
    {
        image = retrieveValidDepthMask();
    }
    else if( outputType == cv::CAP_OPENNI_BGR_IMAGE )
    {
        image = retrieveBGRImage();
    }
    else if( outputType == cv::CAP_OPENNI_GRAY_IMAGE )
    {
        image = retrieveGrayImage();
    }

    return image;
}
