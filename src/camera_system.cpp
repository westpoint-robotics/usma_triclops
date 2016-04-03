#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "typedefs.h"
#include "camera_system.h"

CameraSystem::CameraSystem( int argc, char** argv )
{
    disp_min = 0;
    disp_max = 70;
    disp_map_max = 255;
    disp_map_min = 0;
    disp_map_on = 0;
    stereo_mask = 11;

    // Create control sliders that allow tunning of the parameters for line detection
    cv::namedWindow( "DisparityView", CV_WINDOW_AUTOSIZE );
    cv::createTrackbar( "disp_max", "DisparityView", &disp_max, 240 );
    cv::createTrackbar( "disp_min", "DisparityView", &disp_min, 239 );
    cv::createTrackbar( "disp_map_max", "DisparityView", &disp_map_max, 255 );
    cv::createTrackbar( "disp_map_min", "DisparityView", &disp_map_min, 254 );
    cv::createTrackbar( "disp_map_on", "DisparityView", &disp_map_on, 1 );
    cv::createTrackbar( "stereo_mask", "DisparityView", &stereo_mask, 15 );

    FC2::Error fc2Error;

    this->camera.Connect();
    // configure camera - Identifies what camera is being used?

    if ( configureCamera( this->camera ) )
    {
        exit( -1 );
    }

    FC2::Format7ImageSettings formatSettings;
    unsigned int         packetSize;
    float                percent ;

    fc2Error = this->camera.GetFormat7Configuration( &formatSettings, &packetSize, &percent );
    ROS_INFO( "mode,offX,offY,width,height,pixFormat: %d,%d,%d,%d,%d,%x,%f\n", formatSettings.mode, formatSettings.offsetX, formatSettings.offsetY, formatSettings.width, formatSettings.height, formatSettings.pixelFormat, percent );

    // generate the Triclops context   PIXEL_FORMAT_422YUV8
    if ( generateTriclopsContext( this->camera, this->triclops ) )
    {
        exit( -1 );
    }

    // Part 1 of 2 for grabImage method
    fc2Error = this->camera.StartCapture();

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        exit( FC2T::handleFc2Error( fc2Error ) );
    }

    // Get the camera info and print it out
    fc2Error = this->camera.GetCameraInfo( &camInfo );

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        ROS_INFO( "Failed to get camera info from camera\n" );
        exit( -1 );
    }
    else
    {
        ROS_INFO( ">>>>> CAMERA INFO  Vendor: %s     Model: %s     Serail#: %d  Resolution: %s", camInfo.vendorName, camInfo.modelName, camInfo.serialNumber, camInfo.sensorResolution );
    }


    // Container of Images used for processing
    image_transport::ImageTransport it( nh );
    //Publishers for the camera
    this->image_pub_left = it.advertise( "/camera/left/rgb", 1 );
    this->image_pub_right = it.advertise( "/camera/right/rgb", 1 );
    this->image_pub_disparity = it.advertise( "/camera/disparity", 1 );
    this->image_pub_rectifiedColor = it.advertise( "/camera/color_rectified", 1 );
    char fileName[] = "/home/user1/triclopsContextCurrent.txt";

    if ( triclopsWriteCurrentContextToFile( this->triclops, fileName ) )
    {
        exit( -1 );
    }
}

CameraSystem::~CameraSystem()
{
    this->shutdown();
}

int CameraSystem::shutdown()
{
    this->camera.StopCapture();
    this->camera.Disconnect();
    // Destroy the Triclops context
    TriclopsError     te;
    te = triclopsDestroyContext( triclops ) ;
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );
    return 0;
}

//Copied over from older files. RR
int CameraSystem::configureCamera( FC2::Camera & camera )
{
    FC2T::ErrorType fc2TriclopsError;
    FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;
    fc2TriclopsError = FC2T::setStereoMode( camera, mode );

    if ( fc2TriclopsError )
    {
        return FC2T::handleFc2TriclopsError( fc2TriclopsError, "setStereoMode" );
    }

    return 0;
}

//Copied over from older files.RR
int CameraSystem::convertToBGRU( FC2::Image & image, FC2::Image & convertedImage )
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing( FC2::HQ_LINEAR );

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        return FC2T::handleFc2Error( fc2Error );
    }

    fc2Error = image.Convert( FC2::PIXEL_FORMAT_BGRU, &convertedImage );

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        return FC2T::handleFc2Error( fc2Error );
    }

    return 0;
}

//Copied over from older files.
int CameraSystem::convertToBGR( FC2::Image & image, FC2::Image & convertedImage )
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing( FC2::HQ_LINEAR );

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        return FC2T::handleFc2Error( fc2Error );
    }

    fc2Error = image.Convert( FC2::PIXEL_FORMAT_BGR, &convertedImage );

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        return FC2T::handleFc2Error( fc2Error );
    }

    return 0;
}

// Checked against PGR Code OK.RR
int CameraSystem::generateTriclopsContext( FC2::Camera     & camera,
        TriclopsContext & triclops )
{
    FC2::CameraInfo camInfo;
    FC2::Error fc2Error = camera.GetCameraInfo( &camInfo );

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        return FC2T::handleFc2Error( fc2Error );
    }

    FC2T::ErrorType fc2TriclopsError;
    fc2TriclopsError = FC2T::getContextFromCamera( camInfo.serialNumber, &triclops );

    if ( fc2TriclopsError != FC2T::ERRORTYPE_OK )
    {
        return FC2T::handleFc2TriclopsError( fc2TriclopsError,
                                             "getContextFromCamera" );
    }

    return 0;
}

// Checked against PGR Code OK. RR
int CameraSystem::generateTriclopsInput( FC2::Image const & grabbedImage,
        ImageContainer  & imageContainer,
        TriclopsInput   & triclopsColorInput,
        TriclopsInput   & triclopsMonoInput )
{
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError te;

    FC2::Image * unprocessedImage = imageContainer.unprocessed;

    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                           grabbedImage,
                           true /*assume little endian*/,
                           unprocessedImage[RIGHT],
                           unprocessedImage[LEFT] );

    if ( fc2TriclopsError != FC2T::ERRORTYPE_OK )
    {
        return FC2T::handleFc2TriclopsError( fc2TriclopsError,
                                             "unpackUnprocessedRawOrMono16Image" );
    }

    FC2::Image * monoImage = imageContainer.mono;

    //ROS_INFO( "UnrpocessedImage cols,rows %d,%d", unprocessedImage[RIGHT].GetCols(), unprocessedImage[RIGHT].GetRows() );

    // check if the unprocessed image is color
    if ( unprocessedImage[RIGHT].GetBayerTileFormat() != FC2::NONE )
    {
        FC2::Image * bgruImage = imageContainer.bgru;

        for ( int i = 0; i < 2; ++i )
        {
            if ( convertToBGRU( unprocessedImage[i], bgruImage[i] ) )
            {
                return 1;
            }
        }

        FC2::Image & packedColorImage = imageContainer.packed;

        // pack BGRU right and left image into an image
        fc2TriclopsError = FC2T::packTwoSideBySideRgbImage( bgruImage[RIGHT],
                           bgruImage[LEFT],
                           packedColorImage );

        if ( fc2TriclopsError != FC2T::ERRORTYPE_OK )
        {
            return handleFc2TriclopsError( fc2TriclopsError,
                                           "packTwoSideBySideRgbImage" );
        }

        // Use the row interleaved images to build up a packed TriclopsInput.
        // A packed triclops input will contain a single image with 32 bpp.
        te = triclopsBuildPackedTriclopsInput( grabbedImage.GetCols(),
                                               grabbedImage.GetRows(),
                                               packedColorImage.GetStride(),
                                               ( unsigned long )grabbedImage.GetTimeStamp().seconds,
                                               ( unsigned long )grabbedImage.GetTimeStamp().microSeconds,
                                               packedColorImage.GetData(),
                                               &triclopsColorInput );

        _HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );


        // the following does not change the size of the image
        // and therefore it PRESERVES the internal buffer!
        packedColorImage.SetDimensions( packedColorImage.GetRows(),
                                        packedColorImage.GetCols(),
                                        packedColorImage.GetStride(),
                                        packedColorImage.GetPixelFormat(),
                                        FC2::NONE );

        for ( int i = 0; i < 2; ++i )
        {
            fc2Error = bgruImage[i].Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage[i] );

            if ( fc2Error != FlyCapture2::PGRERROR_OK )
            {
                return Fc2Triclops::handleFc2Error( fc2Error );
            }
        }
    }
    else
    {
        monoImage[RIGHT] = unprocessedImage[RIGHT];
        monoImage[LEFT] = unprocessedImage[LEFT];
    }

    // Use the row interleaved images to build up an RGB TriclopsInput.
    // An RGB triclops input will contain the 3 raw images (1 from each camera).
    te = triclopsBuildRGBTriclopsInput( grabbedImage.GetCols(),
                                        grabbedImage.GetRows(),
                                        grabbedImage.GetCols(),
                                        ( unsigned long )grabbedImage.GetTimeStamp().seconds,
                                        ( unsigned long )grabbedImage.GetTimeStamp().microSeconds,
                                        monoImage[RIGHT].GetData(),
                                        monoImage[LEFT].GetData(),
                                        monoImage[LEFT].GetData(),
                                        &triclopsMonoInput );

    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );

    return 0;
}

// Checked against PGR Code OK.RR
int CameraSystem::doStereo( TriclopsContext const & triclops,
                            TriclopsInput  const & stereoData,
                            TriclopsImage16      & depthImage,
                            TriclopsColorImage   & colorImage)
{
    TriclopsError te;

    // Set subpixel interpolation on to use
    // TriclopsImage16 structures when we access and save the disparity image
    te = triclopsSetSubpixelInterpolation( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

    te = triclopsSetDisparity( triclops, disp_min, disp_max );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

    te = triclopsSetDisparityMapping(triclops,disp_map_min,disp_map_max);
    _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparityMapping()", te );

    te = triclopsSetDisparityMappingOn(triclops, disp_map_on);
    _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparityMappingOn()", te );

    te = triclopsSetStereoMask(triclops, stereo_mask);
    _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparityMappingOn()", te );

    //ROS_INFO("stereoData x,y: %d,%d",stereoData.nrows, stereoData.ncols);
    te = triclopsSetResolution( triclops, stereoData.nrows, stereoData.ncols );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te );

    // Rectify the images
    te = triclopsRectify( triclops, const_cast<TriclopsInput *>( &stereoData ) );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

    // Do stereo processing
    te = triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );

    if ( grabbedImage.GetPixelFormat() == FC2::PIXEL_FORMAT_RAW16 )
    {
        te = triclopsRectifyColorImage( triclops,
                                        TriCam_REFERENCE,
                                        &(this->colorData),
                                        &colorImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );
    }

    // Retrieve the interpolated depth image from the context
    te = triclopsGetImage16( triclops,
                             TriImg16_DISPARITY,
                             TriCam_REFERENCE,
                             &depthImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

    return 0;
}

int CameraSystem::publishImages()
{
    cv::Mat      leftImage;
    cv::Mat      rightImage;
    cv::Mat disparityImageCV;
    cv::Mat rectifiedColorImageCv;

    FC2T::ErrorType fc2TriclopsError;
    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                           grabbedImage,
                           true /*assume little endian*/,
                           imageContainer.unprocessed[RIGHT],
                           imageContainer.unprocessed[LEFT] );

    if ( fc2TriclopsError != FC2T::ERRORTYPE_OK )
    {
        FC2T::handleFc2TriclopsError( fc2TriclopsError, "unpackUnprocessedRawOrMono16Image" );
    }

    // Convert FC2::Image to BGR format
    for ( int i = 0; i < 2; ++i )
    {
        convertToBGR( imageContainer.unprocessed[i], imageContainer.bgr[i] );
    }
    convertTriclops2Opencv( imageContainer.bgr[RIGHT], rightImage );
    convertTriclops2Opencv( imageContainer.bgr[LEFT], leftImage);
    convertTriclops2Opencv( this->disparityImageTriclops, disparityImageCV );
    convertTriclops2Opencv( this->rectifiedColorImage, rectifiedColorImageCv );
    //ROS_INFO("ImageBGR BitsperPixel: %d-- %d",imageContainer.bgr[LEFT].GetBitsPerPixel(),leftImage.channels());

    // Publish images ------------------------------------------
    sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage( std_msgs::Header(), "bgr8", rightImage ).toImageMsg();
    outmsg->header.frame_id = "bumblebee2";
    outmsg->header.stamp = ros::Time::now();
    this->image_pub_right.publish( outmsg );
    outmsg = cv_bridge::CvImage( std_msgs::Header(), "bgr8", leftImage).toImageMsg();
    outmsg->header.frame_id = "bumblebee2";
    outmsg->header.stamp = ros::Time::now();
    this->image_pub_left.publish( outmsg );
    outmsg = cv_bridge::CvImage( std_msgs::Header(), "mono16", disparityImageCV).toImageMsg();
    outmsg->header.frame_id = "bumblebee2";
    outmsg->header.stamp = ros::Time::now();
    this->image_pub_disparity.publish( outmsg );
    outmsg = cv_bridge::CvImage( std_msgs::Header(), "bgr8", rectifiedColorImageCv).toImageMsg();
    outmsg->header.frame_id = "bumblebee2";
    outmsg->header.stamp = ros::Time::now();
    this->image_pub_rectifiedColor.publish( outmsg );
}

void CameraSystem::run()
{
    FC2::Error fc2Error;
    // this image contains both right and left images
    fc2Error = this->camera.RetrieveBuffer( &( this->grabbedImage ) );

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        exit( FC2T::handleFc2Error( fc2Error ) );
    }

    // generate triclops inputs from grabbed image
    if ( this->generateTriclopsInput( this->grabbedImage, this->imageContainer, this->colorData, this->monoData ) )
    {
        exit( EXIT_FAILURE );
    }

    doStereo( this->triclops, this->colorData, this->disparityImageTriclops, this->rectifiedColorImage );
    publishImages();

    cv::Mat img(cv::Mat(5,300, CV_8U));
    img = cv::Scalar(50);
    cv::imshow("DisparityView",img);
    cv::waitKey(3);
    ros::spinOnce();
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

int main( int argc, char **argv )
{
    ros::init( argc, argv, "camera_system" );

    //ROS_INFO("camera.run()\n");
    CameraSystem camera( argc, argv );
    ros::Rate loop_rate( 15 );

    while ( ros::ok() )
    {
        camera.run();
        //ROS_INFO("camera.run()\n");
        loop_rate.sleep();
    }
}
