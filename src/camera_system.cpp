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
#include "image_publisher.h"
#include "camera_system.h"

//TODO: Move this back to a seperate file
int convertTriclops2Opencv( TriclopsImage & bgrImage,
                            cv::Mat & cvImage )
{
    //printf("triImage row,col,rowinc %d,%d,%d",bgrImage.nrows,bgrImage.ncols,bgrImage.rowinc);
    cvImage = cv::Mat( bgrImage.nrows, bgrImage.ncols, CV_8UC1, bgrImage.data, bgrImage.rowinc );
    return 0;
}

//TODO: Move this back to a seperate file
int convertTriclops2Opencv( TriclopsImage16 & bgrImage,
                            cv::Mat & cvImage )
{
    cvImage = cv::Mat( bgrImage.nrows, bgrImage.ncols, CV_16UC1, bgrImage.data, bgrImage.rowinc );
    char numstr[50];
    sprintf( numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows, cvImage.cols, bgrImage.rowinc );
    putText( cvImage, numstr, cv::Point( 10, cvImage.rows - 30 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 100, 100, 250 ), 1, false );
}

CameraSystem::CameraSystem( int argc, char** argv )
{
    disp_min = 0;
    disp_max = 70;
    disp_map_max = 255;
    disp_map_min = 0;
    disp_map_on = 0;
    stereo_mask = 11;

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
    printf( "mode,offX,offY,width,height,pixFormat: %d,%d,%d,%d,%d,%x,%f\n", formatSettings.mode, formatSettings.offsetX, formatSettings.offsetY, formatSettings.width, formatSettings.height, formatSettings.pixelFormat, percent );

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
        printf( "Failed to get camera info from camera\n" );
        exit( -1 );
    }
    else
    {
        printf( ">>>>> CAMERA INFO  Vendor: %s     Model: %s     Serail#: %d  Resolution: %s", camInfo.vendorName, camInfo.modelName, camInfo.serialNumber, camInfo.sensorResolution );
    }


    // Container of Images used for processing
    image_transport::ImageTransport it( nh );
    //Publishers for the camera
    this->image_pub_left = it.advertise( "/camera/left/rgb", 1 );
    this->image_pub_right = it.advertise( "/camera/right/rgb", 1 );
    this->image_pub_disparity = it.advertise( "/camera/disparity", 1 );
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
                            TriclopsImage16      & depthImage )
    {
      TriclopsError te;

      // Set subpixel interpolation on to use
      // TriclopsImage16 structures when we access and save the disparity image
      te = triclopsSetSubpixelInterpolation( triclops, 1 );
      _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

      te = triclopsSetDisparity( triclops, 0, 70 );
      _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

      //ROS_INFO("stereoData x,y: %d,%d",stereoData.nrows, stereoData.ncols);
      te = triclopsSetResolution( triclops, stereoData.nrows, stereoData.ncols );
      _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te );

      // Rectify the images
      te = triclopsRectify( triclops, const_cast<TriclopsInput *>( &stereoData ) );
      _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

      // Do stereo processing
      te = triclopsStereo( triclops );
      _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );

      // Retrieve the interpolated depth image from the context
      te = triclopsGetImage16( triclops,
                               TriImg16_DISPARITY,
                               TriCam_REFERENCE,
                               &depthImage );
      _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

      return 0;
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

    ImageContainer imageContainer;

    // generate triclops inputs from grabbed image
    if ( this->generateTriclopsInput( this->grabbedImage, imageContainer, this->color, this->mono ) )
    {
        exit( EXIT_FAILURE );
    }

    doStereo( this->triclops, this->color, this->disparityImageTriclops );
    convertTriclops2Opencv( this->disparityImageTriclops, this->disparityImageCV );

    // Publish images
    ImagePublisher imagePublisher( this->grabbedImage, imageContainer, &( this->image_pub_left ), &( this->image_pub_right ) );
    //printf("dispIm B4Pub: r,c,in: %d,%d,%d",disparityImageCV.rows,disparityImageCV.cols,int(disparityImageCV.step));
    sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage( std_msgs::Header(), "mono8", this->disparityImageCV ).toImageMsg();
    this->image_pub_disparity.publish( outmsg );
    ros::spinOnce();
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

int main( int argc, char **argv )
{
    ros::init( argc, argv, "camera_system" );

    //printf("camera.run()\n");
    CameraSystem camera( argc, argv );
    ros::Rate loop_rate( 10 );

    while ( ros::ok() )
    {
        camera.run();
        //printf("camera.run()\n");
        loop_rate.sleep();
    }
}
