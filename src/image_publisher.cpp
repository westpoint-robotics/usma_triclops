#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "usma_triclops/typedefs.h"
#include "usma_triclops/triclops_opencv.h"
#include "usma_triclops/image_publisher.h"
#include <image_transport/image_transport.h>

int convertToBGR( FC2::Image & image, FC2::Image & convertedImage )
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

ImagePublisher::ImagePublisher( FC2::Image grabbedImage, ImageContainer imageContainer, image_transport::Publisher *image_pub_left, image_transport::Publisher *image_pub_right )
{

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

    // convert images to OpenCV Mat
    cv::Mat      leftImage;
    cv::Mat      rightImage;
    unsigned int rowBytes = ( double )imageContainer.bgr[LEFT].GetDataSize() / ( double )imageContainer.bgr[LEFT].GetRows();
    //ROS_INFO(">>>>> ROW Bytes: %d rows %d cols %d\n",rowBytes,imageContainer.bgr[LEFT].GetRows(), imageContainer.bgr[LEFT].GetCols());
    leftImage = cv::Mat( imageContainer.bgr[LEFT].GetRows(), imageContainer.bgr[LEFT].GetCols(), CV_8UC3, imageContainer.bgr[LEFT].GetData(), rowBytes );
    rowBytes = ( double )imageContainer.bgr[RIGHT].GetDataSize() / ( double )imageContainer.bgr[RIGHT].GetRows();
    rightImage = cv::Mat( imageContainer.bgr[RIGHT].GetRows(), imageContainer.bgr[RIGHT].GetCols(), CV_8UC3, imageContainer.bgr[RIGHT].GetData(), rowBytes );

    // Publish the images using ROS image containers
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage( std_msgs::Header(), "bgr8", leftImage ).toImageMsg();
    image_pub_left->publish( msg );
    msg = cv_bridge::CvImage( std_msgs::Header(), "bgr8", rightImage ).toImageMsg();
    image_pub_right->publish( msg );
}
