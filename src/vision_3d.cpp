
//=============================================================================
// stereoto3dpoints
//
// Takes input from a Bumblebee and performs subpixel
// interpolation to create a 16-bit disparity image, which is saved.
// The disparity data is then converted to 3-dimensional X/Y/Z
// coordinates which is written to a file.
//
// This point file can be viewed with PGRView under windows.
//
//=============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "typedefs.h"

#include "vision_3d.h"
#include "line_filter.h"

Vision3D::Vision3D( int argc, char **argv )
{
    image_transport::ImageTransport it( nh );

    this->hasDisparity = false;
    this->hasLeftFiltered = false;

    char fileName[] = "/home/user1/triclopsContextCurrent.txt";
    ROS_INFO( ">>>>> VISION3D GETTING CONTEXT FROM FILE" );

    if ( triclopsGetDefaultContextFromFile( &this->triclops, fileName ) )
    {
        ROS_INFO( ">>>>> FAILED GETTING CONTEXT FROM FILE" );
        exit( -1 );
    }

    this->pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>( "/vision3D/points", 0 );
    this->subcamdisp = it.subscribe( "/camera/disparity", 1, &Vision3D::visionCallBackDisparity, this );
    this->subcamfilteredleft = it.subscribe( "/camera/left/linefiltered", 0, &Vision3D::visionCallBackFilteredLeft, this );
    ros::Duration( 1 ).sleep(); // sleep for a second
}

Vision3D::~Vision3D()
{
    cvDestroyAllWindows();
}

void Vision3D::visionCallBackDisparity( const sensor_msgs::ImageConstPtr& msg )
{
    this->disparityImage = cv_bridge::toCvCopy( msg, "mono8" )->image;
    this->hasDisparity = true;
}

void Vision3D::visionCallBackFilteredLeft( const sensor_msgs::ImageConstPtr& msg )
{
    this->filteredLeft = cv_bridge::toCvCopy( msg, "mono8" )->image;
    this->hasLeftFiltered = true;
}

int Vision3D::maskToPointCloud( cv::Mat const &disparityImage,
                                 cv::Mat const &maskImage,
                                 PointCloud      & returnedPoints,
                                 TriclopsContext triclops )
{
    int i, j;
    float            x = 0.0;
    float            y = 0.0;
    float            z = 0.0;
    unsigned short   disparity; // The disparity value of the input pixel.
    unsigned char    mask;

    //cv::resize( this->cyan_image, disImage, cv::Size( 400, 300 ) );
    cv::imshow( "MASK Image", maskImage );
    cv::imshow( "DISPARITY Image", disparityImage );
    cv::waitKey( 3 );


    //printf( "[!!!!!!]rows,cols,channels,elemsize, maskImage: (%d,%d,%d,%d) disparity: (%d,%d,%d,%d)\n", maskImage.rows, maskImage.cols, maskImage.channels(), int( maskImage.elemSize() ), disparityImage.rows, disparityImage.cols, disparityImage.channels(), int( disparityImage.elemSize() ) );

    for ( i = 0; i < disparityImage.rows; i++ )
    {
        for ( j = 0; j < disparityImage.cols; j++ )
        {
            disparity = disparityImage.at<unsigned short>( cv::Point( i, j ) ); //row[j];
            //printf( "disparity, %d\n", disparity );

            // do not save invalid points
            if ( disparity < 0xFF00 )
            {
                mask = maskImage.at<unsigned char>( cv::Point( i, j ) );

                if ( mask != 0 )
                {
                    // convert the 16 bit disparity value to floating point x,y,z in ROS Coordinate Frame
                    triclopsRCD8ToXYZ( triclops, i, j, disparity, &x, &y, &z );
                    PointT point;
                    point.x = z;
                    point.y = -x;
                    point.z = -y;
                    point.r = mask;
                    point.g = mask;
                    point.b = mask;
                    returnedPoints.push_back( point );
                }
            }
        }
    }

    return 0;
}

int Vision3D::doPointCloud( FC2::Image      const & grabbedImage,
                            TriclopsContext const & triclops,
                            TriclopsImage16 const & disparityImage16,
                            TriclopsInput   const & colorData,
                            PointCloud      & returnedPoints )
{
    TriclopsImage monoImage = {0};
    TriclopsColorImage colorImage = {0};
    TriclopsError te;

    float            x, y, z;
    int              nPoints = 0;
    int              pixelinc ;
    int              i, j, k;
    unsigned short * row;
    unsigned short   disparity;
    PointT           point3d;

    // Rectify the color image if applicable
    bool isColor = false;

    if ( grabbedImage.GetPixelFormat() == FC2::PIXEL_FORMAT_RAW16 )
    {
        isColor = true;
        te = triclopsRectifyColorImage( triclops,
                                        TriCam_REFERENCE,
                                        const_cast<TriclopsInput *>( &colorData ),
                                        &colorImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );
    }
    else
    {
        te = triclopsGetImage( triclops,
                               TriImg_RECTIFIED,
                               TriCam_REFERENCE,
                               &monoImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
    }

    // The format for the output file is:
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // ...

    // Determine the number of pixels spacing per row
    pixelinc = disparityImage16.rowinc / 2;
    //ROS_INFO("DisparityData x,y: %d,%d",disparityImage16.nrows, disparityImage16.ncols);

    for ( i = 0, k = 0; i < disparityImage16.nrows; i++ )
    {
        row = disparityImage16.data + i * pixelinc;

        for ( j = 0; j < disparityImage16.ncols; j++, k++ )
        {
            disparity = row[j];

            // do not save invalid points
            if ( disparity < 0xFF00 )
            {
                // convert the 16 bit disparity value to floating point x,y,z
                triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

                // look at points within a range
                if ( z < 5.0 )
                {
                    point3d.x = z;
                    point3d.y = -x;
                    point3d.z = -y;

                    if ( isColor )
                    {
                        point3d.r = ( int )colorImage.red[k];
                        point3d.g = ( int )colorImage.green[k];
                        point3d.b = ( int )colorImage.blue[k];
                    }
                    else
                    {
                        // For mono cameras, we just assign the same value to RGB
                        point3d.r = ( int )monoImage.data[k];
                        point3d.g = ( int )monoImage.data[k];
                        point3d.b = ( int )monoImage.data[k];
                    }

                    returnedPoints.push_back( point3d );

                    //                    fprintf( pPointFile, "%f %f %f %d %d %d %d %d\n", x, y, z, r, g, b, i, j );
                    nPoints++;
                }
            }
        }
    }

    //ROS_INFO( "Points in file: %d\n", nPoints );
    return 0;
}


void Vision3D::run()
{
    if ( this->hasDisparity && this->hasLeftFiltered )
    {
        //TODO FIX THIS METHOD CALL
        maskToPointCloud( this->disparityImage.clone(),
                                    this->filteredLeft.clone(),
                                    this->cloud,
                                    this->triclops );

//        int Vision3D::producePointCloud( grabbedImage,
//                                         triclops,
//                                         disparityImage16,
//                                         colorData,
//                                         returnedPoints )
        //ROS_INFO("<><><><><><><><> After has Disparity image and filtered image\n");

        this->cloud.header.frame_id = "bumblebee2";
        this->cloud.header.stamp = ros::Time::now().toNSec();
        this->pointCloudPublisher.publish( this->cloud );
        this->cloud.clear();
    }
    else
    {
        ROS_INFO( "<><><><><><><><> Missing Disparity image or filtered image\n" );
    }

    ros::spinOnce();
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
int main( int argc, char **argv )
{
    ros::init( argc, argv, "vision3d" );

    //printf("vision.run()\n");
    Vision3D vision3D( argc, argv );
    ros::Rate loop_rate( 10 );

    while ( ros::ok() )
    {
        vision3D.run();
        //printf("vision3D.run()\n");
        loop_rate.sleep();
    }
}
