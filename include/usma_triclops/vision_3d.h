
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

#ifndef VISION_3D_H
#define VISION_3D_H

#include <stdio.h>
#include <stdlib.h>
#include "triclops/triclops.h"
#include "triclops/fc2triclops.h"
#include <pcl_ros/point_cloud.h>
#include "typedefs.h"
#include "common.h"
#include "common.h"
#include "line_filter.h"
#include "camera_system.h"
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>


/**
 * @brief The Vision3D class. This class implements a filter and creates a point cloud from input
 *
 */

class Vision3D
{
    public:
        Vision3D( int argc, char **argv );
        virtual ~Vision3D();
        void run();
        TriclopsContext triclops;

    private:
        void visionCallBackDisparity( const sensor_msgs::ImageConstPtr& msg );
        void visionCallBackFilteredRight( const sensor_msgs::ImageConstPtr& msg );
        void visionCallBackFilteredRectified( const sensor_msgs::ImageConstPtr& msg );
        void visionCallBackRGBRight( const sensor_msgs::ImageConstPtr& msg );
        void visionCallBackRGBLeft( const sensor_msgs::ImageConstPtr& msg );
        void visionCallBackRectColor( const sensor_msgs::ImageConstPtr& msg );
        int doPointCloud( cv::Mat const &disparityImage16,
                          cv::Mat &colorImage,
                          PointCloud      & returnedPoints,
                          TriclopsContext const & triclops );


        int maskToPointCloud( cv::Mat const &disparityImageIn,
                              cv::Mat const &maskImage,
                              PointCloud      & returnedPoints,
                              TriclopsContext const & triclops );
        int producePointCloud( cv::Mat const &disparityImage,
                               cv::Mat const &maskImage,
                               TriclopsContext const & triclops,
                               PointCloud      & returnedPoints );

        cv::Mat disparityImageIn;

        int numDisp;
        int blockSize;
        cv::Mat filteredRectified;
        cv::Mat filteredRight;
        cv::Mat rectifiedColor;
        image_transport::Subscriber subcamfilteredright;
        image_transport::Subscriber subcamfilteredrectified;
        image_transport::Subscriber subcamdisp;
        image_transport::Subscriber subRectColor;
        ros::Publisher pointCloudPublisher;
        ros::NodeHandle nh;
        int mode;


        bool hasDisparity;
        bool hasRectifiedFiltered;
        bool hasrectifiedColor;
};

#endif // VISION_3D_H

