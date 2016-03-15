
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
#include "triclops_vision/typedefs.h"
#include "triclops_vision/common.h"
#include "triclops_vision/common.h"
#include "triclops_vision/line_filter.h"
#include "triclops_vision/camera_system.h"
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>


/**
 * @brief The Vision3D class. This class implements a filter and creates a point cloud from input
 *
 */

class Vision3D
{
    public:
        Vision3D(int argc, char **argv, CameraSystem *camera);
        virtual ~Vision3D();
        void run();
        // reference to the outside camera system
        CameraSystem* camerasystem;

    private:
        void visionCallBackDisparity(const sensor_msgs::ImageConstPtr& msg);
        void visionCallBackFilteredRight(const sensor_msgs::ImageConstPtr& msg);
        void visionCallBackFilteredLeft(const sensor_msgs::ImageConstPtr& msg);
        void visionCallBackRGBRight(const sensor_msgs::ImageConstPtr& msg);
        void visionCallBackRGBLeft(const sensor_msgs::ImageConstPtr& msg);
        int producePointCloud( cv::Mat const &disparity,
                  cv::Mat const &maskImage,
                  PointCloud      & returnedPoints);
        int numDisp;
        int blockSize;
        cv::Mat filteredLeft;
        cv::Mat filteredRight;
        cv::Mat imageLeft;
        cv::Mat imageRight;
        cv::Mat disparityImage;
        image_transport::Subscriber subcamfilteredright;
        image_transport::Subscriber subcamfilteredleft;
        image_transport::Subscriber subcamrgbright;
        image_transport::Subscriber subcamrgbleft;  
        image_transport::Subscriber subcamdisp; 
        ros::Publisher pointCloudPublisher;
        PointCloud cloud;

};

#endif // VISION_3D_H

