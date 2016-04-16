
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
#include "usma_triclops/typedefs.h"
#include "usma_triclops/vision_3d.h"
#include "usma_triclops/line_filter.h"

Vision3D::Vision3D(int argc, char** argv)
{
    image_transport::ImageTransport it(nh);
    if (nh.hasParam("/vision3d/triclopsMode")) {
        nh.getParam("/vision3d/triclopsMode", this->mode);
        ROS_INFO("This mode is set to %d", this->mode);
    }
    else {
        this->mode = 0;
        ROS_INFO("This mode is not found");
    }
    this->hasDisparity = false;
    this->hasRectifiedFiltered = false;
    this->hasrectifiedColor = false;

    char fileName[] = "/home/user1/triclopsContextCurrent.txt";
    ROS_INFO(">>>>> VISION3D GETTING CONTEXT FROM FILE");

    if (triclopsGetDefaultContextFromFile(&this->triclops, fileName)) {
        ROS_INFO(">>>>> FAILED GETTING CONTEXT FROM FILE");
        exit(-1);
    }

    this->pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/vision3D/points", 0);
    this->subcamdisp = it.subscribe("/camera/disparity", 1,
        &Vision3D::visionCallBackDisparity, this);
    // TODO Stop using left image and use a rectified grayscale image for line
    // filtering.
    this->subcamfilteredrectified = it.subscribe("/camera/rectified/linefiltered", 0,
        &Vision3D::visionCallBackFilteredRectified, this);
    this->subRectColor = it.subscribe("/camera/color_rectified", 0,
        &Vision3D::visionCallBackRectColor, this);
    ros::Duration(1).sleep(); // sleep for a second
}

Vision3D::~Vision3D() { cvDestroyAllWindows(); }

void Vision3D::visionCallBackDisparity(const sensor_msgs::ImageConstPtr& msg)
{
    this->disparityImageIn = cv_bridge::toCvCopy(msg, "mono16")->image;
    this->hasDisparity = true;
    // ROS_INFO("INSIDE DISP CALLBACCK");
}

void Vision3D::visionCallBackFilteredRectified(
    const sensor_msgs::ImageConstPtr& msg)
{
    this->filteredRectified = cv_bridge::toCvCopy(msg, "mono8")->image;
    this->hasRectifiedFiltered = true;
    // ROS_INFO("INSIDE filtered CALLBACCK");
}

void Vision3D::visionCallBackRectColor(const sensor_msgs::ImageConstPtr& msg)
{
    this->rectifiedColor = cv_bridge::toCvCopy(msg, "bgr8")->image;
    this->hasrectifiedColor = true;
    // ROS_INFO("CALLBACK POINTER IS: %p", &rectifiedColor);
}

// TODO Cobine this with do 3Dpoint and put if statement in the middle based on
// the triclops vision mode.
int Vision3D::maskToPointCloud(cv::Mat const& disparityImage16,
    cv::Mat const& maskImage,
    PointCloud& returnedPoints,
    TriclopsContext const& triclops)
{
    int i, j, k;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    // int              pixelinc ;
    // unsigned short * row;
    unsigned short disparity; // The disparity value of the input pixel.
    unsigned char mask;

    // ROS_INFO("DisparityData x,y size: %d,%d",disparityImage16.rows,
    // disparityImage16.cols);

    for (i = 0, k = 0; i < disparityImage16.rows; i++) {
        // row = disparityImage16.data + i * pixelinc;

        for (j = 0; j < disparityImage16.cols; j++, k++) {
            // disparity = row[j];
            disparity = disparityImage16.at<unsigned short>(i, j);
            mask = maskImage.at<uchar>(i, j);

            // do not save invalid points
            if (disparity < 0xFF00) {
                // if ( mask != 0 )
                {
                    // convert the 16 bit disparity value to floating point x,y,z in ROS
                    // Coordinate Frame
                    triclopsRCD16ToXYZ(triclops, i, j, disparity, &x, &y, &z);
                    PointT point;
                    point.x = z;
                    point.y = -x;
                    point.z = -y;
                    point.r = mask;
                    point.g = mask;
                    point.b = mask;
                    returnedPoints.push_back(point);
                }
            }
        }
    }

    return 0;
}

int Vision3D::doPointCloud(cv::Mat const& disparityImage16, cv::Mat& colorImage,
    PointCloud& returnedPoints,
    TriclopsContext const& triclops)
{
    float x, y, z;
    int nPoints = 0;
    int i, j;
    unsigned short disparity;
    PointT point3d;
    cv::Vec3b colorPixel;

    //     The format for the output file is:
    //     <x> <y> <z> <red> <grn> <blu> <row> <col>
    //     <x> <y> <z> <red> <grn> <blu> <row> <col>
    //     ...

    // ROS_INFO("Disp x,y color x, y: %d,%d,%d  : %d,%d, %d,%d,
    // %d",disparityImage16.rows,
    // disparityImage16.cols,int(disparityImage16.step),colorImage.rows,
    // colorImage.cols,
    // int(colorImage.step),colorImage.channels(),colorImage.type());
    if (colorImage.type() == CV_8UC3) {
        for (i = 0; i < colorImage.rows; i++) {
            for (j = 0; j < colorImage.cols; j++) {
                disparity = disparityImage16.at<unsigned short>(i, j);
                colorPixel = colorImage.at<cv::Vec3b>(i, j);

                // do not save invalid points
                if (disparity < 0xFF00) {
                    // convert the 16 bit disparity value to floating point x,y,z
                    triclopsRCD16ToXYZ(triclops, i, j, disparity, &x, &y, &z);

                    // look at points within a range
                    if (z < 5.0) {
                        point3d.x = z;
                        point3d.y = -x;
                        point3d.z = -y;
                        point3d.r = colorPixel[2];
                        point3d.g = colorPixel[1];
                        point3d.b = colorPixel[0];
                        returnedPoints.push_back(point3d);
                        nPoints++;
                    }
                }
            }
        }
    }
    else {
        ROS_INFO(
            "IMAGE IS NOT CV_U8C3 "
            "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    }

    // ROS_INFO( "Points in file: %d\n", nPoints );
    return 0;
}

int Vision3D::producePointCloud(cv::Mat const& disparityImage,
    cv::Mat const& maskImage,
    TriclopsContext const& triclops,
    PointCloud& returnedPoints)
{
    float x, y, z;
    int i = 0, j = 0;
    unsigned short disparity;
    unsigned char mask;

    // printf("[!] Searching through image at %p for obstacles..mask
    // %d,%d,%d,%d,%d and dispar %d,%d,%d,%d,%d\n",
    // &maskImage,maskImage.cols,maskImage.rows,int(maskImage.step),maskImage.channels(),int(maskImage.elemSize()),disparityImage.cols,disparityImage.rows,int(disparityImage.step),disparityImage.channels(),int(disparityImage.elemSize())
    // );
    for (i = 0; i < disparityImage.rows; i++) {
        for (j = 0; j < disparityImage.cols; j++) {
            disparity = disparityImage.at<unsigned short>(i, j);
            mask = maskImage.at<unsigned char>(i, j);

            // do not run invalid points
            if (disparity < 0xFF00) {
                // look at points within a range
                PointT point;

                // only fil out for points that are cyan
                if (mask != 0) {
                    triclopsRCD16ToXYZ(triclops, i, j, disparity, &x, &y, &z);
                    point.x = z;
                    point.y = -x;
                    point.z = -y;
                    point.r = 255;
                    point.g = 255;
                    point.b = 255;
                    returnedPoints.push_back(point);
                }
            }
        }
    }

    return 0;
}

void Vision3D::run()
{
    PointCloud cloud;

    if (this->hasDisparity && this->hasRectifiedFiltered && this->hasrectifiedColor) {
        // TODO check if the mode is being set correctly as a param in ros
        // TODO move these if statements into the method calls.
        if (this->mode == 1) {
            doPointCloud(this->disparityImageIn, this->rectifiedColor, cloud,
                this->triclops);
        }
        else if (this->mode == 0) {
            maskToPointCloud(this->disparityImageIn.clone(),
                this->filteredRectified.clone(), cloud, this->triclops);
        }
        else {
            producePointCloud(this->disparityImageIn, this->filteredRectified,
                this->triclops, cloud);
        }
        // ROS_INFO("<><><><><><><><> After has Disparity image and filtered
        // image\n");

        cloud.header.frame_id = "bumblebee2";
        cloud.header.stamp = ros::Time::now().toNSec();
        this->pointCloudPublisher.publish(cloud);
        cloud.clear();
    }
    else {
        ROS_INFO("<><><><><><><><> Missing Disparity image or filtered image\n");
    }

    ros::spinOnce();
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision3d");

    // printf("vision.run()\n");
    Vision3D vision3D(argc, argv);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        vision3D.run();
        // printf("vision3D.run()\n");
        loop_rate.sleep();
    }
}
/*
user1@ros07:~/catkin_ws$ rostopic echo -n 1 /camera/left/linefiltered
header:
  seq: 725
  stamp:
    secs: 1459640920
    nsecs: 465328136
  frame_id: bumblebee2
height: 480
width: 640
encoding: mono8
is_bigendian: 0
step: 640

user1@ros07:~/catkin_ws$ rostopic echo -n 1 /camera/disparity
header:
  seq: 230
  stamp:
    secs: 1459641315
    nsecs: 856924595
  frame_id: bumblebee2
height: 480
width: 640
encoding: mono16
is_bigendian: 0
step: 1280

user1@ros07:~/catkin_ws$ rostopic echo -n 1 /camera/color_rectified
header:
  seq: 3398
  stamp:
    secs: 1459641375
    nsecs: 645998935
  frame_id: bumblebee2
height: 480
width: 640
encoding: bgr8
is_bigendian: 0
step: 1920

user1@ros07:~$ rostopic echo -n 1 /camera/left/rgb
header:
  seq: 196
  stamp:
    secs: 1459651779
    nsecs: 684287520
  frame_id: bumblebee2
height: 480
width: 640
encoding: bgr8
is_bigendian: 0
step: 2560


 */
