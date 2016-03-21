
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
#include "triclops_vision/typedefs.h"

#include "triclops_vision/vision_3d.h"
#include "triclops_vision/line_filter.h"

Vision3D::Vision3D(int argc, char **argv)
{
  ros::init(argc, argv, "vision3d");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  this->hasDisparity = false;
  this->hasLeftFiltered = false;

  char fileName[] = "/home/user1/triclopsContextCurrent.txt";
  ROS_INFO(">>>>> VISION3D GETTING CONTEXT FROM FILE");
  if(triclopsGetDefaultContextFromFile(&this->triclops, fileName))
  {
    ROS_INFO(">>>>> FAILED GETTING CONTEXT FROM FILE");
    exit(-1);
  }
    this->pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/vision3D/points", 0);
    this->subcamdisp = it.subscribe("/camera/disparity", 1, &Vision3D::visionCallBackDisparity, this);
    this->subcamfilteredleft = it.subscribe("/camera/left/linefiltered", 0, &Vision3D::visionCallBackFilteredLeft, this);
  ros::Duration(1).sleep(); // sleep for a second
}

Vision3D::~Vision3D()
{
  cvDestroyAllWindows();
}

void Vision3D::visionCallBackDisparity(const sensor_msgs::ImageConstPtr& msg)
{
  this->disparityImage = cv_bridge::toCvCopy(msg, "mono16")->image;
  this->hasDisparity = true;
}

void Vision3D::visionCallBackFilteredLeft(const sensor_msgs::ImageConstPtr& msg)
{
  this->filteredLeft = cv_bridge::toCvCopy(msg, "mono8")->image;
  this->hasLeftFiltered = true;
}

int Vision3D::producePointCloud(cv::Mat const &disparityImage,
                                cv::Mat const &maskImage,
                                PointCloud      & returnedPoints,
                                TriclopsContext triclops)
{
  int i,j,k;
  float            x = 0.0;
  float            y = 0.0;
  float            z = 0.0;
  unsigned short   disparity; // The disparity value of the input pixel.
  unsigned char    mask;
  int	             pixelinc ;
  //printf("[!] Searching through image rows,cols %i, %i...\n", disparityImage.rows,disparityImage.cols);
    for (i = 0, k = 0; i < disparityImage.nrows; i++ )  {
    //disparityRow = disparityImage.data + (i * disparityImage.step);
        row = disparityImage16.data + i * pixelinc;
        for ( j = 0; j < disparityImage16.ncols; j++, k++ )
    {
      disparity = row[j];
      //printf("At ROW: %i and COL: %i\n", i,j);

      // do not save invalid points
      if ( disparity < 0xFF00 )
      {
         mask = maskImage[i,j];
      if(mask != 0)
      {
        disparity = disparityImage.at<uchar>(i,j);
        // convert the 16 bit disparity value to floating point x,y,z
        triclopsRCD16ToXYZ(triclops, i, j, disparity, &x, &y, &z);
        PointT point;
          point.x = z;
          point.y = -x;
          point.z = -y;
          point.r = disparity;
          point.g = disparity;
          point.b = disparity;
          returnedPoints.push_back(point);
      }
      }
    }
  }
  return 0;
}

void Vision3D::run()
{
  if(this->hasDisparity && this->hasLeftFiltered)
  {
    producePointCloud(this->disparityImage.clone(), this->filteredLeft.clone(), this->cloud, this->triclops);
    //ROS_INFO("<><><><><><><><> After has Disparity image and filtered image\n");

    this->cloud.header.frame_id = "map";
    this->cloud.header.stamp = ros::Time::now().toNSec();
    this->pointCloudPublisher.publish(this->cloud);
    this->cloud.clear();
  }
  else
  {
    ROS_INFO("<><><><><><><><> Missing Disparity image or filtered image\n");
  }

  ros::spinOnce();
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision3d");
  ros::NodeHandle nh;
  ros::Rate loop_rate(5);

  //printf("vision.run()\n");
  Vision3D vision3D(argc, argv);

  while(ros::ok())
  {
    vision3D.run();
    //printf("vision3D.run()\n");
    loop_rate.sleep();
  }
}
