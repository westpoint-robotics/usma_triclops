
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
  ROS_INFO(">>>>> VISION3D AFTER GETTING CONTEXT FROM FILE");
  std::cout<<"Triclops is located at: " << &(this->triclops)<<std::endl; //0012FED4,


    this->pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/vision3D/points", 0);

    this->subcamdisp = it.subscribe("/camera/disparity", 1, &Vision3D::visionCallBackDisparity, this);
    this->subcamfilteredleft = it.subscribe("/camera/left/linefiltered", 0, &Vision3D::visionCallBackFilteredLeft, this);

    this->numDisp = 16 * 5;
    this->blockSize = 21;
    // this->camerasystem = camera;

    /* Create control sliders that allow tunning of the parameters for line detection
    cv::namedWindow("Disparity Control", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar( "Number Disparities", "Disparity Control", &(this->numDisp), 100*5);
    cv::createTrackbar( "Block Size", "Disparity Control", &(this->blockSize), 253);
    */
  ros::Duration(1).sleep(); // sleep for a second
  int min = 0;
  int max = 0;
  int offset = 0;
  triclopsGetDisparity(triclops, &min, &max);
  triclopsGetDisparityOffset(triclops, &offset);
  ROS_INFO("[][][][][][][ disparity min and max %d,%d, %d\n",min,max,offset);


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
  float            x = 0.0;
  float            y = 0.0;
  float            z = 0.0;
  unsigned short   disparity; // The disparity value of the input pixel.
  unsigned char    *disparityRow;
  unsigned char    *maskRow, mask;
  int offset = 0;
  //ROS_INFO("<><><><><><><><> Inside Produce Point cloud %d=%d and %d=%d %d\n",int(maskImage.cols),int(disparityImage.cols),int(maskImage.rows),int(disparityImage.rows),int(disparityImage.size));

  triclopsGetDisparityOffset(triclops,&offset );

  std::cout<<&triclops<<" OFFset " << offset <<std::endl; //

  //printf("[!] Searching through image rows,cols %i, %i...\n", disparityImage.rows,disparityImage.cols);
  for(int i = 0; i < disparityImage.rows; i++)
  {
    //disparityRow = disparityImage.data + (i * disparityImage.step);
    maskRow = maskImage.data + (i * maskImage.step);
    for(int j = 0; j < disparityImage.cols; j++)
    {
      //printf("At ROW: %i and COL: %i\n", i,j);
      //disparity = disparityRow[j];
      mask = maskRow[j];

      // do not run invalid points
      if(mask != 0)
      {
        disparity = disparityImage.at<uchar>(i,j);
        //ROS_INFO("[][][][][][][ disparity of %d,%d is %d\n",i,j,disparity);
        // convert the 16 bit disparity value to floating point x,y,z
        triclopsRCD16ToXYZ(triclops, i, j, disparity, &x, &y, &z);
        //ROS_INFO("[][][][][][][ after RCD16CALL\n");
        // look at points within a range
        PointT point;
        //only fil out for points that are cyan
        if(mask == 0xFF)
        {
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
  //ROS_INFO("::::::: POINTCLOUD IS: %d by %d", returnedPoints.height, returnedPoints.width);


  return 0;
}

void Vision3D::run()
{
  if(this->hasDisparity && this->hasLeftFiltered)
  {
    producePointCloud(this->disparityImage.clone(), this->filteredLeft.clone(), this->cloud, this->triclops);
    ROS_INFO("<><><><><><><><> After has Disparity image and filtered image\n");

    this->cloud.header.frame_id = "map";
    this->cloud.header.stamp = ros::Time::now().toNSec();
    //this->pointCloudPublisher.publish(this->cloud);
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
