#include "triclops/triclops.h"
#include "triclops/fc2triclops.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "usma_triclops/typedefs.h"

#include "usma_triclops/vision3d.h"

/**
At 5mph the robot moves 0.0014 miles/sec or	2.2352 meters/sec.
In a 15th of a second the robot moves 0.1490 meters.
In a 30th of a second the robot moves 0.0745 meters.
**/

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    image_transport::ImageTransport it( nh );

    //Topic you want to publish
    //image_pub = it.advertise( "/camera/linefiltered", 1 );
    whiteLinePcPublisher= nh.advertise<sensor_msgs::PointCloud2>("/vision3D/points", 0);
    redflagPcPublisher= nh.advertise<sensor_msgs::PointCloud2>("/redflag/points", 0);
    blueflagPcPublisher= nh.advertise<sensor_msgs::PointCloud2>("/blueflag/points", 0);

    //Topic you want to subscribe
    image_sub = it.subscribe( "/camera/linefiltered", 1, &SubscribeAndPublish::imageCallback, this);
    disp_sub = it.subscribe( "/camera/disparity", 1, &SubscribeAndPublish::visionCallBackDisparity, this);
    this->hasDisparity = false;

    char fileName[] = "/home/user1/triclopsContextCurrent.txt";
    ROS_INFO(">>>>> VISION3D GETTING CONTEXT FROM FILE");
    if (triclopsGetDefaultContextFromFile(&this->triclops, fileName)) {
        ROS_INFO(">>>>> FAILED GETTING CONTEXT FROM FILE");
        exit(-1);
    }
  }

  void visionCallBackDisparity(const sensor_msgs::ImageConstPtr& msg)
  {
      this->disparityImageIn = cv_bridge::toCvCopy(msg, "mono16")->image;
      this->hasDisparity = true;
      // ROS_INFO("INSIDE DISP CALLBACCK");

  }

  void imageCallback( const sensor_msgs::ImageConstPtr& msg )
  {
      if (this->hasDisparity){
      //Pull subscribed data inside this callback, formatting for linefilter
      cv::Mat cv_filteredImage = cv_bridge::toCvCopy( msg, "bgr8" )->image;

      PointCloud whiteLinePoints;
      vision.producePointCloud(disparityImageIn, cv_filteredImage, triclops, whiteLinePoints);

      // Publish the point clouds
      whiteLinePoints.header.frame_id = "bumblebee2";
      // returnedPoints.header.stamp = ros::Time::now().toNSec();
      //TODO fix the timestamp. The above code causing timing problems.
      whiteLinePcPublisher.publish(whiteLinePoints);
      whiteLinePoints.clear();

      //Publish the filtered image
      sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage( std_msgs::Header(), "mono8", cv_filteredImage ).toImageMsg();
      outmsg->header.frame_id = "bumblebee2";
      outmsg->header.stamp = ros::Time::now();
      //this->image_pub.publish( outmsg );
      }
      else{ // Warn because no disparity image found.
        ROS_INFO("<><><> Missing Disparity image, can't produce a point cloud without it.");
      }
  }

private:
  ros::NodeHandle nh;
  // Create the topic publishers for the pointclouds
  ros::Publisher whiteLinePcPublisher;
  ros::Publisher redflagPcPublisher;
  ros::Publisher blueflagPcPublisher;
  cv::Mat disparityImageIn;
  bool hasDisparity;
  image_transport::Subscriber image_sub;
  image_transport::Subscriber disp_sub;
  Vision3d vision;
  TriclopsContext triclops;
};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "vision3d");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
