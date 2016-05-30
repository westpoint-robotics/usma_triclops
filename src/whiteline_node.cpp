#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "usma_triclops/triclops_opencv.h"
#include "usma_triclops/whiteline_filter.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    image_transport::ImageTransport it( nh );

    //Topic you want to publish
    image_pub = it.advertise( "/camera/linefiltered", 1 );

    //Topic you want to subscribe
    image_sub = it.subscribe( "/camera/rectified", 1, &SubscribeAndPublish::imageCallback, this);
  }

  void imageCallback( const sensor_msgs::ImageConstPtr& msg )
  {
      //Pull subscribed data inside this callback, formatting for linefilter
      cv::Mat cv_rectifiedColorImage = cv_bridge::toCvCopy( msg, "bgr8" )->image;
      cv::Mat cv_filteredImage = wl_filter.findLines(cv_rectifiedColorImage);

      //Publish the filtered image
      sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage( std_msgs::Header(), "mono8", cv_filteredImage ).toImageMsg();
      outmsg->header.frame_id = "bumblebee2";
      outmsg->header.stamp = ros::Time::now();
      this->image_pub.publish( outmsg );
  }

private:
  ros::NodeHandle nh;
  image_transport::Publisher image_pub;
  image_transport::Subscriber image_sub;
  WhitelineFilter wl_filter; // The white line filter

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "whiteline");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
