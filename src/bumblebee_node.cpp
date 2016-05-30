
#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "usma_triclops/triclops_opencv.h"
#include "usma_triclops/bumblebeecamera.h"

using namespace std;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bumbleBee");
    ros::NodeHandle nh;
    image_transport::ImageTransport it( nh );

    // TODO use the ROS stereo_msgs::disparity message instead of an image message
    image_transport::Publisher disparityPublisher = it.advertise("/camera/disparity", 1);
    image_transport::Publisher rectifiedPublisher = it.advertise("/camera/rectified", 1);

    BumbleBeeCamera bb2; // The camera driver

    bb2.startCamera();

    ros::Rate loop_rate(15);
    while (ros::ok()) {

        // Get the disparity and rectified images.
        bb2.retrieveImages();
        TriclopsImage16 tri_disparityImage = bb2.getDisparityImage();
        TriclopsColorImage tri_rectifiedColorImage = bb2.getRectifiedColorImage();

        // Convert the images to opencv formats
        cv::Mat cv_rectifiedColorImage;
        cv::Mat cv_disparityImage;
        convertTriclops2Opencv(tri_rectifiedColorImage, cv_rectifiedColorImage);
        convertTriclops2Opencv(tri_disparityImage, cv_disparityImage);

        //Publish the images to ros
        sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_rectifiedColorImage).toImageMsg();
        outmsg->header.frame_id = "bumblebee2";
        outmsg->header.stamp = ros::Time::now();
        rectifiedPublisher.publish(outmsg);

        outmsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", cv_disparityImage).toImageMsg();
        outmsg->header.frame_id = "bumblebee2";
        outmsg->header.stamp = ros::Time::now();
        disparityPublisher.publish(outmsg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
