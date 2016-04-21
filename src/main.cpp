#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include <stereo_msgs/DisparityImage.h>

#include "usma_triclops/bumblebeecamera.h"
#include "usma_triclops/triclops_opencv.h"
#include "usma_triclops/whiteline_filter.h"
#include "usma_triclops/vision3d.h"
#include "usma_triclops/color_filter.h"

using namespace std;

bool debug = false;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bumbleBee");
    ros::NodeHandle nh;

    // Create the topic publishers for the pointclouds
    ros::Publisher whiteLinePcPublisher = nh.advertise<sensor_msgs::PointCloud2>("/vision3D/points", 0);
    ros::Publisher redflagPcPublisher = nh.advertise<sensor_msgs::PointCloud2>("/redflag/points", 0);
    ros::Publisher blueflagPcPublisher = nh.advertise<sensor_msgs::PointCloud2>("/blueflag/points", 0);
    // TODO use the ROS stereo_msgs::disparity message instead of an image message
    ros::Publisher disparityPublisher = nh.advertise<sensor_msgs::Image>("/triclops/disparity", 1);
    ros::Publisher rectifiedPublisher = nh.advertise<sensor_msgs::Image>("/triclops/rectified", 1);

    WhitelineFilter wl_filter; // The white line filter
    Vision3d i3d; // The pointcloud creatorImage
    Color_Filter cf; // The color filter for red and blue
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
        sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC3, cv_rectifiedColorImage).toImageMsg();
        outmsg->header.frame_id = "bumblebee2";
        outmsg->header.stamp = ros::Time::now();
        rectifiedPublisher.publish(outmsg);

        outmsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, cv_disparityImage).toImageMsg();
        outmsg->header.frame_id = "bumblebee2";
        outmsg->header.stamp = ros::Time::now();
        disparityPublisher.publish(outmsg);

        if (debug) {
            cv::imshow("RectifiedImage", cv_rectifiedColorImage);
            cv::imshow("DisparityImage", cv_disparityImage);
        }
        cv::waitKey(3);

        // Find the whitelines and put them into a point cloud
        cv::Mat mask = wl_filter.findLines(cv_rectifiedColorImage);
        PointCloud whiteLinePoints;
        i3d.producePointCloud(cv_disparityImage, mask, bb2.getTriclopsContext(),
            whiteLinePoints);

        // Find the blue flags and put them into a point cloud
        cv::Mat blueBlob = cf.findBlueHsv(cv_rectifiedColorImage);
        PointCloud bluePoints;
        i3d.producePointCloud(cv_disparityImage, blueBlob, bb2.getTriclopsContext(),
            bluePoints);

        // Find the red flags and put them into a point cloud
        cv::Mat redBlob = cf.findRedHsv(cv_rectifiedColorImage);
        PointCloud redPoints;
        i3d.producePointCloud(cv_disparityImage, redBlob, bb2.getTriclopsContext(),
            redPoints);

        // Below code could be used to tune the vision. To find the HSV limits.
        //    cf.filterControl();
        //    cv::Mat hsvBlob =cf.findContorllerHsv(cv_rectifiedColorImage);
        //    cv::imshow("hsvBlob", hsvBlob );
        //    cv::waitKey(3);

        // Publish the point clouds
        whiteLinePoints.header.frame_id = "bumblebee2";
        redPoints.header.frame_id = "bumblebee2";
        bluePoints.header.frame_id = "bumblebee2";
        // returnedPoints.header.stamp = ros::Time::now().toNSec();
        //TODO fix the timestamp. The above code causing timing problems.
        whiteLinePcPublisher.publish(whiteLinePoints);
        redflagPcPublisher.publish(redPoints);
        blueflagPcPublisher.publish(bluePoints);
        whiteLinePoints.clear();

        if (debug) {
            wl_filter.displayCyan();
            wl_filter.displayThreshold();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    cout << "Shutting Down!" << endl;
    bb2.shutdown();
    return 0;
}
