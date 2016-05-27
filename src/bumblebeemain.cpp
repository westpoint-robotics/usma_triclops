#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "usma_triclops/bumblebeecamera.h"
#include "usma_triclops/triclops_opencv.h"
#include "usma_triclops/whiteline_filter.h"
#include "usma_triclops/vision3d.h"
#include "usma_triclops/color_filter.h"

using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "bumbleBee");
  ros::NodeHandle nh;

  ros::Publisher whiteLinePcPublisher = nh.advertise<sensor_msgs::PointCloud2>("/vision3D/points", 0);
  ros::Publisher redflagPcPublisher = nh.advertise<sensor_msgs::PointCloud2>("/redflag/points", 0);
  ros::Publisher blueflagPcPublisher = nh.advertise<sensor_msgs::PointCloud2>("/blueflag/points", 0);
  ros::Rate loop_rate(5);
  BumbleBeeCamera bb2;
  WhitelineFilter wl_filter;
  Images3d i3d;
  Color_Filter cf;

  bb2.startCamera();
  while (ros::ok()) {
    bb2.retrieveImages(); // This uses 95% of cpu at 5hz
    TriclopsImage16 tri_disparityImage = bb2.getDisparityImage();
    TriclopsColorImage tri_rectifiedColorImage;
    tri_rectifiedColorImage = bb2.getRectifiedColorImage();
    tri_disparityImage = bb2.getDisparityImage();

    cv::Mat cv_rectifiedColorImage;
    cv::Mat cv_disparityImage;
    convertTriclops2Opencv(tri_rectifiedColorImage, cv_rectifiedColorImage);
    convertTriclops2Opencv(tri_disparityImage, cv_disparityImage);
    cv::imshow("RectifiedImage", cv_rectifiedColorImage);
    cv::imshow("DisparityImage", cv_disparityImage);
    cv::waitKey(3);

    cv::Mat mask = wl_filter.findLines(cv_rectifiedColorImage);
    PointCloud whiteLinePoints;
    i3d.producePointCloud(cv_disparityImage,mask, bb2.getTriclopsContext(),whiteLinePoints);

    cv::Mat blueBlob =cf.findBlueHsv(cv_rectifiedColorImage);
    PointCloud bluePoints;
    i3d.producePointCloud(cv_disparityImage,blueBlob, bb2.getTriclopsContext(),bluePoints);

    cv::Mat redBlob =cf.findRedHsv(cv_rectifiedColorImage);
    PointCloud redPoints;
    i3d.producePointCloud(cv_disparityImage,redBlob, bb2.getTriclopsContext(),redPoints);

    //cf.filterControl();
    cv::Mat hsvBlob =cf.findContorllerHsv(cv_rectifiedColorImage);

    // Show blobs
    cv::imshow("hsvBlob", hsvBlob );
    cv::waitKey(3);


    whiteLinePoints.header.frame_id = "bumblebee2";
    redPoints.header.frame_id = "bumblebee2";
    bluePoints.header.frame_id = "bumblebee2";
    //returnedPoints.header.stamp = ros::Time::now().toNSec();
    whiteLinePcPublisher.publish(whiteLinePoints);
    redflagPcPublisher.publish(redPoints);
    blueflagPcPublisher.publish(bluePoints);
    whiteLinePoints.clear();


    wl_filter.displayCyan();
    wl_filter.displayThreshold();
    ros::spinOnce();
    loop_rate.sleep();
  }
  cout << "Shutting Down!" << endl;
  bb2.shutdown();
  return 0;
}
