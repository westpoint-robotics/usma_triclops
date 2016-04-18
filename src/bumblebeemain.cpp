#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "usma_triclops/bumblebeecamera.h"
#include "usma_triclops/triclops_opencv.h"

using namespace std;

int main(int argc, char *argv[])
{
      ros::init(argc, argv, "bumbleBee");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);
    BumbleBeeCamera bb2;
    bb2.startCamera();
  while (ros::ok())
  {
      bb2.retrieveImages();  // This uses 95% of cpu at 5hz
      TriclopsImage16 tri_disparityImage =bb2.getDisparityImage();
      TriclopsColorImage tri_rectifiedColorImage;
      tri_rectifiedColorImage=bb2.getRectifiedColorImage();
      tri_disparityImage=bb2.getDisparityImage();

      cv::Mat cv_rectifiedColorImage;
      cv::Mat cv_disparityImage;
      convertTriclops2Opencv(tri_rectifiedColorImage,cv_rectifiedColorImage);
      convertTriclops2Opencv(tri_disparityImage,cv_disparityImage);
      cv::imshow("RectifiedImage", cv_rectifiedColorImage);
      cv::imshow("DisparityImage", cv_disparityImage);
      cv::waitKey(3);
      //TODO take the opencv images and return a filtered image mask.

      ros::spinOnce();
      loop_rate.sleep();
    }
    cout << "Shutting Down!" << endl;
    bb2.shutdown();
    return 0;
}
