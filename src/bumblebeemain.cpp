#include <iostream>
#include "ros/ros.h"
#include "usma_triclops/bumblebeecamera.h"


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


      ros::spinOnce();
      loop_rate.sleep();
    }
    cout << "Shutting Down!" << endl;
    bb2.shutdown();
    return 0;
}
