#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "usma_triclops/vision_3d.h"
#include "usma_triclops/line_filter.h"
#include "usma_triclops/camera_system.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv,"usma_triclops");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    //printf("camera.run()\n");
    CameraSystem camera(argc, argv);
    //printf("line.run()\n");
    LineFilter linefilter(argc, argv);
    //printf("vision.run()\n");
    Vision3D vision3D(argc, argv);

    while (ros::ok())
    {
        try{
        camera.run();
        //printf("camera.run()\n");
        linefilter.run();
        //printf("linefilter.run()\n");
        vision3D.run();
        //printf("vision3D.run()\n");
        loop_rate.sleep();}
        catch (std::exception& e)
        {
            std::cerr << "Exception catched : " << e.what() << std::endl;
        }
    }
}
