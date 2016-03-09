#ifndef CAMERA_SYSTEM_H
#define CAMERA_SYSTEM_H

#include <stdio.h>
#include <stdlib.h>

#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>

#include "triclops_vision/typedefs.h"
#include "triclops_vision/common.h"
#include "triclops_vision/line_filter.h"

class CameraSystem {
    public:
        CameraSystem(int argc, char** argv);
        // configue camera to capture image
        void run();
        // convert image to BRGU
        int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage );
        // convert image to BRG
        int convertToBGR( FC2::Image & image, FC2::Image & convertedImage );
        // generate triclops input necessary to carry out stereo processing
        int generateTriclopsInput( FC2::Image const & grabbedImage,
                                   ImageContainer   & imageContainer,
                                   TriclopsInput    & colorData,
                                   TriclopsInput    & stereoData);
        // generare triclops input
        int generateTriclopsInput( FC2::Image const & grabbedImage,
                                   ImageContainer   & imageCont,
                                   TriclopsInput    & triclopsInput );
        // carry out stereo processing pipeline
        int doStereo( TriclopsContext const & triclops,
                       TriclopsInput  const & stereoData,
                       TriclopsImage16      & depthImage );
        TriclopsContext triclops;
        
        private:
        // generate Triclops context from connected camera
        int generateTriclopsContext( FC2::Camera     & camera,
                                     TriclopsContext & triclops );
        // capture image from connected camera
        int grabImage ( FC2::Camera & camera, FC2::Image & grabbedImage );
            int configureCamera( FC2::Camera &camera );
            FC2::Camera camera;
            FC2::Image grabbedImage;
            TriclopsInput color;
            TriclopsInput mono;
            TriclopsImage16 disparityImageTriclops;
            cv::Mat disparityImageCV;
            image_transport::Publisher image_pub_left;
            image_transport::Publisher image_pub_right;
            image_transport::Publisher image_pub_disparity;
            
};

#endif // CAMERA_SYSTEM_H
