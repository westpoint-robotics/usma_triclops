#ifndef BUMBLEBEECAMERA_H
#define BUMBLEBEECAMERA_H

#include <stdlib.h>
#include <flycapture/FlyCapture2.h>
#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include "usma_triclops/common.h"

// aliases namespaces
//namespace FC2 = FlyCapture2;
//namespace FC2T = Fc2Triclops;

class BumbleBeeCamera
{

private:
    TriclopsInput triclopsColorInput;
    TriclopsInput triclopsMonoInput;

    ImageContainer imageContainer;
    TriclopsImage16 disparityImage;
    TriclopsColorImage rectifiedColorImage;
    FC2::Camera camera;
    TriclopsContext triclops;
    int disp_max;
    int disp_min;
    u_char disp_map_max;
    u_char disp_map_min;
    int disp_map_on;
    int stereo_mask;
    int generateTriclopsContext();
    int retreiveImageFormat(FC2::Format7ImageSettings formatSettings);
    int preProcessing(const FC2::Image &grabbedImage);
    int convertToBGR(FC2::Image &image, FC2::Image &convertedImage);
    int convertToBGRU(FC2::Image &image, FC2::Image &convertedImage);

    int doStereo();
public:
    BumbleBeeCamera();
    int startCamera();
    int shutdown();
    TriclopsImage16 getDisparityImage();
    TriclopsColorImage getRectifiedColorImage();

    int retrieveImages();
};

#endif // BUMBLEBEECAMERA_H
