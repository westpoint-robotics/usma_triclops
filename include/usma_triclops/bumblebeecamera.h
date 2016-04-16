#ifndef BUMBLEBEECAMERA_H
#define BUMBLEBEECAMERA_H

#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

class BumbleBeeCamera
{

private:
    FC2T::TriclopsInput16 disparityImage;
    FC2T::TriclopsInput leftColorImage;
    FC2T::TriclopsInput rightColorImage;
    FC2T::TriclopsColorInput rectifiedColorImage;
    FC2::Camera camera;
    int configureCamera( FC2::Camera &camera );
    TriclopsContext triclops;
    int startCamera();
    int shutdown();
    int generateTriclopsContext(TriclopsContext triclopsCon);
    int retreiveImageFormat(FC2::Format7ImageSettings formatSettings);
public:
    BumbleBeeCamera();
    ~BumbleBeeCamera();
    FC2T::TriclopsInput16 getDisparityImage();
    FC2T::TriclopsInput getLeftColorImage();
    FC2T::TriclopsInput getRightColorImage();
    FC2T::TriclopsColorInput getRectifiedColorImage();

};

#endif // BUMBLEBEECAMERA_H
