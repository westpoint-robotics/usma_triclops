#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <flycapture/FlyCapture2.h>
#include <triclops/flycapture2bridge.h>
#include "usma_triclops/bumblebeecamera.h"

BumbleBeeCamera::BumbleBeeCamera()
{
    FC2::Error fc2Error;

    this->camera.Connect();
    if (configureCamera(this->camera)) {
        exit(-1);
    }

    FC2::Format7ImageSettings formatSettings;
    unsigned int packetSize;
    float percent;
    fc2Error = this->camera.GetFormat7Configuration(&formatSettings, &packetSize, &percent);
    printf("mode,offX,offY,width,height,pixFormat,percent: %d,%d,%d,%d,%d,%x,%f\n",
        formatSettings.mode, formatSettings.offsetX, formatSettings.offsetY,
        formatSettings.width, formatSettings.height,
        formatSettings.pixelFormat, percent);

}

// Set the camera to stereo mode with two cameras aligned horizontally
int BumbleBeeCamera::configureCamera(FC2::Camera& camera)
{
    FC2T::ErrorType fc2TriclopsError;
    FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;

    fc2TriclopsError = FC2T::setStereoMode(camera, mode);
    if (fc2TriclopsError) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }
    return 0;
}
