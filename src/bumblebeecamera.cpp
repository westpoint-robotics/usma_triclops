#include <flycapture/FlyCapture2.h>
#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <triclops/flycapture2bridge.h>
#include "usma_triclops/bumblebeecamera.h"
#include "usma_triclops/common.h"

BumbleBeeCamera::BumbleBeeCamera(){
    startCamera();
}

//BumbleBeeCamera::~BumbleBeeCamera() { shutdown(); }

int BumbleBeeCamera::startCamera()
{
    FC2::Error fc2Error;

    // Connect to the camera
    this->camera.Connect();
    if (configureCamera(this->camera)) {
        exit(-1);
    }

    FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;
    // Set the camera to stereo mode with cameras arranged horizontally
    FC2T::ErrorType fc2TriclopsError = FC2T::setStereoMode(this->camera, mode);
    if (fc2TriclopsError) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }

    // generate the Triclops context. This contains camera information needed to do stereo processing.
    if (generateTriclopsContext()) {
        exit(-1);
    }

    // Starts isochronous image capture with DROP_FRAMES enabled. Only the most recent image is kept in the buffer.
    fc2Error = this->camera.StartCapture();
    if (fc2Error != FC2::PGRERROR_OK) {
        exit(FC2T::handleFc2Error(fc2Error));
    }
   triclops = generateTriclopsContext();

    return 0;
}

int BumbleBeeCamera::shutdown()
{
    this->camera.StopCapture();
    this->camera.Disconnect();
    // Destroy the Triclops context
    TriclopsError te;
    te = triclopsDestroyContext(triclops);
    _HANDLE_TRICLOPS_ERROR("triclopsDestroyContext()", te);
    return 0;
}

// Retreive the image format setting from the camera to include:
// mode,offX,offY,width,height,pixFormat
int BumbleBeeCamera::retreiveImageFormat(FC2::Format7ImageSettings formatSettings)
{
    FC2::Error fc2Error;
    unsigned int packetSize;
    float percent;

    fc2Error = this->camera.GetFormat7Configuration(&formatSettings, &packetSize, &percent);
    if (fc2Error != FlyCapture2::PGRERROR_OK) {
        return Fc2Triclops::handleFc2Error(fc2Error);
    }
    return 0;
}

// Generates the triclopsContext from the camera.
int BumbleBeeCamera::generateTriclopsContext(TriclopsContext triclopsCon)
{
    FC2::CameraInfo camInfo;
    FC2::Error fc2Error = this->camera.GetCameraInfo(&camInfo);
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    FC2T::ErrorType fc2TriclopsError;
    fc2TriclopsError = FC2T::getContextFromCamera(camInfo.serialNumber, &triclopsCon);
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError,
            "getContextFromCamera");
    }

    return 0;
}
