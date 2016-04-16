#include <flycapture/FlyCapture2.h>
#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <triclops/flycapture2bridge.h>
#include "usma_triclops/bumblebeecamera.h"
#include "usma_triclops/common.h"

BumbleBeeCamera::BumbleBeeCamera(){
    startCamera();
}

//BumbleBeeCamera::~BumbleBeeCamera()

int BumbleBeeCamera::startCamera()
{
    FC2::Error fc2Error;

    // Connect to the camera
    this->camera.Connect();

    FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;
    // Set the camera to stereo mode with cameras arranged horizontally
    FC2T::ErrorType fc2TriclopsError = FC2T::setStereoMode(this->camera, mode);
    if (fc2TriclopsError) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }

    // generate the Triclops context. This contains camera information needed to do stereo processing.
    if (generateTriclopsContext(triclops)) {
        exit(-1);
    }

    // Starts isochronous image capture with DROP_FRAMES enabled. Only the most recent image is kept in the buffer.
    fc2Error = this->camera.StartCapture();
    if (fc2Error != FC2::PGRERROR_OK) {
        exit(FC2T::handleFc2Error(fc2Error));
    }

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

int BumbleBeeCamera::retrieveImages(){
    FC2::Error fc2Error;
    FC2::Image grabbedImage;

    // this image contains both right and left images
    fc2Error = this->camera.RetrieveBuffer(&grabbedImage);
    if (fc2Error != FC2::PGRERROR_OK) {
        exit(FC2T::handleFc2Error(fc2Error));
    }

    // generate triclops inputs from grabbed image
    if (this->preProcessing(grabbedImage)) {
        exit(EXIT_FAILURE);
    }
    return 0;
}


// Checked against PGR Code OK. RR
//Low-pass filtering, Rectification, Edge detection
int BumbleBeeCamera::preProcessing(FC2::Image const& grabbedImage)
{
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError te;

    FC2::Image* unprocessedImage = imageContainer.unprocessed;

    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
        grabbedImage, true /*assume little endian*/, unprocessedImage[RIGHT],
        unprocessedImage[LEFT]);

    if (fc2TriclopsError != FC2T::ERRORTYPE_OK) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError,
            "unpackUnprocessedRawOrMono16Image");
    }

    FC2::Image* monoImage = imageContainer.mono;
    FC2::Image& packedColorImage = imageContainer.packed;

    // check if the unprocessed image is color
    if (unprocessedImage[RIGHT].GetBayerTileFormat() != FC2::NONE) {

        FC2::Image* bgruImage = imageContainer.bgru;
        for (int i = 0; i < 2; ++i) {
            if (convertToBGRU(unprocessedImage[i], bgruImage[i])) {
                return 1;
            }
        }

        // pack BGRU right and left image into an image
        fc2TriclopsError = FC2T::packTwoSideBySideRgbImage(
            bgruImage[RIGHT], bgruImage[LEFT], packedColorImage);
        if (fc2TriclopsError != FC2T::ERRORTYPE_OK) {
            return handleFc2TriclopsError(fc2TriclopsError,
                "packTwoSideBySideRgbImage");
        }

        TriclopsInput triclopsColorInput;
        // Use the row interleaved images to build up a packed TriclopsInput.
        // A packed triclops input will contain a single image with 32 bpp.
        te = triclopsBuildPackedTriclopsInput(
            static_cast<int>(grabbedImage.GetCols()),
            static_cast<int>(grabbedImage.GetRows()),
            static_cast<int>(packedColorImage.GetStride()),
            static_cast<unsigned long>(grabbedImage.GetTimeStamp().seconds),
            static_cast<unsigned long>(grabbedImage.GetTimeStamp().microSeconds),
            packedColorImage.GetData(), &triclopsColorInput);

        _HANDLE_TRICLOPS_ERROR("triclopsBuildPackedTriclopsInput()", te);

        // the following does not change the size of the image
        // and therefore it PRESERVES the internal buffer!
        packedColorImage.SetDimensions(
            packedColorImage.GetRows(), packedColorImage.GetCols(),
            packedColorImage.GetStride(), packedColorImage.GetPixelFormat(),
            FC2::NONE);

        for (int i = 0; i < 2; ++i) {
            fc2Error = bgruImage[i].Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage[i]);

            if (fc2Error != FlyCapture2::PGRERROR_OK) {
                return Fc2Triclops::handleFc2Error(fc2Error);
            }
        }
    }
    else {
        monoImage[RIGHT] = unprocessedImage[RIGHT];
        monoImage[LEFT] = unprocessedImage[LEFT];
    }
    TriclopsInput triclopsMonoInput;

    // Use the row interleaved images to build up an RGB TriclopsInput.
    // An RGB triclops input will contain the 3 raw images (1 from each camera).
    te = triclopsBuildRGBTriclopsInput(
        static_cast<int>(grabbedImage.GetCols()),
        static_cast<int>(grabbedImage.GetRows()),
        static_cast<int>(packedColorImage.GetStride()),
        static_cast<unsigned long>(grabbedImage.GetTimeStamp().seconds),
        static_cast<unsigned long>(grabbedImage.GetTimeStamp().microSeconds),
        monoImage[RIGHT].GetData(), monoImage[LEFT].GetData(),
        monoImage[LEFT].GetData(), &triclopsMonoInput);

    _HANDLE_TRICLOPS_ERROR("triclopsBuildRGBTriclopsInput()", te);

    return 0;
}

// Copied over from older files.RR
int BumbleBeeCamera::convertToBGRU(FC2::Image& image, FC2::Image& convertedImage)
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_BGRU, &convertedImage);
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

// Copied over from older files.
int BumbleBeeCamera::convertToBGR(FC2::Image& image, FC2::Image& convertedImage)
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_BGR, &convertedImage);
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}
