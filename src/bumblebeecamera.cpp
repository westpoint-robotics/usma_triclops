#include <flycapture/FlyCapture2.h>
#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <triclops/flycapture2bridge.h>
#include "usma_triclops/bumblebeecamera.h"
#include "usma_triclops/common.h"

BumbleBeeCamera::BumbleBeeCamera()
{
    disp_min = 0;
    disp_max = 70;
    disp_map_max = 255;
    disp_map_min = 0;
    disp_map_on = 0;
    stereo_mask = 11;
}

// Connec to the camera and prepare it to start streaming images.
int BumbleBeeCamera::startCamera()
{
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError;
    // Connect to the camera
    this->camera.Connect();

    FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;
    // Set the camera to stereo mode with cameras arranged horizontally
    fc2TriclopsError = FC2T::setStereoMode(this->camera, mode);
    if (fc2TriclopsError) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }

    // Starts isochronous image capture with DROP_FRAMES enabled. Only the most
    // recent image is kept in the buffer.
    fc2Error = this->camera.StartCapture();
    if (fc2Error != FC2::PGRERROR_OK) {
        exit(FC2T::handleFc2Error(fc2Error));
    }
    // Get the camera info and print it out
    FC2::CameraInfo camInfo;
    fc2Error = this->camera.GetCameraInfo(&camInfo);
    if (fc2Error != FC2::PGRERROR_OK) {
        printf("Failed to get camera info from camera\n");
        exit(-1);
    }
    else {
        printf(">>>>> CAMERA INFO  Vendor: %s     Model: %s     Serail#: %d  "
               "Resolution: %s\n",
            camInfo.vendorName, camInfo.modelName, camInfo.serialNumber,
            camInfo.sensorResolution);
    }

    // generate the Triclops context. This contains camera information needed to
    // do stereo processing.
    fc2TriclopsError = FC2T::getContextFromCamera(camInfo.serialNumber, &triclops);
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError,
            "getContextFromCamera");
    }
    //TODO Find a better way or place for to move triclops context
    char fileName[] = "/home/user1/triclopsContextCurrent.txt";
    if (triclopsWriteCurrentContextToFile(this->triclops, fileName)) {
        exit(-1);
}
    return 0;
}

// Clean shutdown of the bumbleBee camera and triclops APIsave

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
int BumbleBeeCamera::retreiveImageFormat(
    FC2::Format7ImageSettings formatSettings)
{
    FC2::Error fc2Error;
    unsigned int packetSize;
    float percent;

    fc2Error = this->camera.GetFormat7Configuration(&formatSettings, &packetSize,
        &percent);
    if (fc2Error != FlyCapture2::PGRERROR_OK) {
        return Fc2Triclops::handleFc2Error(fc2Error);
    }
    return 0;
}

// Retreives raw images from the BumbleBee Cameras for
// pre-processing prior to stereo processing.
int BumbleBeeCamera::retrieveImages()
{
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

    // generate triclops inputs from grabbed image
    if (this->doStereo()) {
        exit(EXIT_FAILURE);
    }

    // TODO get the disparity and rectified images
    return 0;
}

// Low-pass filtering, Rectification, Edge detection. This method is
// going to update triclopsColorInput and triclopsMonoInput as part
// of pre-processing. The triclops inputs are then used by the
// doStereo method.
int BumbleBeeCamera::preProcessing(FC2::Image const& grabbedImage)
{
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError te;

    FC2::Image* unprocessedImage = imageContainer.unprocessed;

    // get the raw right and left image
    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
        grabbedImage, true /*assume little endian*/, unprocessedImage[RIGHT],
        unprocessedImage[LEFT]);
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError,
            "unpackUnprocessedRawOrMono16Image");
    }

    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    //    unprocessedImage[RIGHT].Save("rawRightImage.pgm", &pgmOpt);
    //    unprocessedImage[LEFT].Save("rawLeftImage.pgm", &pgmOpt);

    FC2::Image* monoImage = imageContainer.mono;
    FC2::Image& packedColorImage = imageContainer.packed;

    // Convert the image from BGR to BGRU
    FC2::Image* bgruImage = imageContainer.bgru;
    for (int i = 0; i < 2; ++i) {
        if (convertToBGRU(unprocessedImage[i], bgruImage[i])) {
            return 1;
        }
    }

    FC2::PNGOption pngOpt;
    pngOpt.interlaced = false;
    pngOpt.compressionLevel = 9;
    //        bgruImage[RIGHT].Save("colorImageRight.png", &pngOpt);
    //        bgruImage[LEFT].Save("colorImageLeft.png", &pngOpt);

    // Pack BGRU right and left image into one image
    fc2TriclopsError = FC2T::packTwoSideBySideRgbImage(
        bgruImage[RIGHT], bgruImage[LEFT], packedColorImage);
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK) {
        return handleFc2TriclopsError(fc2TriclopsError,
            "packTwoSideBySideRgbImage");
    }

    // Use the packed image to generate triclopsColorInput
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
    packedColorImage.SetDimensions(packedColorImage.GetRows(),
        packedColorImage.GetCols(),
        packedColorImage.GetStride(),
        packedColorImage.GetPixelFormat(), FC2::NONE);

    // Convert the BGRU images into mono8 images
    for (int i = 0; i < 2; ++i) {
        fc2Error = bgruImage[i].Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage[i]);
        if (fc2Error != FlyCapture2::PGRERROR_OK) {
            return Fc2Triclops::handleFc2Error(fc2Error);
        }
    }

    //        monoImage[RIGHT].Save("monoImageRight.pgm", &pgmOpt);
    //        monoImage[LEFT].Save("monoImageLeft.pgm", &pgmOpt);

    // Use the row interleaved images to build up an RGB TriclopsInput.
    // An RGB triclops input will contain the 3 raw images (1 from each camera).
    te = triclopsBuildRGBTriclopsInput(
        static_cast<int>(grabbedImage.GetCols()),
        static_cast<int>(grabbedImage.GetRows()),
        static_cast<int>(grabbedImage.GetCols()),
        // static_cast<int>(packedColorImage.GetStride()),
        static_cast<unsigned long>(grabbedImage.GetTimeStamp().seconds),
        static_cast<unsigned long>(grabbedImage.GetTimeStamp().microSeconds),
        monoImage[RIGHT].GetData(), monoImage[LEFT].GetData(),
        monoImage[LEFT].GetData(), &triclopsMonoInput);
    _HANDLE_TRICLOPS_ERROR("triclopsBuildRGBTriclopsInput()", te);

    // This method updates triclopsMonoInput and triclopColorInput state variables
    return 0;
}

// Conducts post processing of the images. Recieves triclopsColorInput,
// triclopsMonoInput, and triclops context. Produces the disparity image
// and the color rectified images.
int BumbleBeeCamera::doStereo()
{
    TriclopsError te;

    // Set subpixel interpolation oncd sr   cd to use
    // TriclopsImage16 structures when we access and save the disparity image
    te = triclopsSetSubpixelInterpolation(triclops, 1);
    _HANDLE_TRICLOPS_ERROR("triclopsSetSubpixelInterpolation()", te);

    te = triclopsSetDisparity(triclops, disp_min, disp_max);
    _HANDLE_TRICLOPS_ERROR("triclopsSetSubpixelInterpolation()", te);

    te = triclopsSetDisparityMapping(triclops, disp_map_min, disp_map_max);
    _HANDLE_TRICLOPS_ERROR("triclopsSetDisparityMapping()", te);

    te = triclopsSetDisparityMappingOn(triclops, disp_map_on);
    _HANDLE_TRICLOPS_ERROR("triclopsSetDisparityMappingOn()", te);

    te = triclopsSetStereoMask(triclops, stereo_mask);
    _HANDLE_TRICLOPS_ERROR("triclopsSetDisparityMappingOn()", te);

    // ROS_INFO("stereoData x,y: %d,%d",stereoData.nrows, stereoData.ncols);
    te = triclopsSetResolution(triclops, triclopsMonoInput.nrows,
        triclopsMonoInput.ncols);
    _HANDLE_TRICLOPS_ERROR("triclopsSetResolution()", te);

    // Rectify the images
    te = triclopsRectify(triclops,
        const_cast<TriclopsInput*>(&triclopsMonoInput));
    _HANDLE_TRICLOPS_ERROR("triclopsRectify()", te);

    // Do stereo processing
    te = triclopsStereo(triclops);
    _HANDLE_TRICLOPS_ERROR("triclopsStereo()", te);

    te = triclopsRectifyColorImage(triclops, TriCam_REFERENCE,
        &(this->triclopsColorInput),
        &(this->rectifiedColorImage));
    _HANDLE_TRICLOPS_ERROR("triclopsRectifyColorImage()", te);

    // Retrieve the interpolated depth image from the context
    te = triclopsGetImage16(triclops, TriImg16_DISPARITY, TriCam_REFERENCE,
        &disparityImage);
    _HANDLE_TRICLOPS_ERROR("triclopsGetImage()", te);

    // Save the interpolated depth image
    char const* pDispFilename = "disparity16.pgm";
    //    te = triclopsSaveImage16( &disparityImage, const_cast<char
    //    *>(pDispFilename) );
    //    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );

    return 0;
}

// Convert a Flycapture raw image into BGRU format for processing for stereo
// vision
int BumbleBeeCamera::convertToBGRU(FC2::Image& image,
    FC2::Image& convertedImage)
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
