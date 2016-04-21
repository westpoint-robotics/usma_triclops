#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "usma_triclops/color_filter.h"

Color_Filter::Color_Filter()
{
    minThreshold = 10;
    maxThreshold = 200;
    filterByArea = true;
    minArea = 1500;
    filterByCircularity = false;
    minCircularity = 0.1;
    filterByConvexity = false;
    minConvexity = 0.87;
    filterByInertia = false;
    minInertiaRatio = 0.01;
    filterByColor=true;
    blobColor = 175;
    iLowH = 0;
    iLowS = 0;
    iLowV = 0;
    iHighH = 179;
    iHighS = 255;
    iHighV = 255;
}

cv::Mat Color_Filter::findBlueHsv(const cv::Mat &src_image) {
    // Find blue HSV 90:120, 175:255, 0:255
    // Find red HSV 0:30, 175:255, 0:255
    iLowH=90;
    iHighH=120;
    iLowS=175;
    iHighS=255;
    iLowV=0;
    iHighV=255;

    cv::Mat imgHSV;
    cv::cvtColor(src_image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    cv::Mat imgThresholded;

    inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    return imgThresholded;
}


cv::Mat Color_Filter::findRedHsv(const cv::Mat &src_image) {
    // Find blue HSV 90:120, 175:255, 0:255
    // Find red HSV 0:30, 175:255, 0:255
    iLowH=0;
    iHighH=30;
    iLowS=175;
    iHighS=255;
    iLowV=0;
    iHighV=255;
    cv::Mat imgHSV;
    cv::cvtColor(src_image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    cv::Mat imgThresholded;

    inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    return imgThresholded;
}

void Color_Filter::filterControl(){
    // Create control sliders that allow tunning of the parameters for line detection
    cv::namedWindow( "Color_Filter", CV_WINDOW_AUTOSIZE );
    cv::createTrackbar( "iLowH", "Color_Filter", &iLowH, 179 );
    cv::createTrackbar( "iHighH", "Color_Filter", &iHighH, 179 );
    cv::createTrackbar( "iLowS", "Color_Filter", &iLowS, 255 );
    cv::createTrackbar( "iHighS", "Color_Filter", &iHighS, 255 );
    cv::createTrackbar( "iLowV", "Color_Filter", &iLowV, 255 );
    cv::createTrackbar( "iHighV", "Color_Filter", &iHighV, 255 );
}

cv::Mat Color_Filter::findRed(const cv::Mat &src_image) {
    cv::Mat redMask;
    cv::inRange(src_image, cv::Scalar(0,0,50), cv::Scalar(50,50,255), redMask);
    return redMask;
}

cv::Mat Color_Filter::findBlue(const cv::Mat &src_image) {
    cv::Mat blueMask;
    cv::inRange(src_image, cv::Scalar(86,31,4), cv::Scalar(220,88,50), blueMask);
    return blueMask;
}

cv::Mat Color_Filter::findBlueBlog(const cv::Mat &src_image) {
    cv::SimpleBlobDetector::Params params;
    // Change thresholds
    params.minThreshold = minThreshold;
    params.maxThreshold = maxThreshold;
    params.filterByArea = true;
    params.minArea = minArea;
    params.filterByCircularity = false;
    params.minCircularity = 0.1;
    params.filterByConvexity = false;
    params.minConvexity = 0.87;
    params.filterByInertia = filterByInertia;
    params.minInertiaRatio = minInertiaRatio/100.0;
    params.filterByColor=true;
    params.blobColor = blobColor;

    cv::SimpleBlobDetector blueBlobDetector(params);
    // Detect blobs.
    std::vector<cv::KeyPoint> keypoints;
    blueBlobDetector.detect( src_image, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    cv::Mat im_with_keypoints;
    cv::drawKeypoints( src_image, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    return im_with_keypoints;

}

