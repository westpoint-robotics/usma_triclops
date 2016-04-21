#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "usma_triclops/color_filter.h"

Color_Filter::Color_Filter()
{
    iLowH = 0;
    iLowS = 0;
    iLowV = 0;
    iHighH = 179;
    iHighS = 250;
    iHighV = 255;
}

// Return a image filter that represents only the blue pixels
cv::Mat Color_Filter::findBlueHsv(const cv::Mat& src_image)
{
    // Find blue HSV 90:120, 175:255, 0:255
    int lowH = 90;
    int highH = 120;
    int lowS = 175;
    int highS = 255;
    int lowV = 0;
    int highV = 255;
    return findHsv(src_image, lowH, highH, lowS, highS, lowV, highV);
}

// Return a image filter that represents only the red pixels
cv::Mat Color_Filter::findRedHsv(const cv::Mat& src_image)
{
    // Find red HSV 0:30, 175:255, 0:255
    int lowH = 0;
    int highH = 30;
    int lowS = 175;
    int highS = 255;
    int lowV = 0;
    int highV = 255;
    return findHsv(src_image, lowH, highH, lowS, highS, lowV, highV);
}

// Return a image filter that represents only the white pixels
cv::Mat Color_Filter::findWhiteHsv(const cv::Mat& src_image)
{
    // Find red HSV 0:30, 175:255, 0:255
    int lowH = 0;
    int highH = 179;
    int lowS = 0;
    int highS = 255;
    int lowV = 250;
    int highV = 255;
    return findHsv(src_image, lowH, highH, lowS, highS, lowV, highV);
}

// Return a image filter that represents only the pixels choosen using the GUI slider bar values.
cv::Mat Color_Filter::findContorllerHsv(const cv::Mat& src_image)
{
    return findHsv(src_image, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
}

// Provide a GUI control with slidebars to set the HSV limits.
void Color_Filter::filterControl()
{
    // Create control sliders that allow tunning of the parameters for line detection
    cv::namedWindow("Color_Filter", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("iLowH", "Color_Filter", &iLowH, 179);
    cv::createTrackbar("iHighH", "Color_Filter", &iHighH, 179);
    cv::createTrackbar("iLowS", "Color_Filter", &iLowS, 255);
    cv::createTrackbar("iHighS", "Color_Filter", &iHighS, 255);
    cv::createTrackbar("iLowV", "Color_Filter", &iLowV, 255);
    cv::createTrackbar("iHighV", "Color_Filter", &iHighV, 255);
}

// Return a filtered image based on the supplied HSV limits.
cv::Mat Color_Filter::findHsv(const cv::Mat& src_image, int lowH, int highH, int lowS, int highS, int lowV, int highV)
{

    //Convert the captured frame from BGR to HSV
    cv::Mat imgHSV;
    cv::cvtColor(src_image, imgHSV, cv::COLOR_BGR2HSV);

    //Threshold the image
    cv::Mat imgThresholded;
    inRange(imgHSV, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), imgThresholded);

    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    //morphological closing (fill small holes in the foreground)
    cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    return imgThresholded;
}

// Return a filter based on RGB Color space of Red
cv::Mat Color_Filter::findRed(const cv::Mat& src_image)
{
    cv::Mat redMask;
    cv::inRange(src_image, cv::Scalar(0, 0, 50), cv::Scalar(50, 50, 255), redMask);
    return redMask;
}

// Return a filter based on RGB Color space of Blue
cv::Mat Color_Filter::findBlue(const cv::Mat& src_image)
{
    cv::Mat blueMask;
    cv::inRange(src_image, cv::Scalar(86, 31, 4), cv::Scalar(220, 88, 50), blueMask);
    return blueMask;
}
