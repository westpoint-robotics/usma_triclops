#ifndef COLOR_FILTER_H
#define COLOR_FILTER_H


class Color_Filter
{
public:
    Color_Filter();
    cv::Mat findRed(const cv::Mat &src_image);
    cv::Mat findBlue(const cv::Mat &src_image);
    cv::Mat findBlueBlog(const cv::Mat &src_image);
    void filterControl();
    cv::Mat findBlueHsv(const cv::Mat &src_image);
private:
    int minThreshold;
    int maxThreshold;
    int filterByArea;
    int minArea;
    int filterByCircularity;
    float minCircularity;
    int filterByConvexity;
    float minConvexity;
    int filterByInertia;
    int minInertiaRatio;
    int filterByColor;
    int blobColor;
    int iLowH;
    int iLowS;
    int iLowV;
    int iHighH;
    int iHighS;
    int iHighV;

};

#endif // COLOR_FILTER_H
