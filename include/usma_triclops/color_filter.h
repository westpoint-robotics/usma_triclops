#ifndef COLOR_FILTER_H
#define COLOR_FILTER_H


class Color_Filter
{
public:
    Color_Filter();
    cv::Mat findRed(const cv::Mat &src_image);
    cv::Mat findBlue(const cv::Mat &src_image);
    void filterControl();
    cv::Mat findBlueHsv(const cv::Mat &src_image);
    cv::Mat findRedHsv(const cv::Mat &src_image);
    cv::Mat findHsv(const cv::Mat &src_image, int lowH, int lowS, int lowV, int highH, int highS, int highV);
    cv::Mat findWhiteHsv(const cv::Mat &src_image);
    cv::Mat findContorllerHsv(const cv::Mat &src_image);
private:
    int iLowH;
    int iLowS;
    int iLowV;
    int iHighH;
    int iHighS;
    int iHighV;

};

#endif // COLOR_FILTER_H
