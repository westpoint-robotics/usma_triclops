#ifndef VISION3D_H
#define VISION3D_H

class Vision3d {
public:
    Vision3d();
    int producePointCloud(const cv::Mat& disparityImage, const cv::Mat& maskImage, const TriclopsContext& triclops, PointCloud& returnedPoints);
};

#endif // VISION3D_H
