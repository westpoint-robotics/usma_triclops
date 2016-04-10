#include <stdio.h>
#include <stdlib.h>
#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "typedefs.h"


// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

/* ConvertTriclops2Opencv converts images from multiple Point Grey API image formats to an OpenCv Mat. This function is overloaded to handle both fly capture2 and triclops image formats.
*/
// convert a triclops color image to opencv mat

int convertTriclops2Opencv(FC2::Image & inImage,
                           cv::Mat & cvImage){
  // convert bgr image to OpenCV Mat
  unsigned int rowBytes = (double)inImage.GetReceivedDataSize()/(double)inImage.GetRows();
  cvImage = cv::Mat(inImage.GetRows(), inImage.GetCols(), CV_8UC3, inImage.GetData(),rowBytes);
  char numstr[50];
  sprintf(numstr, "rows: %d cols: %d rowbytes: %d", cvImage.rows,cvImage.cols, rowBytes);
  //putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}

// convert a triclops color image to opencv mat
int convertTriclops2Opencv(TriclopsInput & inImage,
                           cv::Mat & cvImage){
   //convert bgr image to OpenCV Mat
      cv::Mat R(inImage.nrows,inImage.ncols,CV_8UC1,inImage.u.rgb.red,inImage.rowinc);
      cv::Mat B(inImage.nrows,inImage.ncols,CV_8UC1,inImage.u.rgb.blue,inImage.rowinc);
      cv::Mat G(inImage.nrows,inImage.ncols,CV_8UC1,inImage.u.rgb.green,inImage.rowinc);
      std::vector<cv::Mat> array_to_merge;
      array_to_merge.push_back(B);
      array_to_merge.push_back(G);
      array_to_merge.push_back(R);
      cv::merge(array_to_merge,cvImage);
      char numstr[50];
      sprintf(numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows,cvImage.cols, inImage.rowinc);
      //putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}

// convert a triclops color image to opencv mat
int convertTriclops2Opencv(TriclopsImage & inImage,
                           cv::Mat & cvImage){
  cvImage = cv::Mat(inImage.nrows, inImage.ncols, CV_8UC3, inImage.data,inImage.rowinc);
  char numstr[50];
  sprintf(numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows,cvImage.cols, inImage.rowinc);
  //putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}

// convert a triclopsImage16 disparity image to opencv mat
int convertTriclops2Opencv(TriclopsImage16 & inImage,
                           cv::Mat & cvImage){
  cvImage = cv::Mat(inImage.nrows, inImage.ncols, CV_16UC1, inImage.data,inImage.rowinc);
  char numstr[50];
  sprintf(numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows,cvImage.cols, inImage.rowinc);
  //putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}


// convert a triclopscolorImage to opencv mat
int convertTriclops2Opencv(TriclopsColorImage & inImage,
                           cv::Mat & cvImage){
  //convert bgr image to OpenCV Mat
     cv::Mat R(inImage.nrows,inImage.ncols,CV_8UC1,inImage.red,inImage.rowinc);
     cv::Mat B(inImage.nrows,inImage.ncols,CV_8UC1,inImage.blue,inImage.rowinc);
     cv::Mat G(inImage.nrows,inImage.ncols,CV_8UC1,inImage.green,inImage.rowinc);
     std::vector<cv::Mat> array_to_merge;
     array_to_merge.push_back(B);
     array_to_merge.push_back(G);
     array_to_merge.push_back(R);
     cv::merge(array_to_merge,cvImage);
//     ROS_INFO("c %d r %d rInc: %d",cvImage.cols,cvImage.rows,inImage.rowinc);
     char numstr[50];
     sprintf(numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows,cvImage.cols, inImage.rowinc);
     //putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}

// convert an Opencv into a fly capture2 image
int convertOpencv2Triclops( cv::Mat & cvImage,

                             FC2::Image & outImage)
{\
  unsigned int imgStride = cvImage.step;
  int sizeImg = cvImage.rows*cvImage.cols*int(cvImage.elemSize());
  outImage.SetDimensions(cvImage.rows,cvImage.cols, imgStride,FC2::PIXEL_FORMAT_BGR, FC2::NONE);

}

// TODO This converter is broken DO NOT USE
int convertOpencv2Triclops( cv::Mat & cvImage,

                             TriclopsColorImage & outImage)
{\
  int from_to[] = {0,0};
  cv::Mat b(cvImage.rows,cvImage.cols,CV_8UC1);
  int bytes = sizeof(b.data);
  cv::mixChannels(&cvImage, 1, &b, 1, from_to, 1);
  from_to[0]= 1;
  cv::Mat g(cvImage.rows,cvImage.cols,CV_8UC1);
  cv::mixChannels(&cvImage, 1, &g, 1, from_to, 1);
  from_to[0]= 2;
  cv::Mat r(cvImage.rows,cvImage.cols,CV_8UC1);
  cv::mixChannels(&cvImage, 1, &r, 1, from_to, 1);
//  int bytes = r.elemSize();
//  unsigned int imgStride = cvImage.step;
  unsigned int imgStride = cvImage.cols;
  outImage.ncols=cvImage.cols;
  outImage.nrows=cvImage.rows;
  outImage.rowinc=imgStride;
  outImage.blue = b.data;
  outImage.green = g.data;
  outImage.red = r.data;// r is 76800 bytes long. sizeof r is 96 size of r.data is 8
  //ROS_INFO("c %d r %d stp: %d %zu sffride: %d",cvImage.cols,cvImage.rows,r.size().width,sizeof(b.data),imgStride);

}

