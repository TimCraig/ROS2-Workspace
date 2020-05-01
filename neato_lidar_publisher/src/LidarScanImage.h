#ifndef LIDARSCANIMAGE_H
#define LIDARSCANIMAGE_H

#pragma once

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include "DMath.h"
#include <chrono>

/*****************************************************************************
*
***  class LidarScanImage
*
*****************************************************************************/

class LidarScanImage
   {
   public:
      LidarScanImage(int nWidth, int nHeight, float fMaxDistance);

      LidarScanImage(const LidarScanImage& src) = delete;
      LidarScanImage(LidarScanImage&& src) = delete;

      ~LidarScanImage() = default;

      LidarScanImage& operator=(const LidarScanImage& rhs) = delete;
      LidarScanImage& operator=(LidarScanImage&& rhs) = delete;

      void DrawScanPoint(float fAngle, float fDistance, cv::Scalar Color = CV_RGB(0, 0, 255));
      bool SaveImage() const;

   protected:
      cv::Mat m_Image;
      cv::Point m_Origin;
      float m_fMaxDistance;
      float m_fScale;
      void DrawBackground();

   private:

   }; // end of class LidarScanImage

#endif // LIDARSCANIMAGE_H
