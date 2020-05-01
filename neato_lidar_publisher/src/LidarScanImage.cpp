#include "LidarScanImage.h"
#include <iostream>

/******************************************************************************
*
***  LidarScanImage::LidarScanImage
*
******************************************************************************/

LidarScanImage::LidarScanImage(int nWidth, int nHeight, float fMaxDistance)
   : m_Image(nWidth, nHeight, CV_8UC3), m_fMaxDistance{fMaxDistance}
   {
   m_Image = cv::Scalar(240, 240, 240);

   // Set the origin in the center of the image
   m_Origin.x = nWidth / 2;
   m_Origin.y = nHeight / 2;

   // Distance is in meters.  Add half a meter to the maximum distance specified
   m_fScale = std::min(nWidth, nHeight) / (2 * (fMaxDistance + 0.5f));

   DrawBackground();

   return;

   } // end of method LidarScanImage::LidarScanImage

/******************************************************************************
*
***  LidarScanImage::DrawBackground
*
******************************************************************************/

void LidarScanImage::DrawBackground()
   {
   // Draw the origin
   cv::circle(m_Image, m_Origin, static_cast<int>(0.1f * m_fScale), CV_RGB(255, 0, 0), cv::FILLED, 8, 0);
//   m_Image.CircleFilled(m_Origin, static_cast<int>(0.1f * m_fScale), CV_RGB(255, 0, 0));

   float fRadius = 1.0f;
   while (fRadius < (m_fMaxDistance + 0.1f))
      {
      cv::circle(m_Image, m_Origin, static_cast<int>(fRadius * m_fScale), CV_RGB(255, 0, 0), 1, 8, 0);
//      m_Image.Circle(m_Origin, static_cast<int>(fRadius * m_fScale), CV_RGB(255, 0, 0));
      fRadius += 1.0f;
      } // end while

   return;

   } // end of method LidarScanImage::DrawBackground

/******************************************************************************
*
***  LidarScanImage::DrawScanPoint
*
******************************************************************************/

void LidarScanImage::DrawScanPoint(float fAngle, float fDistance, cv::Scalar Color)
   {
   int nRadius = 3;

   fDistance *= m_fScale;

   cv::Point Pt;
   Pt.x = (m_Origin.x + static_cast<int>(fDistance * cos(fAngle)));
   Pt.y = (m_Origin.y - static_cast<int>(fDistance * sin(fAngle)));

   cv::circle(m_Image, Pt, nRadius, Color, 1, 8, 0);
//   m_Image.Circle(Pt, nRadius, Color);

   return;

   } // end of method LidarScanImage::DrawScanPoint

/******************************************************************************
*
***  LidarScanImage::SaveImage
*
******************************************************************************/

bool LidarScanImage::SaveImage() const
   {
#define _CRT_SECURE_NO_WARNINGS

   std::stringstream strBuf;

   std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
   std::time_t t_c = std::chrono::system_clock::to_time_t(now);
   strBuf << "Scan_" << std::put_time(std::localtime(&t_c), "%F %T");

   std::string strName = strBuf.str();

   // Put the name on the image
   cv::putText(m_Image, strName, cv::Point(10, 25), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0));

   // Replace the ':'s to keep Windows happy
   auto i = strName.find(":");
   while (i != std::string::npos)
      {
      strName[i] = '-';
      i = strName.find(":", i + 1);
      } // end while

   // Make an absolute directory.  Need to make this more OS friendly
   std::string strFile = "/home/brutusdev/ros2/druai_ws/" + strName + ".png";

   std::cout << "Saving Image: " << strFile << std::endl;

   return (cv::imwrite(strFile, m_Image));
//   return (m_Image.WriteImage(strName));

   } // end of method LidarScanImage::SaveImage
