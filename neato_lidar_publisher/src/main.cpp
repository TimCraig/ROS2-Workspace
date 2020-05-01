#include <iostream>
#include "GetSurrealComm.h"
#include "LidarScanImage.h"

using namespace std;

int main()
   {   
   try
      {
      GetSurrealComm Neato("COM3");
      if (Neato.HideDist() && Neato.ShowErrors())
         {
         if (Neato.ShowDist())
            {
            std::vector<cv::Scalar> Colors =
               {
               CV_RGB(0, 0, 255),
               CV_RGB(255, 255, 0),
               CV_RGB(255, 0, 255),
               CV_RGB(0, 255, 0),
               CV_RGB(0, 0, 0)
               };

            LidarScanImage Image(1000, 1000, 5.0f);

            for (auto& Color : Colors)
               {
               for (int i = 0 ; i < 360 ; i++)
                  {
                  std::string strScanPoint = Neato.ReadLine();
                  std::cout << strScanPoint << std::endl;

                  std::vector<std::string> Fields = Neato.ParseScanPoint(strScanPoint);

                  if ((Fields.size() < 3) || (Fields.size() > 4))
                     {
                     std::cout << "Invalid number of Fields: " << Fields.size();
                     } // end if
                  else if (Fields[0] != "A")
                     {
                     std::cout << "Invalid Start Character: " << Fields[0];
                     }  // end else if
                  else if (Fields.size() == 3)
                     {
                     int nAngle = std::stoi(Fields[1]);
                     std::cout << "Scan error: Angle = " << nAngle << " Error: " << Fields[2];
                     }  // end else if
                  else
                     {
                     int nAngle = std::stoi(Fields[1]);
                     float fDistance = std::stof(Fields[2]) / 1000.0f;
                     float fIntensity = std::stof(Fields[3]);
                     std::cout << "Angle: " << nAngle << " Distance (m): " << fDistance << " Intensity: " << fIntensity;

                     Image.DrawScanPoint(static_cast<float>(nAngle), fDistance, Color);
                     } // end else

                  std::cout << std::endl;
                  } // end for
               } // end for

            Image.SaveImage();
            } // end if

         Neato.HideDist();
         } // end if
      } // end try
   catch (boost::system::system_error& e)
      {
      cout<<"Error: " << e.what() << endl;
      return 1;
      }

   cout << "Exiting SerialTest Normally." << endl;

   return 0;
   }
