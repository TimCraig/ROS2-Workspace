#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "LidarScanImage.h"
#include "DruaiNodeTemplates.h"

class LaserScanSubscriber : public DruaiMinimalSubscriberNode<sensor_msgs::msg::LaserScan>
   {
   public:
      LaserScanSubscriber(const std::string& NodeName, const std::string& Topic, const rclcpp::QoS& QOS)
         : DruaiMinimalSubscriberNode(NodeName, Topic, QOS), m_Image{1000, 1000, 5.0f}
         {
         m_ScanColors =
            {
            CV_RGB(0, 0, 255),
            CV_RGB(255, 255, 0),
            CV_RGB(255, 0, 255),
            CV_RGB(0, 255, 0),
            CV_RGB(0, 0, 0)
            };

         return;
         }

   protected:
      LidarScanImage m_Image;
      std::vector<cv::Scalar> m_ScanColors;

      virtual void HandleMessage(const sensor_msgs::msg::LaserScan::SharedPtr pMsg) override
         {
         static size_t nScan = 0;

         if (nScan < m_ScanColors.size())
            {
            std::cout << "Processing Scan: " << nScan << std::endl;

            float fAngle = pMsg->angle_min;

            for (float fDistance : pMsg->ranges)
               {
               if (fDistance < pMsg->range_max)
                  {
                  m_Image.DrawScanPoint(fAngle, fDistance, m_ScanColors[nScan]);
                  } // end if

               fAngle += pMsg->angle_increment;
               } // end for

            nScan++;
            if (nScan == m_ScanColors.size())
               {
               if (!m_Image.SaveImage())
                  {
                  std::cout << "Image Save Failed" << std::endl;
                  } // end if
               } // end if
            } // end if

         return;
         }

   private:

   };

int main(int argc, char * argv[])
   {
   rclcpp::init(argc, argv);
   auto NodePtr = std::make_shared<LaserScanSubscriber>("NeatoLidarImage", "laserscan", 10);
   RCLCPP_INFO(NodePtr->get_logger(), "NeatoLidarImage Started");
   rclcpp::spin(NodePtr);
   rclcpp::shutdown();
   return 0;
   }
