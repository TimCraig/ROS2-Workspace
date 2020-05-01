#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "GetSurrealComm.h"
#include "DMath.h"


/*****

  class NeatoLidarPublisher

   Class for a ROS2 Node that handles communications with a GetSurreal board (GetSurreal.com) controlling a
   Neato LDS sensor.  The Node publishes standard ROS2 LaserScan messages from sensor_msgs/msg/laser_scan.hpp.

*****/

class NeatoLidarPublisher : public rclcpp::Node
   {
   public:
      using MSGTYPE = sensor_msgs::msg::LaserScan;
//      using MSGTYPEPTR = typename MSGTYPE::SharedPtr;

      NeatoLidarPublisher(
            int32_t nStartAngle = 0, int32_t nEndAngle = 359,
            const std::string strPort = "/dev/ttyACM0",
            float fMotorRPM = 300.0f,
            float fMinRange = 0.3f, float fMaxRange = 5.0,
            const std::string strFrameID = "laser",
            const rclcpp::QoS& QOS = 10,
            const std::string& strTopic = "laserscan",
            const std::string& strNodeName = "neato_lidar_publisher");
      ~NeatoLidarPublisher() = default;

      // Open the serial connetion to the GetSurreal board.  Retries the specified number of times before
      // giving up with a delay between each try.  (Maybe have nTries be -1 for infinite tries?)
      bool OpenNeato(int32_t nTries, std::chrono::milliseconds RetryTime = std::chrono::milliseconds(1000))
         {
         return (m_Neato.Open(nTries, RetryTime));
         }

      void Run();

   protected:
      // Structure with the text data from the GetSurreal board decoded into numeric values.
      struct ScanPt
         {
         int32_t nAngle;
         float fDistance;
         float fIntensity;
         bool bValid;
         }; // end struct ScanPt

      GetSurrealComm m_Neato;

      int32_t m_nStartAngle;
      int32_t m_nEndAngle;

      int32_t m_nActualStartAngle;
      int32_t m_nActualEndAngle;

      rclcpp::Publisher<MSGTYPE>::SharedPtr m_pPublisher;
      MSGTYPE m_ScanMsg;

      bool InitializeMessage(float fMinRange, float fMaxRange, float fMotorRPM, const std::string strFrameID);
      bool StartScaning();
      ScanPt ReadScan();

   private:
   }; // end class NeatoLidarPublisher

/****

  NeatoLidarPublisher::NeatoLidarPublisher

****/

NeatoLidarPublisher::NeatoLidarPublisher(
      int32_t nStartAngle /* = 0 */, int32_t nEndAngle /*= 359 */,
      const std::string strPort /* = "/dev/ttyACM0" */,
      float fMotorRPM /* = 300.0f */,
      float fMinRange /* = 0.5f */, float fMaxRange /* = 5.0 */,
      const std::string strFrameID /* = "laser" */,
      const rclcpp::QoS& QOS /* = 10 */,
      const std::string& strTopic /* = "laserscan" */,
      const std::string& strNodeName /* = "neato_lidar_publisher" */
      )
   : Node(strNodeName),
      m_Neato(strPort),
      m_nStartAngle{nStartAngle},
      m_nEndAngle{nEndAngle}
   {
   m_pPublisher = create_publisher<MSGTYPE>(strTopic, QOS);

   if (!InitializeMessage(fMinRange, fMaxRange, fMotorRPM, strFrameID))
      {
      RCLCPP_ERROR(get_logger(), "NeatoLidarPublisher failed in InitializeMessage()");
      // Throw soemthing
      } // end if

   return;

   } // end constructor NeatoLidarPublisher::NeatoLidarPublisher

/****

  NeatoLidarPublisher::InitializeMessage

  The message structure will be reused but some of the data never changes so initialize it once.  The current version
  has this fixed with no possible updates while running.

****/

bool NeatoLidarPublisher::InitializeMessage(float fMinRange, float fMaxRange, float fMotorRPM,
      const std::string strFrameID)
   {
   bool bRet = true;

   m_ScanMsg.header.frame_id = strFrameID;

   m_ScanMsg.angle_min = DegToRad(static_cast<float>(m_nStartAngle));
   m_ScanMsg.angle_max = DegToRad(static_cast<float>(m_nEndAngle));

   m_nActualStartAngle = (m_nStartAngle < 0) ? (m_nStartAngle) + 360 : m_nStartAngle;
   m_nActualEndAngle = (m_nEndAngle < 0) ? (m_nEndAngle) + 360 : m_nEndAngle;

   m_ScanMsg.angle_increment = DegToRad(1.0f);  // Neato produces a scan point at 1 degree increments
   m_ScanMsg.range_min = fMinRange;
   m_ScanMsg.range_max = fMaxRange;
   m_ScanMsg.scan_time = 60.0f / fMotorRPM;
   m_ScanMsg.time_increment = m_ScanMsg.scan_time / 360.0f;

   std::vector<float>::size_type nScanPts = static_cast<std::vector<float>::size_type>(m_nEndAngle - m_nStartAngle + 1);
   m_ScanMsg.ranges.resize(nScanPts);
   m_ScanMsg.intensities.resize(nScanPts);

   return (bRet);

   } // end member function NeatoLidarPublisher::InitializeMessage

/****

  bool NeatoLidarPublisher::StartScaning

  Set up the GetSurreal board to produce a data record for each angle either with valid or invalid data.  The LDS
  rotates counter clockwise, positive Z rotation, with a point for each angle from 0, in the center, to 359.

****/

bool NeatoLidarPublisher::StartScaning()
   {
   bool bRet = m_Neato.HideDist() && m_Neato.ShowErrors() && m_Neato.ShowDist();

   return (bRet);

   } // end member function bool NeatoLidarPublisher::StartScaning

/****

  NeatoLidarPublisher::Run

  This is where the data is read from the Neato LDS, processed, the scan message built, and published.  Once started
  it's an endless loop until the program is signaled to stop.

****/

void NeatoLidarPublisher::Run()
   {
   static int nScan = 0;

   std::vector<float>::size_type nScanIndex = 0;
   bool bCapture = false;

   if (!StartScaning())
      {
      RCLCPP_ERROR(get_logger(), "NeatoLidarPublisher failed in StartScanning()");
      return;
      } // end if

   // Loop until the program is stopped
   while (rclcpp::ok())
      {
      ScanPt Pt = ReadScan();

      // When the beginning angle is reached, set the time stamp and start capturing scan points
      if (Pt.bValid && (Pt.nAngle == m_nActualStartAngle))
         {
         bCapture = true;
         nScanIndex = 0;
         m_ScanMsg.header.stamp = rclcpp::Node::now();
         } // end if

      // Capture points in the angular range and add to the scan
      if (Pt.bValid && bCapture)
         {
         m_ScanMsg.ranges[nScanIndex] = Pt.fDistance;
         m_ScanMsg.intensities[nScanIndex] = Pt.fIntensity;
         nScanIndex++;

         // If the end angle is reached publish the scan
         if (Pt.nAngle == m_nActualEndAngle)
            {
            m_pPublisher->publish(m_ScanMsg);
            bCapture = false;
            std::cout << "Published Scan Message: " << nScan++ << std::endl;
            } // end if
         } // end if
      } // end while

   // Not sure this is necessary but no one is listening
   m_Neato.HideDist();

   m_Neato.Close();

   return;

   } // end member function NeatoLidarPublisher::Run

/****

  NeatoLidarPublisher::ReadScan

****/

NeatoLidarPublisher::ScanPt NeatoLidarPublisher::ReadScan()
   {
   ScanPt Pt;
   Pt.bValid = false;

   std::string strScanPoint = m_Neato.ReadLine();
//   std::cout << strScanPoint << std::endl;
   std::vector<std::string> Fields = m_Neato.ParseScanPoint(strScanPoint);

   if ((Fields.size() < 3) || (Fields.size() > 4))
      {
      std::cout << "Invalid number of Fields: " << Fields.size() << std::endl;
      } // end if
   else if (Fields[0] != "A")
      {
      std::cout << "Invalid Start Character: " << Fields[0] << std::endl;
      }  // end else if
   else if (Fields.size() == 3)
      {
      Pt.nAngle = std::stoi(Fields[1]);
      Pt.fDistance = m_ScanMsg.range_max + 1.0f;
      Pt.fIntensity =  0.0f;
      Pt.bValid = true;

//      std::cout << "Scan error: Angle = " << Pt.nAngle << " Error: " << Fields[2] << std::endl;
      }  // end else if
   else
      {
      Pt.nAngle = std::stoi(Fields[1]);
      Pt.fDistance = std::stof(Fields[2]) / 1000.0f;
      Pt.fIntensity =  std::stof(Fields[3]);
      Pt.bValid = true;

//      std::cout << "Angle: " << Pt.nAngle << " Distance (m): " << Pt.fDistance << " Intensity: " << Pt.fIntensity << std::endl;

      } // end else

   return (Pt);

   } // end member function NeatoLidarPublisher::ReadScan

int main(int argc, char* argv[])
   {
   int nRetVal = 0;

   rclcpp::init(argc, argv);

   try
      {
      // Get scans from -120 to 120 degrees
      NeatoLidarPublisher Node(-120, 120);
      if (Node.OpenNeato(5))
         {
         Node.Run();
         } // end if
      } // end try
   catch (boost::system::system_error& e)
      {
      std::stringstream strError;
      strError << "neato_lidar_publisher exited from error: " << e.what();

      std::cout << strError.str() << std::endl;

      nRetVal = 1;
      } // end catch

   rclcpp::shutdown();

   return (nRetVal);
   }
