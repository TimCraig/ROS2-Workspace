#ifndef GETSURREALCOMM_H
#define GETSURREALCOMM_H

#pragma once

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include <string>
#include <chrono>
#include <boost/asio.hpp>

/*****************************************************************************
*
***  class GetSurrealComm
*
* This class provides serial communication to a GetSurreal (GetSurreal.com) controller board
* for controlling a Neato Lidar unit.  Boost::asio is used to supply the serial communications.
* Some more work needs to be done to handle error situtions, especially timeouts.  So far in
* testing, when things go correctly, the node runs perfectly.  Some problems occasionally on startup but
* I currently suspect some of this might be because most testing has been done using Ubuntu running as a Guest
* in a VirtualBox on a Windows 10 Host.
*
*****************************************************************************/

class GetSurrealComm
   {
   public:
      GetSurrealComm(std::string strPort) : m_IO(), m_Serial(m_IO), m_strPort(strPort)
         {
         return;
         }

      GetSurrealComm(const GetSurrealComm& src) = delete;
      GetSurrealComm(GetSurrealComm&& src) = delete;

      ~GetSurrealComm() = default;

      GetSurrealComm& operator=(const GetSurrealComm& rhs) = delete;
      GetSurrealComm& operator=(GetSurrealComm&& rhs) = delete;

      bool Open(int32_t nTries, std::chrono::milliseconds RetryTime = std::chrono::milliseconds(1000));

      void Close()
         {
         m_Serial.close();
         return;
         }

      // Write a string out the port
      void WriteString(std::string strOut)
         {
         boost::asio::write(m_Serial, boost::asio::buffer(strOut.c_str(), strOut.size()));
         return;
         }

      // Read a newline, \n, terminated string.  The GetSurreal ends its text responses with \n.
      std::string ReadLine()
         {
         //Reading data char by char, code is optimized for simplicity, not speed
         using namespace boost;
         char c;
         std::string strResult;
         bool bRead = true;
         while (bRead)
            {
            asio::read(m_Serial, asio::buffer(&c, 1));
            switch (c)
               {
               case '\r':
                  break;

               case '\n':
                  bRead = false;
                  break;

               default:
                  strResult += c;
               } // end switch
            } // end while

         return (strResult);
         }

      // Send some of the GetSurreal commands and wait for a response
      bool ShowDist()
         {
         return (SendSimpleCommand("ShowDist\n", "Code,Angle,Distance(mm),Signal strength"));
         }

      bool HideDist()
         {
         return (SendSimpleCommand("HideDist\n", "Hiding Distance data"));
         }

      bool ShowErrors()
         {
         return (SendSimpleCommand("ShowErrors\n", "Showing errors"));
         }

      bool HideErrors()
         {
         return(SendSimpleCommand("HideErrors\n", "Hiding errors"));
         }

      void ProcessScans();
      std::vector<std::string> ParseScanPoint(std::string strScanPoint);

   protected:
      boost::asio::io_service m_IO;
      boost::asio::serial_port m_Serial;
      std::string m_strPort;  // Port name, "COMx" on Windows or "/dev/ttyxxxx" on Linux

      // Send a command to the board and wait for the correct response
      bool SendSimpleCommand(std::string strCommand, std::string strExpectedResponse);
      // Get a response from a command to the GetSurreal
      bool GetResponse(std::string strExpectedResponse);

   private:

   }; // end of class GetSurrealComm

#endif // GETSURREALCOMM_H
