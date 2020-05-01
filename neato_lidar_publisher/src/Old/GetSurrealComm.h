#ifndef GETSURREALCOMM_H
#define GETSURREALCOMM_H

#pragma once

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include "SimpleSerial.h"

/*****************************************************************************
*
***  class GetSurrealComm
*
* This class provides serial communication to a GetSurreal controller board
* for controlling a Neato Lidar unit.
*
*****************************************************************************/

class GetSurrealComm : public SimpleSerial
   {
   public:
      GetSurrealComm(std::string Port) : Base(Port, 115200)
         {
         return;
         }

      GetSurrealComm(const GetSurrealComm& src) = delete;
      GetSurrealComm(GetSurrealComm&& src) = delete;

      ~GetSurrealComm() = default;

      GetSurrealComm& operator=(const GetSurrealComm& rhs) = delete;
      GetSurrealComm& operator=(GetSurrealComm&& rhs) = delete;

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
      bool SendSimpleCommand(std::string strCommand, std::string strExpectedResponse);
      bool GetResponse(std::string strExpectedResponse);

   private:
      using Base = SimpleSerial;

   }; // end of class GetSurrealComm

#endif // GETSURREALCOMM_H
