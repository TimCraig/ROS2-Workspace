/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include <iostream>
#include <thread>

#include "GetSurrealComm.h"

/****

  GetSurrealComm::Open

  Open the serial port to communicate with the GetSurreal Arduino board.  It will perform the specified
  number of tries before giving up with a wait period between tries.

****/

bool GetSurrealComm::Open(int32_t nTries, std::chrono::milliseconds RetryTime)
   {
   bool bRet = false;

   int32_t nTry = 0;
   while (!bRet && (nTry < nTries))
      {
      try
         {
         m_Serial.open(m_strPort);
         bRet = true;
         std::cout << "Opened port: " << m_strPort << std::endl;
         m_Serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
         } // end try
      catch (...)
         {
         nTry++;
         std::cout << "Open port " << m_strPort << "failed try #" << nTry << std::endl;
         std::this_thread::sleep_for(RetryTime);s
         } // end catch
      } // end while


   return (bRet);

   } // end member function GetSurrealComm::Open

/******************************************************************************
*
***  GetSurrealComm::ParseScanPoint
*
* The GetSurreal is setup to return a string for each angle point (0, 359) whether
* valid or invalid.  The fields are simple test separated by commas ','.
*
******************************************************************************/

std::vector<std::string> GetSurrealComm::ParseScanPoint(std::string strScanPoint)
   {
   std::vector<std::string> Result;

   std::string strField;
   for (auto c : strScanPoint)
      {
      if (c != ',')
         {
         strField += c;
         } // end if
      else
         {
         Result.push_back(strField);
         strField.clear();
         } // end else
      } // end for
   Result.push_back(strField);

   return (Result);

   } // end of method GetSurrealComm::ParseScanPoint

/******************************************************************************
*
***  GetSurrealComm::SendSimpleCommand
*
* Sends a text string as a command and waits for the expected response from the
* board.  Any data recieved while waiting that isn't the response is discarded.
*
******************************************************************************/

bool GetSurrealComm::SendSimpleCommand(std::string strCommand, std::string strExpectedResponse)
   {
   std::cout << "Sending Command: " << strCommand << std::endl;
   WriteString(strCommand);

   return (GetResponse(strExpectedResponse));

   } // end of method GetSurrealComm::SendSimpleCommand


/******************************************************************************
*
***  GetSurrealComm::GetResponse
*
* Get the response for a command sent to the board.  Data received other than
* the response is discarded.  Work needs to be done to handle hangs and never
* receiving the response.
*
******************************************************************************/

bool GetSurrealComm::GetResponse(std::string strExpectedResponse)
   {
   bool bRet = false;

   std::cout << "Waiting for response: " << strExpectedResponse << std::endl;
   do
      {
      std::string strResponse = ReadLine();
      std::cout << "Response: " << strResponse << std::endl;
      bRet = strResponse == strExpectedResponse;
      } while (!bRet);

   return (bRet);

   } // end of method GetSurrealComm::GetResponse
