/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include <iostream>

#include "GetSurrealComm.h"

/******************************************************************************
*
***  GetSurrealComm::ParseScanPoint
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
