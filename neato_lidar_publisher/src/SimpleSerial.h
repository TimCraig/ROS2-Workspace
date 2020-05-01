/*
 * File:   SimpleSerial.h
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 *
 * Created on September 10, 2009, 12:12 PM
 */

#ifndef _SIMPLESERIAL_H
#define	_SIMPLESERIAL_H

#pragma once

#include <boost/asio.hpp>

class SimpleSerial
   {
   public:
      /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
      SimpleSerial(std::string port, unsigned int baud_rate)
         : io(), serial(io,port)
         {
         serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
         }

      SimpleSerial(const SimpleSerial& src) = delete;
      SimpleSerial(SimpleSerial&& src) = delete;

      ~SimpleSerial() = default;

      SimpleSerial& operator=(const SimpleSerial& rhs) = delete;
      SimpleSerial& operator=(SimpleSerial&& rhs) = delete;

      /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
      void WriteString(std::string s)
         {
         boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
         }

      void Close()
         {
         serial.close();
         return;
         }

      /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
      std::string ReadLine()
         {
         //Reading data char by char, code is optimized for simplicity, not speed
         using namespace boost;
         char c;
         std::string result;
         for(;;)
            {
            asio::read(serial,asio::buffer(&c,1));
            switch(c)
               {
               case '\r':
                  break;
               case '\n':
                  return result;
               default:
                  result+=c;
               }
            }
         }

   protected:
      boost::asio::io_service io;
      boost::asio::serial_port serial;
   };

#endif	/* _SIMPLESERIAL_H */

