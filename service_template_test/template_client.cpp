#include <chrono>
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "DruaiNodeTemplates.h"
#include "druai_idl/srv/druai_srv.hpp"

using DruaiSrv = druai_idl::srv::DruaiSrv;

/*****

  class ClientTest

  Derived class to provice a concrete implemention of a service client.

  The Multiply() member funcion is provided to provide a simple API to call the service and insulate the user
  from the details of dealing with the details of the Request and Response structures. It would be tailored for the
  particular service.

*****/

class ClientTest : public DruaiSimpleClientNode<DruaiSrv>
   {
   public:
      ClientTest(const std::string& NodeName, const std::string& ServiceName,
            const rmw_qos_profile_t& QOS = rmw_qos_profile_services_default,
            rclcpp::callback_group::CallbackGroup::SharedPtr pGroup = nullptr)
            : DruaiSimpleClientNode(NodeName, ServiceName, QOS, pGroup)
         {
         return;
         }

      /*
       * Just a dumb helper function to demonstate simplying the call process.  Needs a real way to notify
       * the caller if the service call fails.
       */

      int32_t Multiply(int32_t a, int32_t b)
         {
        // Create and initialize the service request structure (specified in the .srv file)
         auto pRequest = std::make_shared<DruaiSrv::Request>();
         pRequest->a = a;
         pRequest->b = b;

         auto pResult = CallService(pRequest);

         int32_t product = 0;
         if (pResult != nullptr)
            {
            product = pResult->product;
            }

         return (product);
         }

   protected:

   private:
   }; // end class ClientTest


int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);

   // Create the node
   auto pNode = std::make_shared<ClientTest>("client", "druai_srv");

   // Make a call to the server
   int32_t a = 10;
   int32_t b = 20;
   RCLCPP_INFO(pNode->get_logger(), "result of %d * %d = %d",
         a, b, pNode->Multiply(a, b));

   rclcpp::shutdown();

   return (0);
   }
