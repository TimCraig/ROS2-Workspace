#include <chrono>
#include <cinttypes>
#include <memory>
#include "druai_idl/srv/druai_srv.hpp"
#include "rclcpp/rclcpp.hpp"

using DruaiSrv = druai_idl::srv::DruaiSrv;

int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);

   // Create the node
   auto pNode = rclcpp::Node::make_shared("client");

   // Create a client for the specified service.
   auto client = pNode->create_client<DruaiSrv>("druai_srv");

   // See if the service is running and wait forever or until shutdown for it to appear
   while (!client->wait_for_service(std::chrono::seconds(1)))
      {
      if (!rclcpp::ok())
         {
         RCLCPP_ERROR(pNode->get_logger(), "client interrupted while waiting for service to appear.");
         return 1;
         }
      RCLCPP_INFO(pNode->get_logger(), "waiting for service to appear...");
      }

   // Create and initialize the service request structure (specified in the .srv file)
   auto pRequest = std::make_shared<DruaiSrv::Request>();
   pRequest->a = 10;
   pRequest->b = 20;

   // Make an asynchronous call to the server which returns a future (C++ std::future?)
   auto result_future = client->async_send_request(pRequest);

   // Check to see if this node can continue to process checking the future without calling this
   if (rclcpp::spin_until_future_complete(pNode, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
      {
      RCLCPP_ERROR(pNode->get_logger(), "service call failed :(");
      return 1;
      }

   // Get the result (If this IS a std::future, get() can only be called once or an exception will be raised!)
   // result is a pointer to a Result structure defined in the .srv file
   auto pResult = result_future.get();
   RCLCPP_INFO(pNode->get_logger(), "result of %d * %d = %d",
         pRequest->a, pRequest->b, pResult->product);

   rclcpp::shutdown();

   return 0;
   }
