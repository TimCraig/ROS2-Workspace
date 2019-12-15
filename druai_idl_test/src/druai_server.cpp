//#include <inttypes.h>
#include <memory>
#include "druai_idl/srv/druai_srv.hpp"
#include "rclcpp/rclcpp.hpp"

using DruaiSrv = druai_idl::srv::DruaiSrv;
static rclcpp::Node::SharedPtr pNode = nullptr;

// Function for entry point to the service
void handle_service_pRequest(
      const std::shared_ptr<rmw_request_id_t> pRequestHeader,  // Unused here, good for something?
      const std::shared_ptr<DruaiSrv::Request> pRequest,  // Request structure passed in
      const std::shared_ptr<DruaiSrv::Response> pResponse)  // Response structure for returning results
   {
   (void) pRequestHeader;  // Avoid unused parameter warning?
   RCLCPP_INFO(pNode->get_logger(), "pRequest: %d * %d", pRequest->a, pRequest->b);
   pResponse->product = pRequest->a * pRequest->b;
   return;
   }

int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);

   // Create the node
   pNode = rclcpp::Node::make_shared("druai_service");

   // Create the server and pass the handler function
   auto server = pNode->create_service<DruaiSrv>("druai_srv", handle_service_pRequest);

   // Keep the node running
   rclcpp::spin(pNode);

   rclcpp::shutdown();

   // Precautionary?
   pNode = nullptr;

   return 0;
   }
