//#include <inttypes.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "DruaiNodeTemplates.h"
#include "druai_idl/srv/druai_srv.hpp"

using DruaiService = druai_idl::srv::DruaiSrv;


// static rclcpp::Node::SharedPtr pNode = nullptr;

/*****

  class ServiceTest

  Dervived class to provide a concrete service node.  This serice takes two integers and returns the product of their
  multiplication.

*****/

class ServiceTest : public DruaiSimpleServerNode<DruaiService>
   {
   public:
      ServiceTest(const std::string& NodeName, const std::string& ServiceName,
            const rmw_qos_profile_t& QOS = rmw_qos_profile_services_default,
            rclcpp::callback_group::CallbackGroup::SharedPtr pGroup = nullptr)
            : DruaiSimpleServerNode(NodeName, ServiceName, QOS, pGroup)
         {
         return;
         }

   protected:

      /*
       * This member function is overridden to provide the actual service of this node
       * which is multiply two integers and return the product
       */

      virtual void Service(const std::shared_ptr<rmw_request_id_t> /* pRequestHeader*/,
            const std::shared_ptr<DruaiService::Request> pRequest,
            const std::shared_ptr<DruaiService::Response> pResponse) override
         {
         RCLCPP_INFO(get_logger(), "pRequest: %d * %d", pRequest->a, pRequest->b);
         pResponse->product = pRequest->a * pRequest->b;
         return;
         }

   private:
   }; // end class ServiceTest


int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);

   // Create the node -- Why static? The node pointer is in scope until the program exits  TTC
   static rclcpp::Node::SharedPtr pNode = std::make_shared<ServiceTest>("druai_service", "druai_srv");

   // Keep the node running
   rclcpp::spin(pNode);

   rclcpp::shutdown();

   // Precautionary?
   pNode = nullptr;

   return 0;
   }
