// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "DruaiNodeTemplates.h"
#include "druai_idl/msg/druai_msg.hpp"
#include "druai_idl_test.h"

//using std::placeholders::_1;

class MinimalSubscriber : public DruaiMinimalSubscriberNode<druai_idl::msg::DruaiMsg>
   {
   public:
      MinimalSubscriber(const std::string& NodeName, const std::string& Topic, const rclcpp::QoS& QOS) : DruaiMinimalSubscriberNode(NodeName, Topic, QOS)
         {
         return;
         }

   protected:
      virtual void HandleMessage(const druai_idl::msg::DruaiMsg::SharedPtr pMsg) override
         {
         RCLCPP_INFO(get_logger(), "%s %d", pMsg->banner.c_str(), pMsg->count);
         return;
         }

   };

int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);

   auto NodePtr = std::make_shared<MinimalSubscriber>("subscriber", druai_idl_topic, 10);

   RCLCPP_INFO(NodePtr->get_logger(), "Subscriber Template Test Started");

   rclcpp::spin(NodePtr);

   rclcpp::shutdown();

   return 0;

   }

