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

#include <chrono>
#include <memory>

#include "druai_idl_test.h"
#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "DruaiNodeTemplates.h"
#include "druai_idl/msg/druai_msg.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public DruaiMinimalPublisherNode<druai_idl::msg::DruaiMsg>
   {
   public:
      template<typename DurationRepT = int64_t, typename DurationT = std::milli>
      MinimalPublisher(const std::string& NodeName, const std::string& Topic, const rclcpp::QoS& QOS,
            std::chrono::duration<DurationRepT, DurationT> Period)
            : DruaiMinimalPublisherNode(NodeName, Topic, QOS, Period), count_(0)
         {
         return;
         }

   protected:
      virtual void ComposeMessage(druai_idl::msg::DruaiMsg& Message) override
         {
 //        auto message = std_msgs::msg::String();
         Message.banner = "Hello, Druai!";
         Message.count = count_++;
         RCLCPP_INFO(get_logger(), "Publishing: %s %d", Message.banner.c_str(), Message.count);
         return;
         }

      int32_t count_;
   };

int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);

   auto NodePtr = std::make_shared<MinimalPublisher>("publisher", druai_idl_topic, 10, 500ms);

   RCLCPP_INFO(NodePtr->get_logger(), "Druai Publisher Test Started");

   rclcpp::spin(NodePtr);

   rclcpp::shutdown();

   return 0;

   }
