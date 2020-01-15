// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <inttypes.h>
#include <memory>
#include <vector>
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "DruaiNodeTemplates.h"

class MinimalActionServer : public DruaiSimpleActionServerNode<example_interfaces::action::Fibonacci>
   {
   public:
      explicit MinimalActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : DruaiSimpleActionServerNode("minimal_action_server", "fibonacci", options)
         {
         return;
         }

   private:

      virtual void Execute(const std::shared_ptr<GOALHANDLEACTION> pGoalHandle) override
         {
         rclcpp::Rate loop_rate(1);
         const auto pGoal = pGoalHandle->get_goal();
               RCLCPP_INFO(this->get_logger(), "Executing pGoal: order = %d", pGoal->order);
         auto pFeedback = std::make_shared<ACTION::Feedback>();
         auto& Sequence = pFeedback->sequence;

         Sequence.push_back(0);
         Sequence.push_back(1);
         auto pResult = std::make_shared<ACTION::Result>();

         for (auto i = 1 ; (i < pGoal->order) && rclcpp::ok()  ; ++i)
            {
            // Check if there is a cancel request
            if (pGoalHandle->is_canceling())
               {
               pResult->sequence = Sequence;
               pGoalHandle->canceled(pResult);
               RCLCPP_INFO(get_logger(), "Goal Canceled");
               return;
               }

            // Update sequence
            auto next = Sequence[i] + Sequence[i - 1];
            Sequence.push_back(next);

            // Publish feedback
            pGoalHandle->publish_feedback(pFeedback);
            RCLCPP_INFO(get_logger(), "Publish Feedback %" PRId32, next);

            loop_rate.sleep();
            }

         // Check if Goal is done
         if (rclcpp::ok())
            {
            pResult->sequence = Sequence;
            pGoalHandle->succeed(pResult);
            RCLCPP_INFO(get_logger(), "Goal Succeeded");
            }

         return;
         }

   };  // class MinimalActionServer

int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);

   auto pActionServer = std::make_shared<MinimalActionServer>();

   rclcpp::spin(pActionServer);

   rclcpp::shutdown();

   return 0;
   }
