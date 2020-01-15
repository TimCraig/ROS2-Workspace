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
#include <string>
#include <iostream>
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "DruaiNodeTemplates.h"

class MinimalActionClient : public DruaiSimpleActionClientNode<example_interfaces::action::Fibonacci>
   {
   public:

      explicit MinimalActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
            : DruaiSimpleActionClientNode("minimal_action_client", "fibonacci", node_options)
         {
         if (m_pClient == nullptr)
            {
            RCLCPP_ERROR(get_logger(), "Client failed to construct");
            throw;
            } // end if

         return;
         }

      /******
      * Provide a variation of SendGoal() that makes the goal paramaters obvious to the caller and hides the
      * details of initializing the goal structure.
      ******/

      void SendGoal(int32_t order)
         {
//         example_interfaces::action::Fibonacci::Goal GoalMsg;
         typename ACTION::Goal GoalMsg;
         GoalMsg.order = order;

         RCLCPP_INFO(get_logger(), "Sending Goal: order = %d", GoalMsg.order);

         DruaiSimpleActionClientNode::SendGoal(GoalMsg);

         return;
         }

   protected:
      /*****
      *  Override the template function to process the feedback from action server.
      *****/

      virtual void ProcessFeedback(typename GOALHANDLEACTION::SharedPtr /* pGoalHandle */,
            const std::shared_ptr<const typename ACTION::Feedback> pFeedback) override
         {
         RCLCPP_INFO(get_logger(),
               "Next number in sequence received: %" PRId32, pFeedback->sequence.back());

         return;
         }

      /****
      * Override the template funciton to process the result from the action server.
      ****/

      virtual void ProcessResult(const typename GOALHANDLEACTION::WrappedResult& Result) override
         {
         RCLCPP_INFO(get_logger(), "Result received");
         for (auto number : Result.result->sequence)
            {
            RCLCPP_INFO(get_logger(), "%" PRId32, number);
            }

         return;
         }
   private:

   };  // class MinimalActionClient

int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);
   auto action_client = std::make_shared<MinimalActionClient>();

   // Make sure the action server is running before trying to communicate
   if (action_client->IsServerRunning())
      {
      action_client->SendGoal(10);

      while (!action_client->IsGoalDone())
         {
         rclcpp::spin_some(action_client);
         }

      action_client->SendGoal(15);

      while (!action_client->IsGoalDone())
         {
         rclcpp::spin_some(action_client);
         }
      } // end if

   rclcpp::shutdown();

   return 0;
   }
