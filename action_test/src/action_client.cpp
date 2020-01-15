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

class MinimalActionClient : public rclcpp::Node
   {
   public:
      using Fibonacci = example_interfaces::action::Fibonacci;
      using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

      explicit MinimalActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
         : Node("minimal_pActionClient", node_options), m_bGoalDone(false)
         {
         m_pClient = rclcpp_action::create_client<Fibonacci>(
                  get_node_base_interface(),
                  get_node_graph_interface(),
                  get_node_logging_interface(),
                  get_node_waitables_interface(),
                  "fibonacci");

         m_pTimer = create_wall_timer(
                  std::chrono::milliseconds(500),
                  std::bind(&MinimalActionClient::SendGoal, this));
         }

      bool IsGoalDone() const
         {
         return m_bGoalDone;
         }

      void SendGoal()
         {
         using namespace std::placeholders;

         m_pTimer->cancel();

         m_bGoalDone = false;

         if (!m_pClient)
            {
            RCLCPP_ERROR(get_logger(), "Action client not initialized");
            }

         if (!m_pClient->wait_for_action_server(std::chrono::seconds(10)))
            {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            m_bGoalDone = true;
            return;
            }

         //    auto GoalMsg = Fibonacci::Goal();
         Fibonacci::Goal GoalMsg;
         GoalMsg.order = 10;

         RCLCPP_INFO(get_logger(), "Sending goal");

         //    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
         typename rclcpp_action::Client<Fibonacci>::SendGoalOptions send_goal_options;
         send_goal_options.goal_response_callback =
               std::bind(&MinimalActionClient::GoalResponseCallback, this, _1);
         send_goal_options.feedback_callback =
               std::bind(&MinimalActionClient::FeedbackCallback, this, _1, _2);
         send_goal_options.result_callback =
               std::bind(&MinimalActionClient::ResultCallback, this, _1);
         auto goal_handle_future = m_pClient->async_send_goal(GoalMsg, send_goal_options);
         }

   private:
      rclcpp_action::Client<Fibonacci>::SharedPtr m_pClient;
      rclcpp::TimerBase::SharedPtr m_pTimer;
      bool m_bGoalDone;

      void GoalResponseCallback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
         {
         auto goal_handle = future.get();
         if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
            } else {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            }
         }

      void FeedbackCallback(GoalHandleFibonacci::SharedPtr,
            const std::shared_ptr<const Fibonacci::Feedback> pFeedback)
         {
         RCLCPP_INFO(get_logger(),
                  "Next number in sequence received: %" PRId32, pFeedback->sequence.back());
         }

      void ResultCallback(const GoalHandleFibonacci::WrappedResult& result)
         {
         m_bGoalDone = true;
         switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
               break;

            case rclcpp_action::ResultCode::ABORTED:
               RCLCPP_ERROR(get_logger(), "Goal was aborted");
               return;

            case rclcpp_action::ResultCode::CANCELED:
               RCLCPP_ERROR(get_logger(), "Goal was canceled");
               return;

            default:
               RCLCPP_ERROR(get_logger(), "Unknown result code");
               return;
            }

         RCLCPP_INFO(get_logger(), "Result received");
         for (auto number : result.result->sequence)
            {
            RCLCPP_INFO(get_logger(), "%" PRId32, number);
            }
         }
   };  // class MinimalActionClient

int main(int argc, char ** argv)
   {
   rclcpp::init(argc, argv);
   auto pActionClient = std::make_shared<MinimalActionClient>();

   while (!pActionClient->IsGoalDone())
      {
      rclcpp::spin_some(pActionClient);
      }

   rclcpp::shutdown();

   return 0;
   }
