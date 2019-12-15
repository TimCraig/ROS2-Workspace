
#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>
#include "druai_idl/action/druai_action.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

class DruaiActionClient : public rclcpp::Node
   {
   public:
      using DruaiAction = druai_idl::action::DruaiAction;
      using GoalHandleDruai = rclcpp_action::ClientGoalHandle<DruaiAction>;

      explicit DruaiActionClient(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
            : Node("druai_action_client", node_options), m_bGoalDone(false)
         {
         m_pActionClient = rclcpp_action::create_client<DruaiAction>(
                  get_node_base_interface(),
                  get_node_graph_interface(),
                  get_node_logging_interface(),
                  get_node_waitables_interface(),
                  "druai_action");

         // Not sure why they do this timer thing to invoke SendGoal rather than just calling it?  TTC
         m_pTimer = create_wall_timer(
                  std::chrono::milliseconds(500),
                  std::bind(&DruaiActionClient::SendGoal, this));

         return;
         }

      bool IsGoalDone() const
         {
         return (m_bGoalDone);
         }

      void SendGoal()
         {
         using namespace std::placeholders;

         m_pTimer->cancel();

         m_bGoalDone = false;

         if (!m_pActionClient)
            {
            RCLCPP_ERROR(get_logger(), "Action client not initialized");
            }

         if (!m_pActionClient->wait_for_action_server(std::chrono::seconds(10)))
            {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            m_bGoalDone = true;

            return;
            }

         auto GoalMsg = DruaiAction::Goal();
         GoalMsg.start = 10;
         GoalMsg.step = 2;
         GoalMsg.end = 20;

         RCLCPP_INFO(get_logger(), "Sending goal: start=%d step=%d end=%d", GoalMsg.start, GoalMsg.step, GoalMsg.end);

         auto SendGoalOptions = rclcpp_action::Client<DruaiAction>::SendGoalOptions();
         SendGoalOptions.goal_response_callback =
               std::bind(&DruaiActionClient::GoalResponseCallback, this, _1);
         SendGoalOptions.feedback_callback =
               std::bind(&DruaiActionClient::FeedbackCallback, this, _1, _2);
         SendGoalOptions.result_callback =
               std::bind(&DruaiActionClient::ResultCallback, this, _1);
         auto GoalHandleFuture = m_pActionClient->async_send_goal(GoalMsg, SendGoalOptions);

         return;
         }

   private:
      rclcpp_action::Client<DruaiAction>::SharedPtr m_pActionClient;
      rclcpp::TimerBase::SharedPtr m_pTimer;
      bool m_bGoalDone;

      void GoalResponseCallback(std::shared_future<GoalHandleDruai::SharedPtr> Future)
         {
         auto pGoalHandle = Future.get();
         if (!pGoalHandle)
            {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
            }
         else
            {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            }

         return;

         }

      void FeedbackCallback(GoalHandleDruai::SharedPtr,
            const std::shared_ptr<const DruaiAction::Feedback> pFeedback)
         {
         RCLCPP_INFO(get_logger(), "Next number in sequence received: %d", pFeedback->current);

         return;

         }

      void ResultCallback(const GoalHandleDruai::WrappedResult& Result)
         {
         m_bGoalDone = true;
         switch (Result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
               RCLCPP_INFO(get_logger(), "Result Received: final=%d", Result.result->final);
               break;

            case rclcpp_action::ResultCode::ABORTED:
               RCLCPP_ERROR(get_logger(), "Goal was aborted");
               break;

            case rclcpp_action::ResultCode::CANCELED:
               RCLCPP_ERROR(get_logger(), "Goal was canceled");
               break;

            default:
               RCLCPP_ERROR(get_logger(), "Unknown result code");
               break;
            }

         return;

         }
   };  // class DruaiActionClient

int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);

   auto pClient = std::make_shared<DruaiActionClient>();

//   pClient->SendGoal();

   // Keep the node running until the goal is done
   while (!pClient->IsGoalDone())
      {
      rclcpp::spin_some(pClient);
      }

   rclcpp::shutdown();

   return 0;
   }
