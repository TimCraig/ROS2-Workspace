#include <inttypes.h>
#include <memory>
#include "druai_idl/action/druai_action.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

class DruaiActionServer : public rclcpp::Node
   {
   public:
      using DruaiAction = druai_idl::action::DruaiAction;
      using GoalHandleDruai = rclcpp_action::ServerGoalHandle<DruaiAction>;

      explicit DruaiActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
         : Node("action_server", options)
         {
         using namespace std::placeholders;

         m_pActionServer = rclcpp_action::create_server<DruaiAction>(
                  get_node_base_interface(),
                  get_node_clock_interface(),
                  get_node_logging_interface(),
                  get_node_waitables_interface(),
                  "druai_action",
                  std::bind(&DruaiActionServer::HandleGoal, this, _1, _2),
                  std::bind(&DruaiActionServer::HandleCancel, this, _1),
                  std::bind(&DruaiActionServer::HandleAccepted, this, _1));
         }

   private:
      rclcpp_action::Server<DruaiAction>::SharedPtr m_pActionServer;

      rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID& /* uuid */,
            std::shared_ptr<const DruaiAction::Goal> pGoal)
         {
         RCLCPP_INFO(get_logger(), "Received goal request with start=%d step=%d end=%d",
               pGoal->start, pGoal->step, pGoal->end);

         // Just for fun and testing reject negative steps and zero steps
         if (pGoal->step <= 0)
            {
            return (rclcpp_action::GoalResponse::REJECT);
            }

         return (rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
         }

      rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleDruai> /* pGoalHandle */)
         {
         RCLCPP_INFO(get_logger(), "Received request to cancel goal");

         return (rclcpp_action::CancelResponse::ACCEPT);
         }

      void execute(const std::shared_ptr<GoalHandleDruai> pGoalHandle)
         {
         RCLCPP_INFO(get_logger(), "Executing goal");

         rclcpp::Rate loop_rate(1);

         const auto pGoal = pGoalHandle->get_goal();
         auto pFeedback = std::make_shared<DruaiAction::Feedback>();

         auto pResult = std::make_shared<DruaiAction::Result>();

         loop_rate.sleep();

         for (int i = pGoal->start ; rclcpp::ok() && (i <= pGoal->end) ; i += pGoal->step)
            {
            pFeedback->current = i;

            // Check if there is a cancel request
            if (pGoalHandle->is_canceling())
               {
               pResult->final = i;
               pGoalHandle->canceled(pResult);
               RCLCPP_INFO(get_logger(), "Goal Canceled final=%d", pResult->final);

               return;
               }

            // Publish feedback
            pGoalHandle->publish_feedback(pFeedback);
            RCLCPP_INFO(get_logger(), "Publish Feedback current=%d", pFeedback->current);

            loop_rate.sleep();
            }

         // Check if goal is done
         if (rclcpp::ok())
            {
            pResult->final = pFeedback->current;
            pGoalHandle->succeed(pResult);
            RCLCPP_INFO(get_logger(), "Goal Succeeded final=%d", pResult->final);
            }

         return;

         }

      void HandleAccepted(const std::shared_ptr<GoalHandleDruai> pGoalHandle)
         {
         using namespace std::placeholders;

         // this needs to return quickly to avoid blocking the executor, so spin up a new thread
         std::thread{std::bind(&DruaiActionServer::execute, this, _1), pGoalHandle}.detach();

         return;

         }
   };  // class DruaiActionServer

int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);

   auto pActionServer = std::make_shared<DruaiActionServer>();

   rclcpp::spin(pActionServer);

   rclcpp::shutdown();

   return 0;
   }
