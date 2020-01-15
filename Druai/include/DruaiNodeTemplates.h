/********************************************************************************
********************************** DruaiNodeTemplates.h *************************
********************************************************************************/

/*
 *
 * This file contains templates to simplify writing Nodes in ROS2.
 *
 */

#pragma once

/********************************************************************************
********************************* I N C L U D E *********************************
********************************************************************************/

#include <memory>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/client_goal_handle.hpp"

/*****

   template class DruaiMinimalSubscriberNode

   MSGTYPE is the fully qualified Message Name.  It will be in the form
      package_name::msg::MessageName where MessageName is the name of the
      message defintion file (MessageName.msg).

*****/

template <typename MSGTYPE>
class DruaiMinimalSubscriberNode : public rclcpp::Node
         {
         public:
            DruaiMinimalSubscriberNode(const std::string& strNodeName, const std::string& strTopic,
                  const rclcpp::QoS& QOS)
                  : Node(strNodeName)
               {
               m_Subscription = create_subscription<MSGTYPE>(strTopic, QOS, std::bind(
                     &DruaiMinimalSubscriberNode::TopicCallback, this, std::placeholders::_1));
               return;
               }

            virtual ~DruaiMinimalSubscriberNode() = default;

         protected:
            typename rclcpp::Subscription<MSGTYPE>::SharedPtr m_Subscription;

            virtual void HandleMessage(const typename MSGTYPE::SharedPtr /* Message */) = 0;

            void TopicCallback(const typename MSGTYPE::SharedPtr msg)
               {
               HandleMessage(msg);
               return;
               }

   }; // end template class MinimalSubscriberNode

/*****

  template class DruaiMinimalPublisherNode

   MSGTYPE is the fully qualified Message Name.  It will be in the form
      package_name::msg::MessageName where MessageName is the name of the
      message defintion file (MessageName.msg).

*****/

template <typename MSGTYPE>
class DruaiMinimalPublisherNode : public rclcpp::Node
   {
   public:
      template<typename DurationRepT = int64_t, typename DurationT = std::milli>
      DruaiMinimalPublisherNode(const std::string& strNodeName, const std::string& strTopic, const rclcpp::QoS& QOS,
            std::chrono::duration<DurationRepT, DurationT> Period)
            : Node(strNodeName)
         {
         m_Publisher = create_publisher<MSGTYPE>(strTopic, QOS);
         m_Timer = create_wall_timer(Period, std::bind(&DruaiMinimalPublisherNode::TimerCallback, this));
         return;
         }

      virtual ~DruaiMinimalPublisherNode() = default;

   protected:
      rclcpp::TimerBase::SharedPtr m_Timer;
      typename rclcpp::Publisher<MSGTYPE>::SharedPtr m_Publisher;

      virtual void ComposeMessage(MSGTYPE& /* Message */) = 0;

      void TimerCallback()
         {
         MSGTYPE Message;
         ComposeMessage(Message);
         m_Publisher->publish(Message);
         return;
         }

   }; // end template class DruaiMinimalPublisherNode

/*****

  template class DruaiSimpleServerNode

*****/

template <typename SERVICE>
class DruaiSimpleServerNode : public rclcpp::Node
   {
   public:
      DruaiSimpleServerNode(const std::string& strNodeName, const std::string& strServiceName,
            const rmw_qos_profile_t& QOS = rmw_qos_profile_services_default,
            rclcpp::callback_group::CallbackGroup::SharedPtr pGroup = nullptr)
            : Node(strNodeName), m_strNodeName{strNodeName}
         {
         m_pService = create_service<SERVICE>(strServiceName,
               std::bind(&DruaiSimpleServerNode::HandleRequest, this,
               std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
               QOS, pGroup);

         return;
         }

      virtual ~DruaiSimpleServerNode() = default;

      const std::string& GetNodeName() const
         {
         return (m_strNodeName);
         }

      std::string GetServiceName() const
         {
         return (m_pService->get_name());
         }

   protected:
      std::string m_strNodeName;
      typename rclcpp::Service<SERVICE>::SharedPtr m_pService;

      void HandleRequest(const std::shared_ptr<rmw_request_id_t> pRequestHeader,
            const std::shared_ptr<typename SERVICE::Request> pRequest,
            const std::shared_ptr<typename SERVICE::Response> pResponse)
         {
         Service(pRequestHeader, pRequest, pResponse);
         return;
         }

      virtual void Service(const std::shared_ptr<rmw_request_id_t> /* pRequestHeader */,
            const std::shared_ptr<typename SERVICE::Request> /* pRequest */,
                           const std::shared_ptr<typename SERVICE::Response> /* pResponse */)
         {
         RCLCPP_INFO(get_logger(), "DruaiSimpleServerNode: Failed to Override the Service Member");
         return;
         }

   private:
   }; // end template class DruaiSimpleServerNode

/*****

  template class DruaiSimpleClientNode

*****/

template <typename SERVICE>
class DruaiSimpleClientNode : public rclcpp::Node
   {
   public:
      DruaiSimpleClientNode(const std::string& strNodeName, const std::string& strServiceName,
            const rmw_qos_profile_t& QOS = rmw_qos_profile_services_default,
            rclcpp::callback_group::CallbackGroup::SharedPtr pGroup = nullptr)
            : Node(strNodeName), m_strNodeName{strNodeName}
         {
         m_pClient = create_client<SERVICE>(strServiceName, QOS, pGroup);

         // See if the service is running and wait forever or until shutdown for it to appear
         while (!m_pClient->wait_for_service(std::chrono::seconds(1)))
            {
            if (!rclcpp::ok())
               {
               RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for service to appear.");
               // throw something?
               }
            RCLCPP_INFO(get_logger(), "Waiting for service to appear...");
            }

         return;
         }

      /****

      CallService

      Make the call to the server.  Returns a shared pointer to the Response stucture, contains nullptr on failure.

      ****/

      std::shared_ptr<typename SERVICE::Response> CallService(const std::shared_ptr<typename SERVICE::Request> pRequest)
         {
         std::shared_ptr<typename SERVICE::Response> pResponse = nullptr;

         // Make an asynchronous call to the server which returns a future (C++ std::future?)
         auto ResultFuture = m_pClient->async_send_request(pRequest);

         // Check to see if this node can continue to process checking the future without calling this
         if (rclcpp::spin_until_future_complete(shared_from_this(), ResultFuture) !=
               rclcpp::executor::FutureReturnCode::SUCCESS)
            {
            RCLCPP_ERROR(get_logger(), "Service Call Failed!");
            }
         else
            {
            // Get the result (If this IS a std::future, get() can only be called once or an exception will be raised!)
            // result is a pointer to a Result structure defined in the .srv file
            pResponse = ResultFuture.get();
            } // end else

         return (pResponse);
         } // end member function DruaiSimpleClientNode::CallService

      const std::string& GetNodeName() const
         {
         return (m_strNodeName);
         }

      std::string GetServiceName() const
         {
         return (m_pClient->get_name());
         }

   protected:
      std::string m_strNodeName;
      typename rclcpp::Client<SERVICE>::SharedPtr m_pClient;

   private:
   }; // end template class DruaiSimpleClientNode


/*****

  template class DruaiSimpleActionServerNode

*****/

template <typename ACTIONT>
class DruaiSimpleActionServerNode : public rclcpp::Node
   {
   public:
      using ACTION = ACTIONT;
      using GOALHANDLEACTION = rclcpp_action::ServerGoalHandle<ACTION>;

      DruaiSimpleActionServerNode(const std::string& strNodeName, const std::string& strActionName,
            const rclcpp::NodeOptions& Options = rclcpp::NodeOptions())
            : Node(strNodeName, Options), m_strNodeName{strNodeName}
         {
         using namespace std::placeholders;

         m_pServer = rclcpp_action::create_server<ACTION>(
            get_node_base_interface(),
            get_node_clock_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            strActionName,
            std::bind(&DruaiSimpleActionServerNode::HandleGoal, this, _1, _2),
            std::bind(&DruaiSimpleActionServerNode::HandleCancel, this, _1),
            std::bind(&DruaiSimpleActionServerNode::HandleAccepted, this, _1));

         return;
         } // end constructor DruaiSimpleActionServerNode

      const std::string& GetNodeName() const
         {
         return (m_strNodeName);
         }

      std::string GetActionName() const
         {
         return (m_pServer->get_name());
         }

   protected:
      std::string m_strNodeName;
      typename rclcpp_action::Server<ACTION>::SharedPtr m_pServer;

      /***************************
      These virtual funcions can be overridden to provide sepcialized handling
      of the action.  Execute() must be overridden.
      ***************************/

      /****

      DruaiSimpleActionServerNode::ProcessGoalRequest

      Called by HandleGoal() when the client's goal request first arrives.

      Override this method to perform specialized goal checking.
      This default version simply accepts the client's goal request.

      Returns either rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE or
      rclcpp_action::GoalResponse::REJECT

      ****/

      virtual rclcpp_action::GoalResponse ProcessGoalRequest(const rclcpp_action::GoalUUID& /* UUID */,
            std::shared_ptr<const typename ACTION::Goal> /* Goal */)
         {
         return (rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
         } // end member function DruaiSimpleActionServerNode::ProcessGoalRequest

      /****

      DruaiSimpleActionServerNode::Accept

      Called by HandleAccept().  This default version simply sets up a thread to
      perform the action by Execute().
      Override if more specialized handling is needed.

      NOTE: This default provides reentrancy so the server can accept goals from multiple indepenent clients.
      However, often a server will control unique piece of hardware and can only handle one goal at a time.  In this
      case there has to be some kind of system to control how multiple goal requests is handled.  If a goal is
      being processed, is the new request rejected, queued, or somehow overrides the running goal.  Does a running
      goal have to be explicitly canceled to accept a new goal?  Is there a priority system for clients as to who
      gets access?

      ****/

      virtual void Accept(const std::shared_ptr<GOALHANDLEACTION> GoalHandle)
         {
         using namespace std::placeholders;
         // this needs to return quickly to avoid blocking the executor, so spin up a new thread
         std::thread{std::bind(&DruaiSimpleActionServerNode::Execute, this, _1), GoalHandle}.detach();

         return;
         } // end member function DruaiSimpleActionServerNode::Accept

      /****

      DruaiSimpleActionServerNode::Cancel

      Called by HandleCancel() when the client sends a cancel request.

      Override this method to perform specialized cancel processing.
      This default version simply accepts the cancel request.

      ****/

      virtual rclcpp_action::CancelResponse Cancel(const std::shared_ptr<GOALHANDLEACTION> /* GoalHandle */)
         {
         return (rclcpp_action::CancelResponse::ACCEPT);
         } // end member function DruaiSimpleActionServerNode::Cancel

      /****

      DruaiSimpleActionServerNode::Execute

      Override this method to perform the action of the node and publish
      feedback to the client.

      The default HandleAccepted() sets this method up in a thread to execute.
      This allows requests from different clients to be executed in parallel.
      If Execute() needs to modify member variables, access must be protected.

      ****/

      virtual void Execute(const std::shared_ptr<typename rclcpp_action::ServerGoalHandle<ACTION>> GoalHandle) = 0;

   private:
      /***************************
      These are the functions which respond to the standard entry points of
      the action server.
      ***************************/

      /****

      DruaiSimpleActionServerNode::HandleGoal

      Called when the client sends a goal for processing. The server should
      evaluate the goal and either accept or reject it.

      ****/

      rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID& UUID,
            std::shared_ptr<const typename ACTION::Goal> Goal)
         {
         // Pass the goal to the derived class for evaluation
         return (ProcessGoalRequest(UUID, Goal));
         } // end member function DruaiSimpleActionServerNode::HandleGoal

      /****

      DruaiSimpleActionServerNode::HandleAccepted

      Called after the goal has been accepted.  This simply sets up a thread to
      perform the action.
      Override if more specialized handling is needed.

      ****/

      void HandleAccepted(const std::shared_ptr<GOALHANDLEACTION> GoalHandle)
         {
         Accept(GoalHandle);
         return;
         } // end member function DruaiSimpleActionServerNode::HandleAccepted


      /****

      DruaiSimpleActionServerNode::HandleCancel

      Called when the goal has been accepted.  This simply sets up a thread to
      perform the action.
      Override if more specialized handling is needed.

      ****/

      rclcpp_action::CancelResponse HandleCancel(
            const std::shared_ptr<GOALHANDLEACTION> GoalHandle)
         {
         return (Cancel(GoalHandle));
         } // end member function DruaiSimpleActionServerNode::HandleCancel

   }; // end template class DruaiSimpleActionServerNode

/*****

  template class DruaiSimpleActionClientNode

*****/

template <typename ACTIONT>
class DruaiSimpleActionClientNode : public rclcpp::Node
   {
   public:
      using ACTION = ACTIONT;
      using GOALHANDLEACTION = rclcpp_action::ClientGoalHandle<ACTION>;

      /****

      DruaiSimpleActionClientNode::DruaiSimpleActionClientNode

      ****/

      DruaiSimpleActionClientNode(const std::string& strNodeName, const std::string& strActionName,
           const rclcpp::NodeOptions& Options = rclcpp::NodeOptions())
         : Node(strNodeName, Options), m_strNodeName{strNodeName}, m_bGoalDone{false}
         {
         m_pClient = rclcpp_action::create_client<ACTION>(get_node_base_interface(), get_node_graph_interface(),
         get_node_logging_interface(), get_node_waitables_interface(), strActionName);

         return;
         }  // end constructor DruaiSimpleActionClientNode::DruaiSimpleActionClientNode

      const std::string& GetNodeName() const
         {
         return (m_strNodeName);
         }

      std::string GetActionName() const
         {
         return (m_pClient->get_name());
         }

      bool IsGoalDone() const
         {
         return m_bGoalDone;
         }

      bool IsServerRunning(std::chrono::seconds Seconds = std::chrono::seconds(10)) const
         {
         bool bRunning = m_pClient->wait_for_action_server(Seconds);
         if (!bRunning)
            {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            } // end if

         return (bRunning);
         }

      /****

      DruaiSimpleActionClientNode::SendGoal

      Send the goal to the action server.  Goal is a structure based on the goal section of the action definition.

      ****/

      void SendGoal(typename ACTION::Goal GoalMsg)
         {
         using namespace std::placeholders;

         m_bGoalDone = false;

         if (IsServerRunning())
            {
            typename rclcpp_action::Client<ACTION>::SendGoalOptions SendGoalOptions;

            // Responses from the action server are initially handled by these template functions
            // which in turn call virtual functions supplied by the derived class
            SendGoalOptions.goal_response_callback =
                  std::bind(&DruaiSimpleActionClientNode::GoalResponseCallback, this, _1);
            SendGoalOptions.feedback_callback =
                  std::bind(&DruaiSimpleActionClientNode::FeedbackCallback, this, _1, _2);
            SendGoalOptions.result_callback =
                  std::bind(&DruaiSimpleActionClientNode::ResultCallback, this, _1);

            // return value isn't used.  What's it good for?  TTC
            std::shared_future<typename GOALHANDLEACTION::SharedPtr> goal_handle_future =
                  m_pClient->async_send_goal(GoalMsg, SendGoalOptions);
            }

         return;
         } // end member function DruaiSimpleActionClientNode::SendGoal

   protected:
      std::string m_strNodeName;
      typename rclcpp_action::Client<ACTION>::SharedPtr m_pClient;
      bool m_bGoalDone;

      /***************************`
      These virtual funcions can be overridden to provide sepcialized handling
      of the action.  Execute() must be overridden.
      ***************************/

      virtual bool CheckResultCode(rclcpp_action::ResultCode nCode) const
         {
         bool bOK = false;

         switch (nCode)
            {
            case rclcpp_action::ResultCode::SUCCEEDED :
               bOK = true;
               break;

            case rclcpp_action::ResultCode::ABORTED :
               RCLCPP_ERROR(get_logger(), "Goal was aborted");
               break;

            case rclcpp_action::ResultCode::CANCELED :
               RCLCPP_INFO(get_logger(), "Goal was canceled");
               break;

            default:
               RCLCPP_ERROR(get_logger(), "Unknown result code");
               break;
            }

         return (bOK);
         }

      /****

      DruaiSimpleActionClientNode::ProcessGoalResponse

      This default version simply logs the result and moves on.  If other processing is needed, the derived
      class should override this method.

      ****/

      virtual void ProcessGoalResponse(std::shared_future<typename GOALHANDLEACTION::SharedPtr> Future)
         {
         typename GOALHANDLEACTION::SharedPtr pGoalHandle = Future.get();
         if (!pGoalHandle)
            {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
            }
         else
            {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            }

         return;
         } // end member function DruaiSimpleActionClientNode::ProcessGoalResponse

      virtual void ProcessFeedback(typename GOALHANDLEACTION::SharedPtr pGoalHandle,
            const std::shared_ptr<const typename ACTION::Feedback> pFeedback) = 0;
      virtual void ProcessResult(const typename GOALHANDLEACTION::WrappedResult& Result) = 0;

   private:
      /***************************
      These are the functions which respond to the standard entry points of
      the action Client.
      ***************************/

      void GoalResponseCallback(std::shared_future<typename GOALHANDLEACTION::SharedPtr> Future)
         {
         ProcessGoalResponse(Future);
         return;
         }

      void FeedbackCallback(typename GOALHANDLEACTION::SharedPtr pGoalHandle,
            const std::shared_ptr<const typename ACTION::Feedback> pFeedback)
         {
         ProcessFeedback(pGoalHandle, pFeedback);

         return;
         }

      void ResultCallback(const typename GOALHANDLEACTION::WrappedResult& Result)
         {
         m_bGoalDone = true;

         if (CheckResultCode(Result.code))
            {
            ProcessResult(Result);
            } // end if

         return;
         }

   }; // end template class DruaiSimpleActionClientNode
   


