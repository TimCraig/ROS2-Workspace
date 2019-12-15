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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
            DruaiMinimalSubscriberNode(const std::string& strNodeName, const std::string& strTopic, const rclcpp::QoS& QOS)
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
         } // end member function ::CallService

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
   


