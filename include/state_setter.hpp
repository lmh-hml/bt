#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include <std_msgs/msg/string.hpp>
#include "states.hpp"

class StateSetterNode :  public BT::RosTopicSubNode<std_msgs::msg::String>
{
    public:
        StateSetterNode(const std::string& name, 
                        const BT::NodeConfiguration& config,
                        const BT::RosNodeParams& params)
            : BT::RosTopicSubNode<std_msgs::msg::String>(name, config,params)
        {
            state_ = STATE::UNKNOWN;
        }

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({ 
                BT::OutputPort<STATE>("state") 
            });
        }

        void topicCallback(const std_msgs::msg::String::SharedPtr msg) override
        {
            printf("StateSetter: %s\n", msg->data.c_str());
            if(msg->data == "idle")
                state_ = STATE::IDLE;
            else if(msg->data == "auto")
                state_ = STATE::AUTO;
            else if(msg->data == "manual")
                state_ = STATE::MANUAL;
            else if(msg->data == "error")
                state_ = STATE::ERROR;
            else
                state_ = STATE::UNKNOWN;
        }

        //The BT is ticked at a much faster rate than the topic is updated, so last_msg will be reset 
        //nullptr between subscriber callbacks. This makes last_msg unsuitable for retrieving the
        //last subscription message.
        //state_ is used to keep track of the subscription state instead of last_msg.

        BT::NodeStatus onTick(const typename std_msgs::msg::String::SharedPtr& last_msg) override
        {
            if(state_ == STATE::UNKNOWN)
            {
                return BT::NodeStatus::FAILURE;
            }
            setOutput("state", state_);
            state_ = STATE::UNKNOWN;
            return BT::NodeStatus::SUCCESS;
        }
    private:
    STATE state_;
};