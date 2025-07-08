#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

// 반복되는 클래스 선언을 위한 매크로
#define DECLARE_LIFECYCLE_NODE_ACTION(ClassName)                                \
class ClassName : public BT::StatefulActionNode                                 \
{                                                                               \
public:                                                                         \
    ClassName(const std::string& name, const BT::NodeConfiguration& config,     \
              rclcpp::Node::SharedPtr node_ptr);                                \
    static BT::PortsList providedPorts();                                       \
    BT::NodeStatus onStart() override;                                          \
    BT::NodeStatus onRunning() override;                                        \
    void onHalted() override;                                                   \
private:                                                                        \
    rclcpp::Node::SharedPtr node_ptr_;                                          \
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;  \
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_; \
    std::shared_future<std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>> future_change_state_; \
};

// 매크로를 이용해 5개 클래스 선언
DECLARE_LIFECYCLE_NODE_ACTION(ConfigureNode)
DECLARE_LIFECYCLE_NODE_ACTION(ActivateNode)
DECLARE_LIFECYCLE_NODE_ACTION(DeactivateNode)
DECLARE_LIFECYCLE_NODE_ACTION(CleanupNode)
DECLARE_LIFECYCLE_NODE_ACTION(ShutdownNode)