#include "zero_controller/bt_action_nodes.hpp"

// --- 생성자, 포트, Halted 함수 공통 정의 ---
#define DEFINE_LIFECYCLE_NODE_ACTION_COMMON(ClassName)                               \
ClassName::ClassName(const std::string& name, const BT::NodeConfiguration& config,   \
                     rclcpp::Node::SharedPtr node_ptr)                               \
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr) {}                  \
                                                                                    \
BT::PortsList ClassName::providedPorts() {                                          \
    return{ BT::InputPort<std::string>("node_name") };                              \
}                                                                                   \
                                                                                    \
void ClassName::onHalted() {}                                                       \
                                                                                    \
BT::NodeStatus ClassName::onRunning() {                                             \
    if (future_change_state_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) { \
        return future_change_state_.get()->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; \
    }                                                                               \
    return BT::NodeStatus::RUNNING;                                                 \
}

DEFINE_LIFECYCLE_NODE_ACTION_COMMON(ConfigureNode)
DEFINE_LIFECYCLE_NODE_ACTION_COMMON(ActivateNode)
DEFINE_LIFECYCLE_NODE_ACTION_COMMON(DeactivateNode)
DEFINE_LIFECYCLE_NODE_ACTION_COMMON(CleanupNode)
DEFINE_LIFECYCLE_NODE_ACTION_COMMON(ShutdownNode)

// --- 각 노드의 핵심 로직 (onStart) ---

BT::NodeStatus ConfigureNode::onStart() {
    std::string target_node_name;
    if (!getInput<std::string>("node_name", target_node_name)) { throw BT::RuntimeError("Missing [node_name]"); }
    get_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::GetState>("/" + target_node_name + "/get_state");
    if (!get_state_client_->wait_for_service(std::chrono::seconds(1))) return BT::NodeStatus::FAILURE;
    auto future_get_state = get_state_client_->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (future_get_state.wait_for(std::chrono::seconds(1)) != std::future_status::ready) return BT::NodeStatus::FAILURE;
    
    if (future_get_state.get()->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Node [%s] is not in unconfigured state, no need to configure.", target_node_name.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    change_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::ChangeState>("/" + target_node_name + "/change_state");
    auto change_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    future_change_state_ = change_state_client_->async_send_request(change_request).future.share();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActivateNode::onStart() {
    std::string target_node_name;
    if (!getInput<std::string>("node_name", target_node_name)) { throw BT::RuntimeError("Missing [node_name]"); }
    get_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::GetState>("/" + target_node_name + "/get_state");
    change_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::ChangeState>("/" + target_node_name + "/change_state");
    if (!get_state_client_->wait_for_service(std::chrono::seconds(1))) return BT::NodeStatus::FAILURE;
    auto future_get_state = get_state_client_->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (future_get_state.wait_for(std::chrono::seconds(1)) != std::future_status::ready) return BT::NodeStatus::FAILURE;
    
    uint8_t current_state = future_get_state.get()->current_state.id;
    if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Node [%s] is already active.", target_node_name.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    auto change_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        change_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        auto future_conf = change_state_client_->async_send_request(change_request).future;
        if (future_conf.wait_for(std::chrono::seconds(2)) != std::future_status::ready) return BT::NodeStatus::FAILURE;
    }

    change_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    future_change_state_ = change_state_client_->async_send_request(change_request).future.share();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DeactivateNode::onStart() {
    std::string target_node_name;
    if (!getInput<std::string>("node_name", target_node_name)) { throw BT::RuntimeError("Missing [node_name]"); }
    get_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::GetState>("/" + target_node_name + "/get_state");
    if (!get_state_client_->wait_for_service(std::chrono::seconds(1))) return BT::NodeStatus::FAILURE;
    auto future_get_state = get_state_client_->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (future_get_state.wait_for(std::chrono::seconds(1)) != std::future_status::ready) return BT::NodeStatus::FAILURE;

    uint8_t current_state = future_get_state.get()->current_state.id;
    if (current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Node [%s] is not active, no need to deactivate.", target_node_name.c_str());
        return BT::NodeStatus::SUCCESS;
    }
  
    change_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::ChangeState>("/" + target_node_name + "/change_state");
    auto change_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    future_change_state_ = change_state_client_->async_send_request(change_request).future.share();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CleanupNode::onStart() {
    std::string target_node_name;
    if (!getInput<std::string>("node_name", target_node_name)) { throw BT::RuntimeError("Missing [node_name]"); }
    get_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::GetState>("/" + target_node_name + "/get_state");
    if (!get_state_client_->wait_for_service(std::chrono::seconds(1))) return BT::NodeStatus::FAILURE;
    auto future_get_state = get_state_client_->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (future_get_state.wait_for(std::chrono::seconds(1)) != std::future_status::ready) return BT::NodeStatus::FAILURE;

    uint8_t current_state = future_get_state.get()->current_state.id;
    if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Node [%s] is already unconfigured, no need to cleanup.", target_node_name.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Node [%s] is active, must be deactivated before cleanup.", target_node_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    change_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::ChangeState>("/" + target_node_name + "/change_state");
    auto change_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
    future_change_state_ = change_state_client_->async_send_request(change_request).future.share();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ShutdownNode::onStart() {
    std::string target_node_name;
    if (!getInput<std::string>("node_name", target_node_name)) { throw BT::RuntimeError("Missing [node_name]"); }
    
    get_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::GetState>("/" + target_node_name + "/get_state");
    if (!get_state_client_->wait_for_service(std::chrono::seconds(1))) return BT::NodeStatus::FAILURE;
    
    auto future_get_state = get_state_client_->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (future_get_state.wait_for(std::chrono::seconds(1)) != std::future_status::ready) return BT::NodeStatus::FAILURE;

    uint8_t current_state = future_get_state.get()->current_state.id;

    if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Node [%s] is already finalized.", target_node_name.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    change_state_client_ = node_ptr_->create_client<lifecycle_msgs::srv::ChangeState>("/" + target_node_name + "/change_state");
    auto change_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    
    // ✅ 현재 상태에 맞는 올바른 Shutdown Transition ID를 선택합니다.
    switch (current_state) {
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
            change_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
            break;
        case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
            change_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN;
            break;
        case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
            change_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
            break;
        default:
            RCLCPP_ERROR(node_ptr_->get_logger(), "Node [%s] is in a state that cannot be shut down.", target_node_name.c_str());
            return BT::NodeStatus::FAILURE;
    }
    
    future_change_state_ = change_state_client_->async_send_request(change_request).future.share();
    return BT::NodeStatus::RUNNING;
}