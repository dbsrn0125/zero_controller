#include "rclcpp/rclcpp.hpp"
#include "zenith_interfaces/srv/change_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "zero_controller/bt_action_nodes.hpp" // 비어있는 헤더

#include <map>
#include <string>

enum class RobotState {
  IDLE, MANUAL, EMERGENCY,
  AUTO_1, AUTO_2, AUTO_3, AUTO_4, AUTO_5,
  UNKNOWN 
};

RobotState g_current_state = RobotState::UNKNOWN;
RobotState g_previous_state = RobotState::UNKNOWN; 

std::map<RobotState, std::string> g_state_names = {
  {RobotState::IDLE, "IDLE"}, {RobotState::MANUAL, "MANUAL"},
  {RobotState::EMERGENCY, "EMERGENCY"}, {RobotState::AUTO_1, "AUTO_1"},
  {RobotState::AUTO_2, "AUTO_2"}, {RobotState::AUTO_3, "AUTO_3"},
  {RobotState::AUTO_4, "AUTO_4"}, {RobotState::AUTO_5, "AUTO_5"}
};

void change_state_callback(
  const std::shared_ptr<zenith_interfaces::srv::ChangeState::Request> request,
  std::shared_ptr<zenith_interfaces::srv::ChangeState::Response> response)
{
  if (request->requested_state == "IDLE")      g_current_state = RobotState::IDLE;
  else if (request->requested_state == "MANUAL")  g_current_state = RobotState::MANUAL;
  else if (request->requested_state == "EMERGENCY") g_current_state = RobotState::EMERGENCY;
  else if (request->requested_state == "AUTO_1")  g_current_state = RobotState::AUTO_1;
  else if (request->requested_state == "AUTO_2")  g_current_state = RobotState::AUTO_2;
  else if (request->requested_state == "AUTO_3")  g_current_state = RobotState::AUTO_3;
  else if (request->requested_state == "AUTO_4")  g_current_state = RobotState::AUTO_4;
  else if (request->requested_state == "AUTO_5")  g_current_state = RobotState::AUTO_5;
  else {
    response->success = false;
    response->message = "Unknown state requested: " + request->requested_state;
    return;
  }
  response->success = true;
  response->message = request->requested_state;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("zero_controller_node");

  // 1. BT 팩토리 생성 및 노드 등록
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ConfigureNode>("ConfigureNode", node);
  factory.registerNodeType<ActivateNode>("ActivateNode", node);
  factory.registerNodeType<DeactivateNode>("DeactivateNode", node);
  factory.registerNodeType<CleanupNode>("CleanupNode", node);
  factory.registerNodeType<ShutdownNode>("ShutdownNode", node);

  // 2. 미션 XML 파일 로드
  std::string package_share_path = ament_index_cpp::get_package_share_directory("zero_controller");
  auto auto_1_tree = factory.createTreeFromFile(package_share_path + "/bt_xml/auto_1_mission.xml");
  auto auto_2_tree = factory.createTreeFromFile(package_share_path + "/bt_xml/auto_2_mission.xml");
  // ... (나머지 트리는 필요 시 주석 해제) ...

  // 3. 서비스 서버 생성
  auto service = node->create_service<zenith_interfaces::srv::ChangeState>(
    "change_state", &change_state_callback);

  RCLCPP_INFO(node->get_logger(), "--- Zero Controller Node Started (Skeleton Ready) ---");

  // 4. 멀티 스레드 Executor 생성 및 노드 추가
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // 5. Behavior Tree를 실행할 별도의 스레드 생성
  auto bt_thread = std::thread([&]() {
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
      if (g_current_state != g_previous_state) {
        RCLCPP_INFO(node->get_logger(), "State Changed To: [ %s ]", g_state_names[g_current_state].c_str());
        g_previous_state = g_current_state;
      }

      switch (g_current_state) {
        case RobotState::AUTO_1:
          auto_1_tree.tickOnce();
          break;
        case RobotState::AUTO_2:
          auto_2_tree.tickOnce();
          break;
        default:
          break;
      }
      loop_rate.sleep();
    }
  });

  // 6. Executor 실행 (ROS 통신 처리 시작)
  executor.spin();

  // 7. 종료 시 스레드 정리
  bt_thread.join();
  rclcpp::shutdown();
  return 0;
}