#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ego_planner_node");

  EGOReplanFSM rebo_replan;

  rebo_replan.init(node);

  // rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0)));
  // 单线程执行器（与 ROS1 ros::spin() 等价）——GridMap 回调无内部锁，禁止 MultiThreadedExecutor
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
