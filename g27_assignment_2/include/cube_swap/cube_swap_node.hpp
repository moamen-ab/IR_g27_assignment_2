#ifndef CUBE_SWAP__CUBE_SWAP_NODE_HPP_
#define CUBE_SWAP__CUBE_SWAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include "g27_assignment_2/srv/get_cube_poses.hpp"

namespace mtc = moveit::task_constructor;

namespace cube_swap
{

// Cube dimensions (metres)
constexpr double CUBE_X = 0.06;
constexpr double CUBE_Y = 0.06;
constexpr double CUBE_Z = 0.10;

// TEMP1 near cube1
constexpr double TEMP1_X = 4.12;
constexpr double TEMP1_Y = -0.98;
constexpr double TEMP1_Z = 0.38;

// TEMP2 near cube2
constexpr double TEMP2_X = 4.48;
constexpr double TEMP2_Y = -0.62;
constexpr double TEMP2_Z = 0.38;


class CubeSwapNode
{
public:
  explicit CubeSwapNode(const rclcpp::NodeOptions & options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // Main entry — calls pose service then executes the full swap
  void run();

private:
  // Register collision objects so MoveIt plans around the cubes
  void setupPlanningScene(
    const geometry_msgs::msg::PoseStamped & cube1_pose,
    const geometry_msgs::msg::PoseStamped & cube2_pose);

  // Build a full pick-and-place MTC task for one cube
  mtc::Task createPickPlaceTask(
    const std::string & task_name,
    const std::string & object_id,
    const geometry_msgs::msg::PoseStamped & pick_pose,
    const geometry_msgs::msg::PoseStamped & place_pose);

  // Plan and execute a task; returns true on success
  bool executeTask(mtc::Task & task);

  rclcpp::Node::SharedPtr node_;

  const std::string ARM_GROUP     = "ir_arm";
  const std::string GRIPPER_GROUP = "ir_gripper";  // planning group name 
  const std::string EEF_NAME      = "gripper";     // end effector name  
  const std::string EEF_FRAME     = "tool0";       // tip link of ir_arm
  const std::string WORLD_FRAME   = "world";
};

}  // namespace cube_swap

#endif  // CUBE_SWAP__CUBE_SWAP_NODE_HPP_
