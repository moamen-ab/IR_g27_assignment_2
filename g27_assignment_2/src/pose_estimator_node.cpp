#include "cube_swap/pose_estimator_node.hpp"

namespace cube_swap
{

// ===========================================================================
// Constructor
// ===========================================================================
PoseEstimatorNode::PoseEstimatorNode(const rclcpp::NodeOptions & options)
: Node("pose_estimator_node", options)
{
  // TF2 buffer and listener
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Advertise the service
  service_ = this->create_service<g27_assignment_2::srv::GetCubePoses>(
    SERVICE_NAME,
    std::bind(
      &PoseEstimatorNode::getCubePosesCallback, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));

  RCLCPP_INFO(get_logger(),
    "PoseEstimatorNode ready. Waiting for service call on '%s'.",
    SERVICE_NAME);
  RCLCPP_INFO(get_logger(),
    "Expecting TF frames:  '%s'  and  '%s'.",
    TAG_FRAME_CUBE1, TAG_FRAME_CUBE2);
}

// ===========================================================================
// getCubePosesCallback
// ===========================================================================
void PoseEstimatorNode::getCubePosesCallback(
  const std::shared_ptr<rmw_request_id_t>                                  /*request_header*/,
  const std::shared_ptr<g27_assignment_2::srv::GetCubePoses::Request>   /*request*/,
  const std::shared_ptr<g27_assignment_2::srv::GetCubePoses::Response>  response)
{
  geometry_msgs::msg::PoseStamped pose_cube1, pose_cube2;

  bool ok1 = lookupTagInWorld(TAG_FRAME_CUBE1, pose_cube1);
  bool ok2 = lookupTagInWorld(TAG_FRAME_CUBE2, pose_cube2);

  if (!ok1 || !ok2)
  {
    response->success = false;
    response->message =
      std::string("Could not look up TF for: ") +
      (!ok1 ? std::string(TAG_FRAME_CUBE1) + " " : "") +
      (!ok2 ? std::string(TAG_FRAME_CUBE2)         : "") +
      ". Make sure apriltag_ros is running and both tags are visible.";

    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

    // adding offset
    pose_cube1.pose.position.x += 0.04;
    pose_cube1.pose.position.y += -0.213;
    pose_cube1.pose.position.z  = 0.379;
    pose_cube1.pose.orientation.w = 1.0;

    pose_cube2.pose.position.x += -0.133;
    pose_cube2.pose.position.y += -0.387;
    pose_cube2.pose.position.z  = 0.379;
    pose_cube2.pose.orientation.w = 1.0;

    response->cube1_pose  = pose_cube1;
    response->cube2_pose  = pose_cube2;
  
  response->success = true;
  response->message = "Both cube poses returned.";
  
  RCLCPP_INFO(get_logger(),
    "Returning cube1 (tag %d) world pos: (%.3f, %.3f, %.3f)",
    RED_CUBE_TAG_ID,
    pose_cube1.pose.position.x,
    pose_cube1.pose.position.y,
    pose_cube1.pose.position.z);

  RCLCPP_INFO(get_logger(),
    "Returning cube2 (tag %d) world pos: (%.3f, %.3f, %.3f)",
    BLUE_CUBE_TAG_ID,
    pose_cube2.pose.position.x,
    pose_cube2.pose.position.y,
    pose_cube2.pose.position.z);
    
}

// ===========================================================================
// lookupTagInWorld
// ===========================================================================
bool PoseEstimatorNode::lookupTagInWorld(
  const std::string &               tag_frame,
  geometry_msgs::msg::PoseStamped & pose_out)
{
  geometry_msgs::msg::TransformStamped tf_stamped;

  try {
    // tf2::TimePointZero = latest available transform (no timestamp required)
    tf_stamped = tf_buffer_->lookupTransform(
      WORLD_FRAME,          // target frame
      tag_frame,            // source frame
      tf2::TimePointZero);  // latest available
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(),
      "TF lookup failed [%s -> %s]: %s",
      tag_frame.c_str(), WORLD_FRAME, ex.what());
    return false;
  }

  // Convert TransformStamped → PoseStamped in world frame
  pose_out.header.stamp    = tf_stamped.header.stamp;
  pose_out.header.frame_id = WORLD_FRAME;

  pose_out.pose.position.x  = tf_stamped.transform.translation.x;
  pose_out.pose.position.y  = tf_stamped.transform.translation.y;
  pose_out.pose.position.z  = tf_stamped.transform.translation.z;
  //pose_out.pose.orientation = tf_stamped.transform.rotation;

  return true;
}

}  // namespace cube_swap

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cube_swap::PoseEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
