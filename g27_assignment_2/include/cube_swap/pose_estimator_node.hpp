#ifndef CUBE_SWAP__POSE_ESTIMATOR_NODE_HPP_
#define CUBE_SWAP__POSE_ESTIMATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include "g27_assignment_2/srv/get_cube_poses.hpp"

namespace cube_swap
{

// ---------------------------------------------------------------------------
// Tag IDs
// ---------------------------------------------------------------------------
constexpr int RED_CUBE_TAG_ID  = 1;    //  (red  cube)
constexpr int BLUE_CUBE_TAG_ID = 10;   //  (blue cube)

constexpr char TAG_FRAME_CUBE1[] = "tag36h11:1";
constexpr char TAG_FRAME_CUBE2[] = "tag36h11:10";

// ---------------------------------------------------------------------------
// PoseEstimatorNode
// ---------------------------------------------------------------------------
class PoseEstimatorNode : public rclcpp::Node
{
public:
  explicit PoseEstimatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Service callback — looks up TF for both tag frames and returns poses
  void getCubePosesCallback(
    const std::shared_ptr<rmw_request_id_t>                                 request_header,
    const std::shared_ptr<g27_assignment_2::srv::GetCubePoses::Request>  request,
    const std::shared_ptr<g27_assignment_2::srv::GetCubePoses::Response> response);

  // Look up the latest transform for tag_frame in world frame.
  // Returns true and fills pose_out on success.
  bool lookupTagInWorld(
    const std::string &                    tag_frame,
    geometry_msgs::msg::PoseStamped &      pose_out);

  // -----------------------------------------------------------------------
  rclcpp::Service<g27_assignment_2::srv::GetCubePoses>::SharedPtr service_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  static constexpr char WORLD_FRAME[]  = "world";
  static constexpr char SERVICE_NAME[] = "/get_cube_poses";
};

}  // namespace cube_swap

#endif  // CUBE_SWAP__POSE_ESTIMATOR_NODE_HPP_
