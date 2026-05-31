#include "cube_swap/cube_swap_node.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace cube_swap
{

// ===========================================================================
// Constructor
// ===========================================================================
CubeSwapNode::CubeSwapNode(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("cube_swap_node", options))
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
CubeSwapNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// ===========================================================================
// run()
// ===========================================================================
void CubeSwapNode::run()
{
  // -----------------------------------------------------------------------
  // Step 1: Call the pose estimator service to get both cube poses
  // -----------------------------------------------------------------------
  auto client = node_->create_client<g27_assignment_2::srv::GetCubePoses>(
    "/get_cube_poses");

  RCLCPP_INFO(node_->get_logger(), "Waiting for /get_cube_poses service...");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) { return; }
    RCLCPP_INFO(node_->get_logger(), "  ...still waiting for service");
  }

  // Keep calling until both cubes are detected
  geometry_msgs::msg::PoseStamped cube1_pose, cube2_pose;
  while (rclcpp::ok())
  {
    auto request  = std::make_shared<
      g27_assignment_2::srv::GetCubePoses::Request>();
    auto future   = client->async_send_request(request);

    // Wait for the future without touching the executor — the node is
    // already being spun by MultiThreadedExecutor in the spin_thread.
    auto timeout  = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (future.wait_for(std::chrono::milliseconds(50)) !=
           std::future_status::ready)
    {
      if (!rclcpp::ok() ||
          std::chrono::steady_clock::now() > timeout) {
        RCLCPP_WARN(node_->get_logger(), "Service call timed out, retrying...");
        break;
      }
    }
    if (future.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready) {
      continue;
    }

    auto response = future.get();
    if (response->success) {
      cube1_pose = response->cube1_pose;
      cube2_pose = response->cube2_pose;
      RCLCPP_INFO(node_->get_logger(), "Got both cube poses. Starting swap.");
      break;
    }

    RCLCPP_WARN(node_->get_logger(),
      "Pose service: %s — retrying in 2s", response->message.c_str());
    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  // -----------------------------------------------------------------------
  // Step 2: Register collision objects in the MoveIt planning scene
  // -----------------------------------------------------------------------
  setupPlanningScene(cube1_pose, cube2_pose);

  // temp1_pose near cube1
  geometry_msgs::msg::PoseStamped temp1_pose;
  temp1_pose.header.frame_id    = WORLD_FRAME;
  temp1_pose.pose.position.x    = TEMP1_X;
  temp1_pose.pose.position.y    = TEMP1_Y;
  temp1_pose.pose.position.z    = TEMP1_Z;
  temp1_pose.pose.orientation.w = 1.0;

  // temp2_pose near cube2
  geometry_msgs::msg::PoseStamped temp2_pose;
  temp2_pose.header.frame_id    = WORLD_FRAME;
  temp2_pose.pose.position.x    = TEMP2_X;
  temp2_pose.pose.position.y    = TEMP2_Y;
  temp2_pose.pose.position.z    = TEMP2_Z;
  temp2_pose.pose.orientation.w = 1.0;

  // -----------------------------------------------------------------------
  // Step 3: Execute the three-step swap using MTC
  //
  //   Phase A: pick cube1 → place at temp2
  //   Phase B: pick cube2 → place at temp1
  // -----------------------------------------------------------------------

  // --- Phase A ---
  RCLCPP_INFO(node_->get_logger(), "Phase A: cube2 → temp1");
  auto task_a = createPickPlaceTask(
    "phase_a", "cube2", cube2_pose, temp1_pose);
  if (!executeTask(task_a)) {
    RCLCPP_ERROR(node_->get_logger(), "Phase A failed — aborting swap.");
    return;
  }

  // --- Phase B ---
  RCLCPP_INFO(node_->get_logger(), "Phase B: cube1 → temp2");
  auto task_b = createPickPlaceTask(
    "phase_b", "cube1", cube1_pose, temp2_pose);
  if (!executeTask(task_b)) {
    RCLCPP_ERROR(node_->get_logger(), "Phase B failed — aborting swap.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Cube swap completed successfully!");
}

// ===========================================================================
// setupPlanningScene
// ===========================================================================
void CubeSwapNode::setupPlanningScene(
  const geometry_msgs::msg::PoseStamped & cube1_pose,
  const geometry_msgs::msg::PoseStamped & cube2_pose)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  auto makeCubeObject =
    [&](const std::string & id,
        const geometry_msgs::msg::PoseStamped & ps) -> moveit_msgs::msg::CollisionObject
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id                  = id;
    obj.header.frame_id     = WORLD_FRAME;
    obj.operation           = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type               = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions         = { CUBE_X, CUBE_Y, CUBE_Z };
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(ps.pose);
    return obj;
  };

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  // Raise cube poses by 1mm to avoid table surface penetration
  auto raised1 = cube1_pose;
  auto raised2 = cube2_pose;
  raised1.pose.position.z += 0.001;
  raised2.pose.position.z += 0.001;

  objects.push_back(makeCubeObject("cube1", raised1));
  objects.push_back(makeCubeObject("cube2", raised2));

  psi.applyCollisionObjects(objects);
  RCLCPP_INFO(node_->get_logger(),
    "Planning scene: cube1 and cube2 registered as collision objects.");
}

// ===========================================================================
// createPickPlaceTask
// ===========================================================================
mtc::Task CubeSwapNode::createPickPlaceTask(
  const std::string & task_name,
  const std::string & object_id,
  const geometry_msgs::msg::PoseStamped & pick_pose,
  const geometry_msgs::msg::PoseStamped & place_pose)
{
  mtc::Task task;
  task.stages()->setName(task_name);
  task.loadRobotModel(node_);

  // Set top-level task properties from our SRDF names
  task.setProperty("group",    ARM_GROUP);
  task.setProperty("eef",      EEF_NAME);
  task.setProperty("ik_frame", EEF_FRAME);

  // -------------------------------------------------------------------
  // Planners
  // -------------------------------------------------------------------
  auto sampling_planner     = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("planning_time", 50.0);   // increase from default 10s
  sampling_planner->setProperty("num_planning_attempts", 25u);

  sampling_planner->setProperty("ompl.workspace.min.x", 3.0);
  sampling_planner->setProperty("ompl.workspace.min.y", -1.5);
  sampling_planner->setProperty("ompl.workspace.min.z",  0.3);

  sampling_planner->setProperty("ompl.workspace.max.x",  5.5);
  sampling_planner->setProperty("ompl.workspace.max.y",  0.5);
  sampling_planner->setProperty("ompl.workspace.max.z",  1.5);

  auto interpolation_planner= std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner    = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(0.01);

  // -------------------------------------------------------------------
  // Stage 0: Current state
  // -------------------------------------------------------------------
  mtc::Stage * current_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // -------------------------------------------------------------------
  // Stage 1: Allow collisions
  // -------------------------------------------------------------------

  // Allow cube-table cube-ground collision
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow cube1-table collision");
    stage->allowCollisions(
      "cube1",
      std::vector<std::string>{"cafe_table_link", "cafe_table2_link",
                               "ground_plane_link"},
      true);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow cube2-table collision");
    stage->allowCollisions(
      "cube2",
      std::vector<std::string>{"cafe_table_link", "cafe_table2_link",
                               "ground_plane_link"},
      true);
    task.add(std::move(stage));
  }

  // Allow cube1 - cube2 collision
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow cube1-cube2 collision");
    stage->allowCollisions(
      "cube1","cube2",true);
    task.add(std::move(stage));
  }

  // 3c: Allow collision between gripper and cube
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow collision (gripper, object)");
    stage->allowCollisions(
      "cube1",
      task.getRobotModel()
          ->getJointModelGroup(GRIPPER_GROUP)
          ->getLinkModelNamesWithCollisionGeometry(),
      true);
    task.add(std::move(stage));
  }
  // Allow collision between gripper and the OTHER cube
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow collision (gripper, other cube)");
    stage->allowCollisions(
      "cube2",
      task.getRobotModel()
          ->getJointModelGroup(GRIPPER_GROUP)
          ->getLinkModelNamesWithCollisionGeometry(),
      true);
    task.add(std::move(stage));
  }
  // -------------------------------------------------------------------
  // Stage 2: Open gripper
  // -------------------------------------------------------------------
  if(task_name == "phase_a"){  
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
        "open gripper", interpolation_planner);
      stage->setGroup(GRIPPER_GROUP);
      stage->setGoal("open");   
      task.add(std::move(stage));
    }
  }  
  // -------------------------------------------------------------------
  // Stage 2: Move to pick position (Connect stage)
  // -------------------------------------------------------------------
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{
        { ARM_GROUP, sampling_planner }
      });
    stage->setTimeout(50.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // -------------------------------------------------------------------
  // Stage 3: Pick serial container
  //   3a  approach object  (move down toward cube)
  //   3b  generate grasp pose + IK
  //   3c  close gripper
  //   3d  attach object
  //   3e  lift object
  // -------------------------------------------------------------------
  mtc::Stage * attach_object_stage = nullptr;
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(pick->properties(), { "eef", "group", "ik_frame" });
    pick->properties().configureInitFrom(mtc::Stage::PARENT,
                                         { "eef", "group", "ik_frame" });

    // 3a: Approach — move down in the world Z direction
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
        "approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach");
      stage->properties().set("link", EEF_FRAME);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.1);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = WORLD_FRAME;
      vec.vector.z = -1.0;   // descend
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    // 3b: Generate grasp pose + IK
    {
      auto grasp_stage = std::make_unique<mtc::stages::GenerateGraspPose>(
        "generate grasp pose");
      grasp_stage->properties().configureInitFrom(mtc::Stage::PARENT);
      grasp_stage->properties().set("marker_ns", "grasp_pose");
      grasp_stage->setPreGraspPose("open");
      grasp_stage->setObject(object_id);
      grasp_stage->setAngleDelta(M_PI / 8);
      grasp_stage->setMonitoredStage(current_state_ptr);

      // IK frame: align gripper for top-down approach.
      Eigen::Isometry3d grasp_frame = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0,    Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI / 2,    Eigen::Vector3d::UnitZ());
      grasp_frame.linear()          = q.matrix();
      grasp_frame.translation().z() = 0.18;  // offset from tool0 to fingertip

      auto ik = std::make_unique<mtc::stages::ComputeIK>(
        "grasp pose IK", std::move(grasp_stage));
      ik->setMaxIKSolutions(8);
      ik->setMinSolutionDistance(1.0);
      ik->setIKFrame(grasp_frame, EEF_FRAME);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      pick->insert(std::move(ik));
    }


    // 3c: Close gripper

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
          "close gripper", interpolation_planner);
      stage->setGroup(GRIPPER_GROUP);
      
      std::map<std::string, double> gripper_goal;
      if(object_id == "cube2"){
        gripper_goal["robotiq_85_left_knuckle_joint"] = 0.02;  // adjust between 0 (open) and 0.8 (close)
      } else if(object_id == "cube1"){
        gripper_goal["robotiq_85_left_knuckle_joint"] = 0.07;  // adjust between 0 (open) and 0.8 (close)
      }
      stage->setGoal(gripper_goal);

      //stage->setGoal("close");
      
      pick->insert(std::move(stage));
    }


    // 3d: Attach object to end effector
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "attach object");
      stage->attachObject(object_id, EEF_FRAME);
      attach_object_stage = stage.get();
      pick->insert(std::move(stage));
    }

    // 3e: Lift object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
        "lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.3);
      stage->setIKFrame(EEF_FRAME);
      stage->properties().set("marker_ns", "lift");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = WORLD_FRAME;
      vec.vector.z = 1.0;   // ascend
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    task.add(std::move(pick));
  }

  // -------------------------------------------------------------------
  // Stage 4: Move to place position (Connect stage)
  // -------------------------------------------------------------------
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{
        { ARM_GROUP,     sampling_planner },
        //{ GRIPPER_GROUP, interpolation_planner }
      });
    stage->setTimeout(50.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // -------------------------------------------------------------------
  // Stage 5: Place serial container
  //   5a  generate place pose + IK
  //   5b  open gripper
  //   5c  detach object
  //   5d  retreat
  // -------------------------------------------------------------------
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });


    // 5a: Generate place pose + IK
    {
      auto place_stage = std::make_unique<mtc::stages::GeneratePlacePose>(
        "generate place pose");
      place_stage->properties().configureInitFrom(mtc::Stage::PARENT);
      place_stage->properties().set("marker_ns", "place_pose");
      place_stage->setObject(object_id);
      place_stage->setPose(place_pose);
      place_stage->setMonitoredStage(attach_object_stage);

      // Same IK frame as grasp — top-down approach, 0.15m z offset.
      Eigen::Isometry3d place_frame = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond pq =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0,    Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI / 2,    Eigen::Vector3d::UnitZ());
      place_frame.linear()          = pq.matrix();
      place_frame.translation().z() = 0.2;

      auto ik = std::make_unique<mtc::stages::ComputeIK>(
        "place pose IK", std::move(place_stage));
      ik->setMaxIKSolutions(8);
      ik->setMinSolutionDistance(1.0);
      ik->setIKFrame(place_frame, EEF_FRAME);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(ik));
    }

    // 5b: Open gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
        "open gripper", interpolation_planner);
      stage->setGroup(GRIPPER_GROUP);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }


    // 5c: Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "detach object");
      stage->detachObject(object_id, EEF_FRAME);
      place->insert(std::move(stage));
    }


    // 5d: Retreat — lift well clear before returning home
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
        "retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.3);   // increased to clear scene
      stage->setIKFrame(EEF_FRAME);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = WORLD_FRAME;
      vec.vector.z = 1.0;   // lift away
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }


    task.add(std::move(place));
  }
    


  // -------------------------------------------------------------------
  // Stage 7: Return home
  // -------------------------------------------------------------------
  
  if (task_name == "phase_b"){
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
        "return home", sampling_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setGoal("home");
      task.add(std::move(stage));
    }

    // Close gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
        "close gripper", interpolation_planner);
      stage->setGroup(GRIPPER_GROUP);
      stage->setGoal("close");
      task.add(std::move(stage));
    }
    
  }

  return task;
}

// ===========================================================================
// executeTask
// ===========================================================================
bool CubeSwapNode::executeTask(mtc::Task & task)
{
  try {
    task.init();
  }
  catch (mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
      "Task init failed: " << e);
    return false;
  }

  if (!task.plan(10)) {  // try up to 10 solutions
    RCLCPP_ERROR(node_->get_logger(),
      "Task '%s' planning failed.", task.stages()->name().c_str());
    return false;
  }

  // Publish solution for RViz visualisation
  task.introspection().publishSolution(*task.solutions().front());


  auto result = task.execute(*task.solutions().front());

  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
      "Task '%s' execution failed.", task.stages()->name().c_str());
    return false;
  }

  return true;
}

}  // namespace cube_swap

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto swap_node = std::make_shared<cube_swap::CubeSwapNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>(
    [&executor, &swap_node]() {
      executor.add_node(swap_node->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(swap_node->getNodeBaseInterface());
    });

  swap_node->run();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
