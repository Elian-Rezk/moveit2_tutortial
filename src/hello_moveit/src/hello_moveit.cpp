#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();


  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
  auto const text_pose = [] {
    auto msg = Eigen::Isometry3d::Identity();
    msg.translation().z() = 1.0;
    return msg;
  }();
  moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                  rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
  moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools,
     jmg = move_group_interface.getRobotModel()->getJointModelGroup(
         "panda_arm")](auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };

  // Set a target Pose with updated values !!!
  auto const target_pose = [] {
  geometry_msgs::msg::Pose msg;
  msg.orientation.y = 0.8;
  msg.orientation.w = 0.6;
  msg.position.x = 0.1;
  msg.position.y = 0.4;
  msg.position.z = 0.4;
  return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create collision object for the robot to avoid
  auto const collision_object1 = [frame_id =
                                 move_group_interface.getPlanningFrame()] {
  moveit_msgs::msg::CollisionObject collision_object1;
  collision_object1.header.frame_id = frame_id;
  collision_object1.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive1;


  // Define the size of the box in meters
  primitive1.type = primitive1.BOX;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[primitive1.BOX_X] = 0.5;
  primitive1.dimensions[primitive1.BOX_Y] = 0.1;
  primitive1.dimensions[primitive1.BOX_Z] = 1;


  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose1;
  box_pose1.orientation.w = 1.0;
  box_pose1.position.x = 0.2;
  box_pose1.position.y = 0.2;
  box_pose1.position.z = 0.5;


  collision_object1.primitives.push_back(primitive1);
  collision_object1.primitive_poses.push_back(box_pose1);
  collision_object1.operation = collision_object1.ADD;



  return collision_object1;
  }();

    //added another collision object
  auto const collision_object2 = [frame_id =
                                 move_group_interface.getPlanningFrame()] {
  moveit_msgs::msg::CollisionObject collision_object2;
  collision_object2.header.frame_id = frame_id;
  collision_object2.id = "box2";
  shape_msgs::msg::SolidPrimitive primitive2;


  primitive2.type = primitive2.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[primitive2.BOX_X] = 0.5;
  primitive2.dimensions[primitive2.BOX_Y] = 0.1;
  primitive2.dimensions[primitive2.BOX_Z] = 0.5;

  geometry_msgs::msg::Pose box_pose2;
  box_pose2.orientation.w = 1.0;
  box_pose2.position.x = -0.3;
  box_pose2.position.y = 0.2;
  box_pose2.position.z = 0.75;

  collision_object2.primitives.push_back(primitive2);
  collision_object2.primitive_poses.push_back(box_pose2);
  collision_object2.operation = collision_object2.ADD;

  return collision_object2;
  }();

  //   //added yet another collision object (third)
  // auto const collision_object3 = [frame_id =
  //                                move_group_interface.getPlanningFrame()] {
  // moveit_msgs::msg::CollisionObject collision_object3;
  // collision_object3.header.frame_id = frame_id;
  // collision_object3.id = "box3";
  // shape_msgs::msg::SolidPrimitive primitive3;


  // primitive3.type = primitive3.BOX;
  // primitive3.dimensions.resize(3);
  // primitive3.dimensions[primitive3.BOX_X] = 0.5;
  // primitive3.dimensions[primitive3.BOX_Y] = 0.1;
  // primitive3.dimensions[primitive3.BOX_Z] = 1;

  // geometry_msgs::msg::Pose box_pose3;
  // box_pose3.orientation.w = 1.0;
  // box_pose3.position.x = -0.8;
  // box_pose3.position.y = 0.2;
  // box_pose3.position.z = 0.5;

  // collision_object3.primitives.push_back(primitive3);
  // collision_object3.primitive_poses.push_back(box_pose3);
  // collision_object3.operation = collision_object3.ADD;

  // return collision_object3;
  // }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object1);
  planning_scene_interface.applyCollisionObject(collision_object2);
  //planning_scene_interface.applyCollisionObject(collision_object3);

  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
  draw_trajectory_tool_path(plan.trajectory_);
  moveit_visual_tools.trigger();
  prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
  draw_title("Executing");
  moveit_visual_tools.trigger();
  move_group_interface.execute(plan);
  } else {
  draw_title("Planning Failed!");
  moveit_visual_tools.trigger();
  RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}