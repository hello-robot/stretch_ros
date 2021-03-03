/*******************************************************
 * Copyright (C) 2021 Picknik Robotics
 *
 * This file can not be copied and/or distributed without the express
 * permission of Picknik Robotics.
 *******************************************************/

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_handle = rclcpp::Node::make_shared("stretch_plan_client");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_handle);
  std::thread([&executor]() { executor.spin(); }).detach();

  const std::string PLANNING_GROUP = "mobile_base_arm";

  moveit::planning_interface::MoveGroupInterface move_group(node_handle, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit::core::RobotState target_state(*move_group.getCurrentState());
  target_state.setVariablePosition("position/x", 2.0);
  target_state.setVariablePosition("position/y", 1.0);
  target_state.setVariablePosition("position/theta", 1.5);

  move_group.setJointValueTarget(target_state);

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 1.0;
  box_pose.position.y = 0.5;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  RCLCPP_INFO(rclcpp::get_logger("tutorial"), "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(rclcpp::get_logger("tutorial"), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
}
