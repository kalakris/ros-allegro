#ifndef __ALLEGRO_NODE_PD_H__
#define __ALLEGRO_NODE_PD_H__

#include "allegro_node.h"


// Joint-space PD control of the Allegro hand.
//
// Allows you to save a position and command it to the hand controller.
// Controller gains are loaded from the ROS parameter server.
class AllegroNodePD : public AllegroNode {

 public:
  AllegroNodePD(const std::string node_name);

  ~AllegroNodePD();

  // Main spin code: just waits for messages.
  void doIt(bool polling = false);

  // Uses the String received command to set the hand into its home
  // position, or saves the grasp in order to go into PD control mode. Also
  // can turn the hand off.
  void libCmdCallback(const std_msgs::msg::String::SharedPtr msg);

  void setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Loads all gains and initial positions from the parameter server.
  void initController(const std::string &whichHand);

  // PD control happens here.
  void computeDesiredTorque();

 protected:
  // Handles defined grasp commands (std_msgs/String).
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lib_cmd_sub;

  // Subscribe to desired joint states, only so we can set control_hand_ to true
  // when we receive a desired command.
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;

  // If this flag is true, the hand will be controlled (either in joint position
  // or joint torques). If false, desired torques will all be zero.
  bool control_hand_ = false;
};

#endif  // __ALLEGRO_NODE_PD_H__
