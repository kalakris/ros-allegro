using namespace std;

#include "allegro_node_pd.h"
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

// Default parameters.
double k_p[DOF_JOINTS] =
        {
                // Default P Gains for PD Controller, loaded if
                // 'gains_pd.yaml' file is not loaded.
                600.0, 600.0, 600.0, 1000.0, 600.0, 600.0, 600.0, 1000.0,
                600.0, 600.0, 600.0, 1000.0, 1000.0, 1000.0, 1000.0, 600.0
        };

double k_d[DOF_JOINTS] =
        {
                // Default D Gains for PD Controller, loaded if
                // 'gains_pd.yaml' file is not loaded.
                15.0, 20.0, 15.0, 15.0, 15.0, 20.0, 15.0, 15.0,
                15.0, 20.0, 15.0, 15.0, 30.0, 20.0, 20.0, 15.0
        };

double home_pose[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };

std::string pGainParams[DOF_JOINTS] =
        {
                "gains_pd/p/j00", "gains_pd/p/j01", "gains_pd/p/j02",
                "gains_pd/p/j03",
                "gains_pd/p/j10", "gains_pd/p/j11", "gains_pd/p/j12",
                "gains_pd/p/j13",
                "gains_pd/p/j20", "gains_pd/p/j21", "gains_pd/p/j22",
                "gains_pd/p/j23",
                "gains_pd/p/j30", "gains_pd/p/j31", "gains_pd/p/j32",
                "gains_pd/p/j33"
        };

std::string dGainParams[DOF_JOINTS] =
        {
                "gains_pd/d/j00", "gains_pd/d/j01", "gains_pd/d/j02",
                "gains_pd/d/j03",
                "gains_pd/d/j10", "gains_pd/d/j11", "gains_pd/d/j12",
                "gains_pd/d/j13",
                "gains_pd/d/j20", "gains_pd/d/j21", "gains_pd/d/j22",
                "gains_pd/d/j23",
                "gains_pd/d/j30", "gains_pd/d/j31", "gains_pd/d/j32",
                "gains_pd/d/j33"
        };

std::string initialPosition[DOF_JOINTS] =
        {
                "initial_position/j00", "initial_position/j01",
                "initial_position/j02",
                "initial_position/j03",
                "initial_position/j10", "initial_position/j11",
                "initial_position/j12",
                "initial_position/j13",
                "initial_position/j20", "initial_position/j21",
                "initial_position/j22",
                "initial_position/j23",
                "initial_position/j30", "initial_position/j31",
                "initial_position/j32",
                "initial_position/j33"
        };

// Constructor subscribes to topics.
AllegroNodePD::AllegroNodePD(const std::string nodeName, bool sim)
  : AllegroNode(nodeName, sim) {
  control_hand_ = false;

  initController(whichHand);

  lib_cmd_sub = create_subscription<std_msgs::msg::String>(
          LIB_CMD_TOPIC, 1, std::bind(&AllegroNodePD::libCmdCallback, this, std::placeholders::_1));

  joint_cmd_sub = create_subscription<sensor_msgs::msg::JointState>(
          DESIRED_STATE_TOPIC, 1, std::bind(&AllegroNodePD::setJointCallback, this, std::placeholders::_1));
}

AllegroNodePD::~AllegroNodePD() {
  RCLCPP_INFO(get_logger(), "PD controller node is shutting down");
}

// Called when an external (string) message is received
void AllegroNodePD::libCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "CTRL: Heard: [%s]", msg->data.c_str());

  const std::string lib_cmd = msg->data.c_str();

  // Compare the message received to an expected input
  if (lib_cmd.compare("pdControl") == 0) {
    control_hand_ = true;
  }

  else if (lib_cmd.compare("home") == 0) {
    // Set the home position as the desired joint states.
    mutex->lock();
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    control_hand_ = true;
    mutex->unlock();
  }
  else if (lib_cmd.compare("off") == 0) {
    control_hand_ = false;
  }

  else if (lib_cmd.compare("save") == 0) {
    // Set the current position as the desired joint states.
    mutex->lock();
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = current_position[i];
    mutex->unlock();
  }
}

void AllegroNodePD::setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  RCLCPP_WARN_EXPRESSION(get_logger(), !control_hand_, "Setting control_hand_ to True because of "
                "received JointState message");
  control_hand_ = true;
}

void AllegroNodePD::computeDesiredTorque() {
  // NOTE: here we just compute and set the desired_torque class member
  // variable.

  // No control: set torques to zero.
  if (!control_hand_) {
    //ROS_INFO_THROTTLE(1.0, "Hand control is false");
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_torque[i] = 0.0;
    }
    return;
  }

  // Sanity/defensive check: if *both* position and torques are set in the
  // message, do nothing.
  if (desired_joint_state.position.size() > 0 &&
      desired_joint_state.effort.size() > 0) {
    RCLCPP_WARN(get_logger(), "Error: both positions and torques are specified in the desired "
                     "state. You cannot control both at the same time.");
    return;
  }

  {
    mutex->lock();

    if (desired_joint_state.position.size() == DOF_JOINTS) {
      // Control joint positions: compute the desired torques (PD control).
      double error;
      for (int i = 0; i < DOF_JOINTS; i++) {
//        error = desired_joint_state.position[i] - current_position_filtered[i];
        error = desired_joint_state.position[i] - current_position[i];
        desired_torque[i] = 1.0/canDevice->torqueConversion() *
                (k_p[i] * error - k_d[i] * current_velocity[i]);
        desired_torque[i] = max(min(desired_torque[i], 0.5), -0.5);
      }
    } else if (desired_joint_state.effort.size() > 0) {
      // Control joint torques: set desired torques as the value stored in the
      // desired_joint_state message.
      for (int i = 0; i < DOF_JOINTS; i++) {
        desired_torque[i] = desired_joint_state.effort[i];
      }
    }
    mutex->unlock();
  }
}

void AllegroNodePD::initController(const std::string &whichHand) {
  // set gains_pd via gains_pd.yaml or to default values
  for (int i = 0; i < DOF_JOINTS; i++) {
    declare_parameter(pGainParams[i], k_p[i]);
    declare_parameter(dGainParams[i], k_d[i]);
  }

  for (int i = 0; i < DOF_JOINTS; i++) {
    k_p[i] = get_parameter(pGainParams[i]).as_double();
    k_d[i] = get_parameter(dGainParams[i]).as_double();
    RCLCPP_INFO(get_logger(), "gains_p[%d]=%f %f", i,  k_p[i], k_d[i]);
  }

  // set initial position via initial_position.yaml or to default values
  for (int i = 0; i < DOF_JOINTS; i++) {
    declare_parameter(initialPosition[i], home_pose[i]);
  }

  double tmp;
  mutex->lock();
  desired_joint_state.position.resize(DOF_JOINTS);
  for (int i = 0; i < DOF_JOINTS; i++) {
    tmp = get_parameter(initialPosition[i]).as_double();
    desired_joint_state.position[i] = DEGREES_TO_RADIANS(tmp);
    RCLCPP_INFO(get_logger(), "home_pose [%d]=%f", i,  tmp);
  }
  mutex->unlock();

  control_hand_ = false;

  printf("*************************************\n");
  printf("      Joint PD Control Method        \n");
  printf("-------------------------------------\n");
  printf("  Only 'H', 'O', 'S', 'Space' works. \n");
  printf("*************************************\n");
}

void AllegroNodePD::doIt(bool polling) {
  // Main spin loop, uses the publisher/subscribers.
  auto this_node = std::shared_ptr<AllegroNodePD>(this);
  rclcpp::Rate rate(300);
  if (polling) {
    RCLCPP_INFO(get_logger(), "Polling = true.");
    while (rclcpp::ok()) {
      updateController();
      rclcpp::spin_some(this_node);
      rate.sleep();
    }
  } else {
    RCLCPP_INFO(get_logger(), "Polling = false.");

    // Timer callback (not recommended).
    rclcpp::TimerBase::SharedPtr timer = startTimerCallback();
    rclcpp::spin(this_node);
  }
}

int main(int argc, char **argv) {
  auto clean_argv = rclcpp::init_and_remove_ros_arguments(argc, argv); 

  bool polling = false;
  if (clean_argv.size() > 1 && clean_argv[1] == std::string("true")) {
    polling = true;
  }

  bool is_sim = std::find(clean_argv.begin(), clean_argv.end(), "--sim") != clean_argv.end();
  AllegroNodePD allegroNode("allegro_node_pd", is_sim);
  allegroNode.doIt(polling);
}
