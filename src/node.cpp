/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Implementation of an interface between ROS2 and the Force Dimension SDK for 
// haptic robots.



// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

// Import the Force Dimension haptics library.
#include "dhdc.h"

// Import package headers.
#include "qos.hpp"
#include "topics.hpp"


// Select namespace.
//using namespace force_dimension;
using force_dimension::Node;


// Constructor.
Node::Node(bool configure=true, bool activate=true) 
  : rclcpp::Node("force_dimension", "robot")
{
  // Initialize default values.
  device_id_ = -1;
  active_ = false;
  configured_ = false;
  
  // Configure and activate, if requested.
  configure = activate ? true : configure;
  if(configure) on_configure();
  if(activate) on_activate();
}


// Destructor.
Node::~Node() {
  if(active_) on_deactivate();
  if(configured_) on_cleanup();
}

// Configures the ROS node by creating publishers, subscriptions, and 
// initializing parameters.
void Node::on_configure(void) {
  
  // Use the default ROS2 Quality-of-Service.
  auto qos = DefaultQoS();
  
  // Create the pose state publisher.
  auto topic = POSE_FEEDBACK_TOPIC;
  pose_publisher_ = create_publisher<PoseMessage>(topic, qos);
  
  // Create the button state publisher.
  topic = BUTTON_FEEDBACK_TOPIC;
  button_publisher_ = create_publisher<ButtonMessage>(topic, qos);
  
  // Create the gripper gap state publisher.
  topic = GRIPPER_GAP_FEEDBACK_TOPIC;
  gripper_gap_publisher_ = create_publisher<GripperGapMessage>(topic, qos);
  
  // Create the gripper angle state publisher.
  topic = GRIPPER_ANGLE_FEEDBACK_TOPIC;
  gripper_angle_publisher_ = create_publisher<GripperAngleMessage>(topic, qos);
  
  // Create the velocity state publisher.
  topic = TWIST_FEEDBACK_TOPIC;
  twist_publisher_ = create_publisher<TwistMessage>(topic, qos);
  
  //// Create the force state publisher.
  //topic = FORCE_FEEDBACK_TOPIC;
  //force_publisher = create_publisher<force_message>(topic, qos);
  
  // Initialize ROS2 parameters.
  declare_parameter<float>("sample_interval_s", 0.025);
  declare_parameter<bool>("disable_hardware", false);
  declare_parameter<bool>("gripper.emulate_button", false);
  declare_parameter<int>("feedback_sample_decimation.pose", 50);
  declare_parameter<int>("feedback_sample_decimation.twist", 50);
  declare_parameter<int>("feedback_sample_decimation.button", 50);
  declare_parameter<int>("feedback_sample_decimation.gripper_gap", 50);
  declare_parameter<int>("feedback_sample_decimation.gripper_angle", 50);
  declare_parameter<float>("effector_mass_kg", 0.190000);
  declare_parameter<bool>("gravity_compensation", true);
  declare_parameter<bool>("enable_force", true);

  // Create the force control subcription.
  SubscribeForce();
}


/** Activates the ROS node by initializing the Force Dimension interface and 
 *  the publication timer / callback.
 */
void Node::on_activate(void) {
  
  // Check to see if hardware has been disabled.
  // This is done once, at the time of activation, and stored in a member 
  // variable while active.
  get_parameter("disable_hardware", hardware_disabled_);
  
  // Open the first available Force Dimension device.
  Log("Initializing the Force Dimension interface.");
  device_id_ = hardware_disabled_ ? 999 : dhdOpen();
  if(device_id_ < 0) {
      std::string message = "Cannot open Force Dimension device: ";
      message += dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  else {
      std::string message = "Force Dimension device detected: ";
      message += hardware_disabled_ ? "hardware disabled" : dhdGetSystemName();
      Log(message);
  }
  
  // Enable button emulation, if requested.
  unsigned char val = get_parameter("gripper.emulate_button").as_bool()
                    ? DHD_ON : DHD_OFF;
  int result = hardware_disabled_ ? 0 : dhdEmulateButton(val, device_id_);
  if(result != 0) {
      std::string message = "Button emulation failure: ";
      //message += dhdErrorGetLastStr();
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  
  // Apply zero force.
  result = hardware_disabled_ 
         ? DHD_NO_ERROR 
         : dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 
                                               0.0, 0.0, 0.0, 0.0);
  if(result < DHD_NO_ERROR) {
      std::string message = "Cannot set force: ";
      //message += dhdErrorGetLastStr();
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  else Log("Force Dimension interface initialized.");
  
  // Report the BASELINE communication refresh rate.
  result = hardware_disabled_ ? -1.0 : dhdGetComFreq(device_id_);
  if(result == 0.0) {
      std::string message = "Failure getting communication refresh rate: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
  }
  else
  {
      std::string message = "Communication refresh rate: ";
      message += std::to_string(result);
      message += " kHz";
      Log(message);
  }
  
  // Initialize the publication interval variable and create the timer.
  // Sets the ROS publication interval for state messages.
  // The sampling rate should be set low enough that any receiving nodes 
  // (e.g., a GUI), as well as the ROS system, can handle the message volume.
  auto callback = [this]() { this->PublishState(); };
  float sample_interval_s;
  if(get_parameter("sample_interval_s", sample_interval_s))
  {
    int publication_interval_ns = round(sample_interval_s * 1e9);
    std::chrono::nanoseconds publication_interval(publication_interval_ns);
    timer_ = create_wall_timer(publication_interval, callback);
    std::string message = "Sample timer initialized: Interval (ms) = ";
    message += std::to_string(publication_interval_ns * (1e3 / 1e9));
    Log(message);
  }
  else
  {
    Log("Failed to initialize sample timer.");
    on_error();
  }
  
  // Reset the sample counter.
  sample_number_ = 0;
  
  // Get baseline effector mass to be used for gravity compensation.
  baseline_effector_mass_kg_ = get_effector_mass();
  {
      std::string message = "BASELINE EFFECTOR MASS: ";
      message += std::to_string(baseline_effector_mass_kg_);
      message += " kg";
      Log(message);
  }
  
  // Set effector mass to be used for gravity compensation.
  // Defaults to the ROS parameter value.
  set_effector_mass();
  
  // Enable gravity compensation.
  // Defaults to the ROS parameter value.
  set_gravity_compensation();
  
  // Add a set parameters callback.
  // Initialize a function pointer to the set_parameters_callback member with 
  // one argument placeholder (for the parameter vector).
  auto parameters_callback 
    = std::bind(&Node::set_parameters_callback, this, std::placeholders::_1);
  parameters_callback_handle_
    = this->add_on_set_parameters_callback(parameters_callback);
}


/** 
 *  
 */
void Node::on_deactivate(void) {
  
  // Stop the publication timer.
  timer_->cancel();
  //timer_->destroy();
  
  // Close the connection to the Force Dimension device.
  Log("Shutting the Force Dimension interface down.");
  auto result = hardware_disabled_ ? DHD_NO_ERROR : dhdClose();
  if(result == DHD_NO_ERROR)
    Log("Force Dimension interface closed.");
  else {
    std::string message = "Unable to close the device connection: ";
    //message += dhdErrorGetLast();
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }
  
}

/** 
 *  
 */
void Node::on_cleanup(void) {
}

// Error handling callback de-activates and cleans up.
void Node::on_error(void) {
  if(active_) on_deactivate();
  if(configured_) on_cleanup();
}




