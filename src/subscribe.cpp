/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to receiving ROS2 messages.


// Import node header.
#include "node.hpp"

// Import the Force Dimension haptics header.
#include "dhdc.h"

// Import package headers.
#include "messages.hpp"
#include "topics.hpp"
#include "qos.hpp"


// Scope namespace elements.
using force_dimension::Node;


// Subscribes to ROS messages that indicate an instantaneous force to be 
// applied by the robot.
void Node::SubscribeForce(void) {
  auto callback = [this](ForceMessage m) { this->force_callback(m); };
  auto topic = FORCE_COMMAND_TOPIC;
  auto qos = DefaultQoS();
  force_subscription_ \
      = this->create_subscription<ForceMessage>(topic, qos, callback);
}


// Applies a force to the robotic manipulandum, as requested via ROS message.
void Node::force_callback(const ForceMessage message) {
  auto result = hardware_disabled_ 
              ? 0
              : dhdSetForce(message.x, message.y, message.z, device_id_);
  if((result != 0) & (result != DHD_MOTOR_SATURATED)) {
      std::string message = "Cannot set force: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
}


/** Subscribes to ROS messages that specify the configured mass of the 
 *  endpoint effector that is to be used for gravity compensation.
 */
void Node::SubscribeMass(void) {
  auto callback = [this](MassMessage m) { this->mass_callback(m); };
  auto topic = MASS_COMMAND_TOPIC;
  auto qos = DefaultQoS();
  mass_subscription_ \
      = this->create_subscription<MassMessage>(topic, qos, callback);
}


/** Adjusts the configured mass parameter that is to be used for gravity 
 *  compensation.
 */
void Node::mass_callback(const MassMessage message) {
  rclcpp::Parameter mass("effector_mass_kg", message.data);
  rcl_interfaces::msg::SetParametersResult result = set_parameter(mass);
  if(!result.successful) {
      std::string message = "Failed to set parameter: ";
      message += result.reason;
      Log(message);
      on_error();
  }
}


