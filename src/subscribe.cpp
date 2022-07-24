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


// Subscribes to ROS messages that indicate that an instantaneous force is to 
// be applied to the robot endpoint.
void Node::SubscribeForce(void) {
  auto callback = [this](ForceMessage m) { this->force_callback(m); };
  auto topic = FORCE_COMMAND_TOPIC;
  auto qos = DefaultQoS();
  force_subscription_ \
      = this->create_subscription<ForceMessage>(topic, qos, callback);
}


// Applies a force to the robotic manipulandum, as requested via ROS message.
void Node::force_callback(const ForceMessage message) {
  
  // Update the current state.
  current_endpoint_force_[0] = message.x;
  current_endpoint_force_[1] = message.y;
  current_endpoint_force_[2] = message.z;
  
  // Set the endpoint force, if hardware is enabled.
  auto result = hardware_disabled_ 
              ? 0
              : dhdSetForce(message.x, message.y, message.z, device_id_);
  
  // Check for errors.
  if((result != 0) & (result != DHD_MOTOR_SATURATED)) {
      std::string message = "Cannot set force: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
}


// Subscribes to ROS messages that indicate that an instantaneous force is to 
// be applied to the robotic gripper.
void Node::SubscribeGripperForce(void) {
  auto callback = [this](GripperForceMessage m) 
    { this->gripper_force_callback(m); };
  auto topic = GRIPPER_FORCE_COMMAND_TOPIC;
  auto qos = DefaultQoS();
  gripper_force_subscription_ \
      = this->create_subscription<GripperForceMessage>(topic, qos, callback);
}


/** Apply a force to the robotic gripper, as requested via ROS message.
 *  
 *  Since the Force Dimension SDK does not decouple gripper force commands from 
 *  endpoint force commands, this callback function also sets the latter. The 
 *  last commanded value is used for the endpoint force. This might not be the 
 *  appropriate solution in every case.
 */ 
void Node::gripper_force_callback(const GripperForceMessage message) {
  
  // Update the current state.
  current_gripper_force_ = message.data;
  
  // Set the gripper force, if hardware is enabled.
  auto result = hardware_disabled_ 
              ? 0
              : dhdSetForceAndGripperForce(current_endpoint_force_[0],
                                           current_endpoint_force_[1],
                                           current_endpoint_force_[2],
                                           message.data,
                                           device_id_);
  
  // Check for errors.
  if((result != 0) & (result != DHD_MOTOR_SATURATED)) {
      std::string message = "Cannot set force: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
}



