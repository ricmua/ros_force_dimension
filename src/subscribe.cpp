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
void Node::SubscribeWrench(void) {
  auto callback = [this](WrenchMessage m) { this->wrench_callback(m); };
  auto topic = WRENCH_COMMAND_TOPIC;
  auto qos = DefaultQoS();
  wrench_subscription_ \
      = this->create_subscription<WrenchMessage>(topic, qos, callback);
}


// Applies a wrench to the robotic manipulandum, as requested via ROS message.
void Node::wrench_callback(const WrenchMessage message) {
  auto f = message.force;
  auto t = message.torque;

  auto result = hardware_disabled_ 
              ? 0
              : dhdSetForceAndTorque(f.x, f.y, f.z, t.x, t.y, t.z, device_id_);
  if((result != 0) && (result != DHD_MOTOR_SATURATED)) {
      std::string message = "Cannot set wrench: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
}
