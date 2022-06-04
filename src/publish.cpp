/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to publishing ROS2 messages.


// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

// Import the Force Dimension haptics library.
#include "dhdc.h"

// Import package headers.
#include "messages.hpp"

/** 
 *  
 *  
 *  
 *  
 */
 
/** Publish state feedback. The state consists of the position, velocity, and 
 *  force.
 */
void force_dimension::Node::PublishState() {
  PublishPosition();
  //publish_velocity();
  //publish_force();
}

/** Publish the position of the robotic end-effector.
 *  
 */
void force_dimension::Node::PublishPosition() {    
  double px, py, pz;
  if(dhdGetPosition(&px, &py, &pz) < DHD_NO_ERROR)  {
      std::string message = "Failed to read position: ";
      message += dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  
  auto message          = PositionMessage();
  message.x             = px;
  message.y             = py;
  message.z             = pz;
  //message.sample_number = sample_number;
  
  position_publisher_->publish(message);
}

