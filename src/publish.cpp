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
  sample_number_++;
  PublishPosition();
  //publish_velocity();
  //publish_force();
  //publish_button();
  //publish_orientation();
}

/** Publish the position of the robotic end-effector.
 *  
 */
void force_dimension::Node::PublishPosition() {
  
  // Retrieve the position.
  double px, py, pz;
  auto result = hardware_disabled_ 
              ? DHD_NO_ERROR 
              : dhdGetPosition(&px, &py, &pz);
  if(result < DHD_NO_ERROR)  {
      std::string message = "Failed to read position: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  
  auto message          = PositionMessage();
  message.x             = px;
  message.y             = py;
  message.z             = pz;
  //message.sample_number = sample_number;
  
  // Publish.
  if(IsPublishableSample("position")) position_publisher_->publish(message);
}


/** Check whether or not the current data sample should be published.
 *  
 */
bool force_dimension::Node::IsPublishableSample(std::string parameter_name) {
  
  // Decide whether or not to publish based on decimation of the sample counter.
  std::string parameter_path = "feedback_sample_decimation." + parameter_name;
  //int decimation_divisor;
  rclcpp::Parameter parameter = get_parameter(parameter_path); //, decimation_divisor);
  int decimation_divisor = parameter.as_int();
  bool publish = (decimation_divisor > 0)
               ? ((sample_number_ % decimation_divisor) == 0)
               : false;
  return publish;
}

