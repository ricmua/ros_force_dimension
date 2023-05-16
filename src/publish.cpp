/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to publishing ROS2 messages.


// Include guard.
#ifndef FORCE_DIMENSION_PUBLISH_H_
#define FORCE_DIMENSION_PUBLISH_H_

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
  PublishButton();
  PublishGripperGap();
  PublishGripperAngle();
  PublishVelocity();
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


/** Publish button events.
 *  
 */
void force_dimension::Node::PublishButton() {
  
  // Read the button mask.
  int result = hardware_disabled_ ? 0 : dhdGetButtonMask(device_id_);
  
  // Prepare a button message.    
  auto message = ButtonMessage();
  message.data = result;
  
  // Publish.
  if(IsPublishableSample("button")) button_publisher_->publish(message);
}


/** Publish gripper opening distance in meters.
 *  
 */
void force_dimension::Node::PublishGripperGap() {
  
  // Read the gripper gap.
  double gap = -1;
  bool has_gripper = hardware_disabled_ ? false : dhdHasGripper(device_id_);
  int result = has_gripper ? dhdGetGripperGap(&gap, device_id_) : 0;
  if((result != 0) & (result != DHD_TIMEGUARD))  {
      std::string message = "Failed to read gripper gap: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  
  // Prepare a gripper gap message.
  // ROS2 messages have "no constructor with positional arguments for the 
  // members".
  auto message = GripperGapMessage();
  message.data = gap;
  
  // Publish.
  if(IsPublishableSample("gripper_gap"))
    gripper_gap_publisher_->publish(message);
}


/** Publish gripper opening angle in radians.
 *  
 */
void force_dimension::Node::PublishGripperAngle() {
  
  // Read the gripper angle.
  double angle = -1;
  bool has_gripper = hardware_disabled_ ? false : dhdHasGripper(device_id_);
  int result = has_gripper ? dhdGetGripperAngleRad(&angle, device_id_) : 0;
  if(result != 0)  {
      std::string message = "Failed to read gripper angle: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  
  // Prepare a gripper angle message.
  // ROS2 messages have "no constructor with positional arguments for the 
  // members".
  auto message = GripperAngleMessage();
  message.data = angle;
  
  // Publish.
  if(IsPublishableSample("gripper_angle"))
    gripper_angle_publisher_->publish(message);
}


/** Publish the velocity of the robotic end-effector.
 *  
 */
void force_dimension::Node::PublishVelocity() {
  
  // Retrieve the velocity.
  double vx, vy, vz;
  auto result = hardware_disabled_ 
              ? DHD_NO_ERROR 
              : dhdGetLinearVelocity(&vx, &vy, &vz);
  if(result < DHD_NO_ERROR)  {
      std::string message = "Failed to read velocity: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  
  // Prepare a message.
  auto message          = VelocityMessage();
  message.x             = vx;
  message.y             = vy;
  message.z             = vz;
  //message.sample_number = sample_number;
  
  // Publish.
  if(IsPublishableSample("velocity")) velocity_publisher_->publish(message);
}



