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

// Import ROS tf2 for transform math
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
 
/** Publish state feedback. The state consists of the pose, velocity, and 
 *  force.
 */
void force_dimension::Node::PublishState() {
  sample_number_++;
  PublishPose();
  PublishButton();
  PublishGripperGap();
  PublishGripperAngle();
  PublishTwist();
  //publish_velocity();
  //publish_force();
  //publish_button();
}

/** Publish the pose of the robotic end-effector.
 *  
 */
void force_dimension::Node::PublishPose() {
  
  // Get the device pose
  double px, py, pz;
  double rot[3][3];
  auto result = hardware_disabled_ 
              ? DHD_NO_ERROR 
              : dhdGetPositionAndOrientationFrame(&px, &py, &pz, rot);
  if(result < DHD_NO_ERROR)  {
      std::string message = "Failed to read pose: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }

  // Convert rotation matrix to quaternion
  tf2::Matrix3x3 tf2_rot(
    rot[0][0], rot[0][1], rot[0][2],
    rot[1][0], rot[1][1], rot[1][2],
    rot[2][0], rot[2][1], rot[2][2]
  );
  tf2::Quaternion q;
  tf2_rot.getRotation(q);

  
  // Populate message fields
  auto message          = PoseMessage();

  message.position.x    = px;
  message.position.y    = py;
  message.position.z    = pz;

  message.orientation   = tf2::toMsg(q);
  
  // Publish.
  if(IsPublishableSample("pose")) pose_publisher_->publish(message);
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


/** Publish the twist of the robotic end-effector.
 *  
 */
void force_dimension::Node::PublishTwist() {
  
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
  
  // Get angular velocity
  double ax, ay, az;
  result = hardware_disabled_ 
              ? DHD_NO_ERROR 
              : dhdGetAngularVelocityRad(&ax, &ay, &az);
  if(result < DHD_NO_ERROR)  {
      std::string message = "Failed to read angular velocity: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  
  // Prepare a message.
  auto message          = TwistMessage();
  message.linear.x      = vx;
  message.linear.y      = vy;
  message.linear.z      = vz;
  
  message.angular.x     = ax;
  message.angular.y     = ay;
  message.angular.z     = az;
  
  // Publish.
  if(IsPublishableSample("twist")) twist_publisher_->publish(message);
}


#endif  //FORCE_DIMENSION_PUBLISH_H_

