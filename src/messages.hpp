/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// ROS2 message definitions.


// Include guard.
#ifndef FORCE_DIMENSION_MESSAGES_H_
#define FORCE_DIMENSION_MESSAGES_H_


// Import message types.
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "example_interfaces/msg/float64.hpp"


// Declare namespace.
namespace force_dimension {

  // Message type definitions.

  /** Effector position ROS message type definition.
   *  
   */
  typedef geometry_msgs::msg::Point PositionMessage;

  /** Effector velocity ROS message type definition.
   *  
   */
  typedef geometry_msgs::msg::Vector3 VelocityMessage;

  /** Effector force ROS message type definition.
   *  
   */
  typedef geometry_msgs::msg::Vector3 ForceMessage;

  /** Event ROS message type definition.
   *  
   */
  typedef example_interfaces::msg::Int32 ButtonMessage;

  /** Gripper gap ROS message type definition.
   *  
   */
  typedef example_interfaces::msg::Float64 GripperGapMessage;

  /** Gripper angle ROS message type definition.
   *  
   */
  typedef example_interfaces::msg::Float64 GripperAngleMessage;

  /** Endpoint effector mass message.
   *  
   */
  typedef example_interfaces::msg::Float64 MassMessage;

} // namespace force_dimension


#endif // FORCE_DIMENSION_MESSAGES_H_
