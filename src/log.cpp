/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to logging.


// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"


// Declare namespace.
namespace force_dimension {
  
  // Logging function.
  void force_dimension::Node::Log(std::string message) {
     RCLCPP_INFO(get_logger(), message.c_str());
  }
  
} // namespace force_dimension



