/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// ROS2 Quality-of-Service definitions.


// Include guard.
#ifndef FORCE_DIMENSION_QOS_H_
#define FORCE_DIMENSION_QOS_H_


// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"


// Declare namespace.
namespace force_dimension {

  // Define a default QoS type.
  typedef rclcpp::SystemDefaultsQoS DefaultQoS;

} // namespace force_dimension


#endif // FORCE_DIMENSION_QOS_H_

