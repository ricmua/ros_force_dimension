/** Functionality related to ROS2 parameters.
 */

/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */


// Include guard.
#ifndef FORCE_DIMENSION_PARAMETERS_H_
#define FORCE_DIMENSION_PARAMETERS_H_

// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"


/** Parameter change callback.
 *  
 *  This function is called any time a ROS2 parameter value changes.
 */
rcl_interfaces::msg::SetParametersResult 
force_dimension::Node::set_parameters_callback
(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if(parameter.get_name()  == "effector_mass_kg")
        set_effector_mass(parameter.as_double()); //get_value()
    if(parameter.get_name()  == "gravity_compensation")
        set_gravity_compensation(parameter.as_bool());
    if(parameter.get_name()  == "enable_force")
        set_enable_force(parameter.as_bool());
  }
  return result;
}


#endif // FORCE_DIMENSION_PARAMETERS_H_

