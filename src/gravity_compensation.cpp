/** Functionality related to publishing the Force Dimension gravity 
 *  compensation functionality.
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
#ifndef FORCE_DIMENSION_GRAVITY_COMPENSATION_H_
#define FORCE_DIMENSION_GRAVITY_COMPENSATION_H_

// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

// Import the Force Dimension haptics library.
#include "dhdc.h"


/** Get the currently-configured mass of the robot end effector.
 */
double force_dimension::Node::get_effector_mass() {
  
  double mass_kg = -1;
  int result = hardware_disabled_ 
             ? 0 
             : dhdGetEffectorMass(&mass_kg, device_id_);
  if(result != 0) {
      std::string message = "Failed to get effector mass: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  
  // Return the result.
  return mass_kg;
}


/** Set the configured mass of the robot end effector.
 */
void force_dimension::Node::set_effector_mass(double mass_kg) {
  
  // Get the effector mass from parameter, if not specified.
  if(mass_kg == -1) {
      bool success = get_parameter("effector_mass_kg", mass_kg);
      if(!success) {
          Log("Unable to get parameter value");
          on_error();
      }
  }
  
  // Set the effector mass.
  int result = hardware_disabled_ 
             ? 0 
             : dhdSetEffectorMass(mass_kg, device_id_);
  if(result != 0) {
      std::string message = "Failed to get effector mass: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  else {
      std::string message = "Effector mass set to ";
      //message += std::to_string(mass_kg);
      message += std::to_string(get_effector_mass());
      message += " kg.";
      Log(message);
  }
    
  return;
}


/** Set the configured mass of the robot end effector to the baseline value 
 *  recorded at startup.
 */
void force_dimension::Node::reset_effector_mass(void) {
  
  // Reset the effector mass to the baseline value.
  set_effector_mass(baseline_effector_mass_kg_);
  
}


/** Enable or disable gravity compensation.
 */
void force_dimension::Node::set_gravity_compensation(bool enable) {
  
  int val = enable ? DHD_ON : DHD_OFF;
  int result = hardware_disabled_ 
             ? 0 
             : dhdSetGravityCompensation(val, device_id_);
  if(result != 0) {
      std::string message = "Failed to set gravity compensation";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  else
  {
      // Report the action.
      auto message = enable 
                   ? "Gravity compensation: Enabled" 
                   : "Gravity compensation: Disabled";
      Log(message);
  }
  
}



/** Enable or disable gravity compensation using the parameter value.
 */
void force_dimension::Node::set_gravity_compensation() {
  bool enable;
  bool success = get_parameter("gravity_compensation", enable);
  if(!success) {
      Log("Unable to get parameter value");
      on_error();
  }
  else set_gravity_compensation(enable);
  
}
  
#endif // FORCE_DIMENSION_GRAVITY_COMPENSATION_H_

