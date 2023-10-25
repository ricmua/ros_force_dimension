/** Functionality related to forces.
 */

/** Copyright 2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */



// Include guard.
#ifndef FORCE_DIMENSION_FORCES_H_
#define FORCE_DIMENSION_FORCES_H_

// Import the node header.
#include "node.hpp"

// Import the Force Dimension haptics library.
#include "dhdc.h"


/** Enable or disable forces.
 */
void force_dimension::Node::set_enable_force(bool enable) {
  
  int val = enable ? DHD_ON : DHD_OFF;
  int result = hardware_disabled_ ? 0 : dhdEnableForce(val, device_id_);
  if(result != 0) {
      std::string message = "Failed to enable forces";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  else
  {
      // Report the action.
      auto message = enable ? "Forces: Enabled" : "Forces: Disabled";
      Log(message);
  }
  
}



/** Enable or disable forces using the parameter value.
 */
void force_dimension::Node::set_enable_force() {
  bool enable;
  bool success = get_parameter("enable_force", enable);
  if(!success) {
      Log("Unable to get parameter value");
      on_error();
  }
  else set_enable_force(enable);
  
}
  
#endif // FORCE_DIMENSION_FORCES_H_

