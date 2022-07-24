/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Interface between ROS2 and the Force Dimension SDK for haptic robots.


// Include guard.
#ifndef FORCE_DIMENSION_TOPICS_H_
#define FORCE_DIMENSION_TOPICS_H_



/** Topic for publishing robot position feedback.
 *  
 */

namespace force_dimension {

  template <typename MessageT>
  struct topic_data
  {
    char name;
    MessageT message;
    
  };

  /** Topic for publishing robot endpoint position feedback.
   *  
   */
  const char POSITION_FEEDBACK_TOPIC[] = "feedback/position";

  /** Topic for publishing button press feedback.
   *  
   */
  const char BUTTON_FEEDBACK_TOPIC[] = "feedback/button";

  /** Topic for publishing robot endpoint velocity feedback.
   *  
   */
  const char VELOCITY_FEEDBACK_TOPIC[] = "feedback/velocity";

  /** Topic for publishing robot endpoint force feedback.
   *  
   */
  const char FORCE_FEEDBACK_TOPIC[] = "feedback/force";

  /** Topic for subscribing to robot endpoint force commands.
   *  
   */
  const char FORCE_COMMAND_TOPIC[] = "command/force";
  
  /** Topic for gripper gap width feedback.
   *  
   */
  const char GRIPPER_GAP_FEEDBACK_TOPIC[] = "feedback/gripper_gap";

  /** Topic for gripper angle feedback.
   *  
   */
  const char GRIPPER_ANGLE_FEEDBACK_TOPIC[] = "feedback/gripper_angle";

  /** Topic for gripper force feedback.
   *  
   */
  const char GRIPPER_FORCE_FEEDBACK_TOPIC[] = "feedback/gripper_force";

  /** Topic for subscribing to robot gripper force commands.
   *  
   */
  const char GRIPPER_FORCE_COMMAND_TOPIC[] = "command/gripper_force";
  
} // namespace force_dimension


#endif // FORCE_DIMENSION_TOPICS_H_

