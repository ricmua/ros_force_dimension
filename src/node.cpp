/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Implementation of an interface between ROS2 and the Force Dimension SDK for 
// haptic robots.



// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

// Import the Force Dimension haptics library.
#include "dhdc.h"

// Import package headers.
#include "qos.hpp"
#include "topics.hpp"


// Select namespace.
//using namespace force_dimension;
using force_dimension::Node;


// Constructor.
Node::Node(bool configure=true, bool activate=true) 
  : rclcpp::Node("force_dimension", "robot")
{
  // Initialize default values.
  device_id_ = -1;
  active_ = false;
  configured_ = false;
  
  // Configure and activate, if requested.
  configure = activate ? true : configure;
  if(configure) on_configure();
  if(activate) on_activate();
}


// Destructor.
Node::~Node() {
  if(active_) on_deactivate();
  if(configured_) on_cleanup();
}

// Configures the ROS node by creating publishers, subscriptions, and 
// initializing parameters.
void Node::on_configure(void) {
  
  // Use the default ROS2 Quality-of-Service.
  auto qos = DefaultQoS();
  
  // Create the position state publisher.
  auto topic = POSITION_FEEDBACK_TOPIC;
  position_publisher_ = create_publisher<PositionMessage>(topic, qos);
  
  //// Create the velocity state publisher.
  //topic = VELOCITY_FEEDBACK_TOPIC;
  //velocity_publisher = create_publisher<velocity_message>(topic, qos);
  
  //// Create the force state publisher.
  //topic = FORCE_FEEDBACK_TOPIC;
  //force_publisher = create_publisher<force_message>(topic, qos);
  
  // Initialize ROS2 parameters.
  declare_parameter<float>("sample_interval_s", 0.025);
  declare_parameter<bool>("disable_hardware", false);
  declare_parameter<int>("feedback_sample_decimation.position", 1);
  
  // Create the force control subcription.
  SubscribeForce();
}


/** Activates the ROS node by initializing the Force Dimension interface and 
 *  the publication timer / callback.
 */
void Node::on_activate(void) {
  
  // Check to see if hardware has been disabled.
  get_parameter("disable_hardware", hardware_disabled_);
  
  // Open the first available Force Dimension device.
  Log("Initializing the Force Dimension interface.");
  device_id_ = hardware_disabled_ ? 999 : dhdOpen();
  if(device_id_ < 0) {
      std::string message = "Cannot open Force Dimension device: ";
      message += dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  else {
      std::string message = "Force Dimension device detected: ";
      message += hardware_disabled_ ? "hardware disabled" : dhdGetSystemName();
      Log(message);
  }
  
  // Apply zero force.
  auto result = hardware_disabled_ 
              ? DHD_NO_ERROR 
              : dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0);
  if(result < DHD_NO_ERROR) {
      std::string message = "Cannot set force: ";
      //message += dhdErrorGetLastStr();
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
      on_error();
  }
  else Log("Force Dimension interface initialized.");
  
  // Initialize the publication interval variable and create the timer.
  // Sets the ROS publication interval for state messages.
  // The sampling rate should be set low enough that any receiving nodes 
  // (e.g., a GUI), as well as the ROS system, can handle the message volume.
  auto callback = [this]() { this->PublishState(); };
  float sample_interval_s;
  if(get_parameter("sample_interval_s", sample_interval_s))
  {
    int publication_interval_ms = round(sample_interval_s * 1000);
    std::chrono::milliseconds publication_interval(publication_interval_ms);
    timer_ = create_wall_timer(publication_interval, callback);
    std::string message = "Sample timer initialized: Interval (ms) = ";
    message += std::to_string(publication_interval_ms);
    Log(message);
  }
  else
  {
    Log("Failed to initialize sample timer.");
    on_error();
  }
  
  // Reset the sample counter.
  sample_number_ = 0;
}


/** 
 *  
 */
void Node::on_deactivate(void) {
  
  // Stop the publication timer.
  timer_->cancel();
  //timer_->destroy();
  
  // Close the connection to the Force Dimension device.
  Log("Shutting the Force Dimension interface down.");
  auto result = hardware_disabled_ ? DHD_NO_ERROR : dhdClose();
  if(result == DHD_NO_ERROR)
    Log("Force Dimension interface closed.");
  else {
    std::string message = "Unable to close the device connection: ";
    //message += dhdErrorGetLast();
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }
  
}

/** 
 *  
 */
void Node::on_cleanup(void) {
}

// Error handling callback de-activates and cleans up.
void Node::on_error(void) {
  if(active_) on_deactivate();
  if(configured_) on_cleanup();
}




