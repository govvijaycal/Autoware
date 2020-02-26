/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "JoystickDemo.h"

namespace joystick_demo
{

JoystickDemo::JoystickDemo(ros::NodeHandle &node, ros::NodeHandle &priv_nh) : counter_(0)
{
  joy_.axes.resize(AXIS_COUNT, 0);
  joy_.buttons.resize(BTN_COUNT, 0);

  ignore_ = false;
  count_ = false;
  svel_ = 0.0;
  priv_nh.getParam("ignore", ignore_);
  priv_nh.getParam("count", count_);
  priv_nh.getParam("svel", svel_);

  sub_joy_ = node.subscribe("/joy", 1, &JoystickDemo::recvJoy, this);

  data_.brake_joy = 0.0;
  data_.steering_joy = 0.0;
  data_.steering_mult = false;
  data_.throttle_joy = 0.0;
  data_.joy_throttle_valid = false;
  data_.joy_brake_valid = false;

  pub_accel_ = node.advertise<std_msgs::Float32>("control/accel", 1);
  pub_steering_ = node.advertise<std_msgs::Float32>("control/steer_angle", 1);
  pub_enable_accel_ = node.advertise<std_msgs::UInt8>("control/enable_accel", 1);
  pub_enable_spas_ = node.advertise<std_msgs::UInt8>("control/enable_spas", 1);
  // TODO create a button interface to select LKAS or SPAS
  // LKAS is not currently set up to work with the joystick
  pub_enable_lkas_ = node.advertise<std_msgs::UInt8>("control/enable_lkas", 1);

  timer_ = node.createTimer(ros::Duration(0.02), &JoystickDemo::cmdCallback, this);
}

void JoystickDemo::cmdCallback(const ros::TimerEvent& event)
{
  // Detect joy timeouts and reset
  if (event.current_real - data_.stamp > ros::Duration(0.1)) {
    data_.joy_throttle_valid = false;
    data_.joy_brake_valid = false;
    return;
  }

  // Optional watchdog counter
  if (count_) {
    counter_++;
  }

  // Accel and steering commands
  float accel_cmd = 0;
  if (data_.brake_joy > 0) {
    accel_cmd = -5.0 * data_.brake_joy;
  } else {
    accel_cmd = 4.0 * data_.throttle_joy;
  }
  pub_accel_.publish(accel_cmd);
  pub_steering_.publish(data_.steering_joy);

  // TODO add some safety features like count, etc to our controller - vehicle
  // interface layer to detect controller crashes and return control to the
  // driver automatically
  // Throttle
  /* dbw_mkz_msgs::ThrottleCmd throttle_msg; */
  /* throttle_msg.enable = true; */
  /* throttle_msg.ignore = ignore_; */
  /* throttle_msg.count = counter_; */
  /* throttle_msg.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT; */
  /* throttle_msg.pedal_cmd = data_.throttle_joy; */
  /* pub_throttle_.publish(throttle_msg); */
}

void JoystickDemo::recvJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Check for expected sizes
  if (msg->axes.size() != (size_t)AXIS_COUNT) {
    ROS_ERROR("Expected %zu joy axis count, received %zu", (size_t)AXIS_COUNT, msg->axes.size());
    return;
  }
  if (msg->buttons.size() != (size_t)BTN_COUNT) {
    ROS_ERROR("Expected %zu joy button count, received %zu", (size_t)BTN_COUNT, msg->buttons.size());
    return;
  }

  // Handle joystick startup
  if (msg->axes[AXIS_THROTTLE] != 0.0) {
    data_.joy_throttle_valid = true;
  }
  if (msg->axes[AXIS_BRAKE] != 0.0) {
    data_.joy_brake_valid = true;
  }

  // Throttle
  if (data_.joy_throttle_valid) {
    data_.throttle_joy = 0.5 - 0.5 * msg->axes[AXIS_THROTTLE];
  }

  /* // Brake */
  if (data_.joy_brake_valid) {
    data_.brake_joy = 0.5 - 0.5 * msg->axes[AXIS_BRAKE];
  }

  // Steering
  data_.steering_joy = 20.0 * M_PI / 180.0 * ((fabs(msg->axes[AXIS_STEER_1]) > fabs(msg->axes[AXIS_STEER_2])) ? msg->axes[AXIS_STEER_1] : msg->axes[AXIS_STEER_2]);
  /* data_.steering_mult = msg->buttons[BTN_STEER_MULT_1] || msg->buttons[BTN_STEER_MULT_2]; */

  // enable and disable buttons
  // TODO add button-based interface to turn SPAS, LKAS, and SCC on/off
  // individually instead of all at once
  if (msg->buttons[BTN_ENABLE]) {
    pub_enable_spas_.publish(1);
    pub_enable_accel_.publish(1);
  }
  if (msg->buttons[BTN_DISABLE]) {
    pub_enable_spas_.publish(0);
    pub_enable_accel_.publish(0);
  }

  data_.stamp = ros::Time::now();
  joy_ = *msg;
}

}

