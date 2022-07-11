/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#include <vector>
#include <unordered_map>
// #include <chrono>
// #include <thread>
#include "ros/node_handle.h"
#include "ros/param.h"
#include "spnav.h"
#include "geometry_msgs/Vector3.h"
// #include "geometry_msgs/Twist.h"
#include "spacenav_node/spacenav.h"


/** Ensure that the vector parameter has three components.
 *
 * Used for linear_scale and angular_scale. If the parameter has one
 * component, this value is copied as second and third components.
 *
 * @return True if the parameter was set correctly.
 */
bool ensureThreeComponents(std::vector<double> &param) {
  switch(param.size()) {
    case 0:                 // If param is empty, set it to [1, 1, 1].
      param.assign(3, 1);
      return true;
    case 1:                 // If param is [x], set it to [x, x, x].
      param.assign(3, param.back());
    case 3:                 // The param has 3 components, so return true.
      return true;
    default:                // The param doesn't have 3 components, so return false.
      return false;
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "spacenav");

  ros::NodeHandle nh;
  // ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("spacenav/twist", 2);
  ros::Publisher sp_pub = nh.advertise<spacenav_node::spacenav>("spacenav/sp_msg", 2);

  // Hash in which keys are button indices/numbers and values are their functionality.
  std::unordered_map<short, std::string>
  btnMap{{0, "multipleSolution"}, {1, "fit"}, {2, "upperRight"}, {4, "lowerRight"}, {5, "lowerLeft"}, {8, "uppperLeft"}, {10, "iso"},
  {12, "btn01"}, {13, "btn02"}, {14, "btn03"}, {15, "btn04"}, {16, "btn05"}, {17, "btn06"}, {18, "btn07"}, {19, "btn08"}, {20, "btn09"},
  {21, "btn10"}, {22, "esc"}, {23, "alt"}, {24, "shift"}, {25, "ctrl"}, {26, "middle"}, {35, "enter"}, {36, "del"}};

  // Used to scale joystick output to be in [-1, 1]. Estimated from data, and not necessarily correct.
  ros::NodeHandle private_nh("~");
  double full_scale;
  private_nh.param<double>("full_scale", full_scale, 1);
  if(full_scale < 1e-10)
    full_scale = 512;

  // Scale factors for the different axes. End output will be within [-scale, +scale], provided
  // full_scale normalizes to within [-1, 1]. Both take a vector (x, y, z).
  std::vector<double> linear_scale, angular_scale;
  private_nh.param<std::vector<double>>("linear_scale", linear_scale, std::vector<double>(3, 1));
  private_nh.param<std::vector<double>>("angular_scale", angular_scale, std::vector<double>(3, 1));

  // Ensure that linear_scale has three components, if not then terminate the program.
  if(!ensureThreeComponents(linear_scale)) {
    ROS_ERROR_STREAM("Parameter " << private_nh.getNamespace()
    << "/linear_scale must have either one or three components.");
    exit(EXIT_FAILURE);
  }
  // Ensure that angular_scale has three components, if not then terminate the program.
  if(!ensureThreeComponents(angular_scale)) {
    ROS_ERROR_STREAM("Parameter " << private_nh.getNamespace()
    << "/angular_scale must have either one or three components.");
    exit(EXIT_FAILURE);
  }

  ROS_DEBUG("linear_scale: %.3f %.3f %.3f", linear_scale[0], linear_scale[1], linear_scale[2]);
  ROS_DEBUG("angular_scale: %.3f %.3f %.3f", angular_scale[0], angular_scale[1], angular_scale[2]);

  // Terminate the program if space navigator device isn't detected.
  if(spnav_open() == -1) {
    ROS_ERROR("Could not open the space navigator device. "
              "Did you remember to run spacenavd (as root)?");
    return 1;
  }

  // Parameters:
  // The number of polls needed to be done before the device is considered "static".
  int static_count_threshold = 30;
  ros::param::get("~/static_count_threshold", static_count_threshold);

  // If true, sets values to zero when the spacenav is not moving (device is "static").
  bool zero_when_static = true;
  ros::param::get("~/zero_when_static", zero_when_static);

  // If the device is considered "static" and each trans, rot normed component
  // is below the deadband, it will output zeros in either rotation,
  // translation, or both.
  double static_trans_deadband = 0.1, static_rot_deadband = 0.1;
  ros::param::get("~/static_trans_deadband", static_trans_deadband);
  ros::param::get("~/static_rot_deadband", static_rot_deadband);

  // Message definitions:
  // geometry_msgs::Vector3 offset_msg, rot_offset_msg;
  // geometry_msgs::Twist twist_msg;
  spacenav_node::spacenav sp_msg;
  sp_msg.axes.resize(6);
  sp_msg.button.resize(2);

  spnav_event sev;                              // Current space navigation event handler.
  int no_motion_count = 0;                      // Number of iterations the device was static.
  double normed_v[3] = {0}, normed_w[3] = {0};  // Normalized values of the trans (v) and rot (w) components.
  int loop_freq = 10;                           // Loop rate/frequency in Hz.
  ros::param::get("~/loop_freq", loop_freq);    // Get value from launch file using "roslaunch spacenav_node filename.launch" or "rosrun spacenav_node spacenav_node _loop_freq:=value".
  // const int queue_sleep_duration__usec = 1000;  // Duration for how much program sleeps when the queue is empty in microseconds.

  ros::Rate loop_rate(loop_freq);
  while(nh.ok()) {
    bool update_joystick = false;               // If true, the joystick/controller is offset in the current event.
    // bool update_button = false;                 // If true, a button is pressed or released in the current event.
    // bool queue_empty = false;                   // If true, device is static in the current event.

    // Sleep when the queue is empty.
    // If the queue is empty 30 times in a row output zeros.
    // Output changes each time a button event happens, or when a motion
    // event happens and the queue is empty.
    sp_msg.header.stamp = ros::Time().now();

    switch(spnav_poll_event(&sev)) {
      case 0:                         // No SPNAV_EVENT is present (device is static).
        // queue_empty = true;
        if(++no_motion_count > static_count_threshold) {
          if(zero_when_static)        // Set all trans and rot components to 0.
            normed_w[0] = normed_w[1] = normed_w[2] = normed_v[0] = normed_v[1] = normed_v[2] = 0;
          else {
            // If all trans components are within the static trans deadband, set all trans components to 0.
            if(fabs(normed_v[0]) < static_trans_deadband &&
               fabs(normed_v[1]) < static_trans_deadband &&
               fabs(normed_v[2]) < static_trans_deadband)
              normed_v[0] = normed_v[1] = normed_v[2] = 0;
            // If all rot components are within the static rot deadband, set all rot components to 0.
            if(fabs(normed_w[0]) < static_rot_deadband &&
               fabs(normed_w[1]) < static_rot_deadband &&
               fabs(normed_w[2]) < static_rot_deadband)
              normed_w[0] = normed_w[1] = normed_w[2] = 0;
          }
          // no_motion_count = 0;
          update_joystick = true;
        }
        break;

      case SPNAV_EVENT_MOTION:        // Linear and rotation force applied to controller.
        // Update trans components to their normalized values.
        normed_v[0] = sev.motion.z / full_scale;
        normed_v[1] = sev.motion.x / full_scale;
        normed_v[2] = sev.motion.y / full_scale;
        // Update rot components to their normalized values.
        normed_w[0] = sev.motion.rz / full_scale;
        normed_w[1] = sev.motion.rx / full_scale;
        normed_w[2] = sev.motion.ry / full_scale;
        update_joystick = true;
        break;

      case SPNAV_EVENT_BUTTON:        // Button pressed or released.
        // Update button pressed/released information.
        sp_msg.button[0] = btnMap[sev.button.bnum];
        sp_msg.button[1] = sev.button.press ? "pressed" : "released";
        // update_button = true;
        break;

      default:                        // Unexpected/unknown SPNAV_EVENT.
        ROS_WARN("Unknown message type in spacenav. This should never happen.");
    }

    // if(update_joystick && (queue_empty || update_button)) {
    if(update_joystick) {
      // The offset and rot_offset are scaled.
      // offset_msg.x = normed_v[0] * linear_scale[0];
      // offset_msg.y = normed_v[1] * linear_scale[1];
      // offset_msg.z = normed_v[2] * linear_scale[2];
      // rot_offset_msg.x = normed_w[0] * angular_scale[0];
      // rot_offset_msg.y = normed_w[1] * angular_scale[1];
      // rot_offset_msg.z = normed_w[2] * angular_scale[2];
      // twist_msg.linear = offset_msg;
      // twist_msg.angular = rot_offset_msg;
      // spnav_flush();                  // Flushes the buffer so no delay occurs when publishing messages in low frequencies.
      // twist_pub.publish(twist_msg);   // Publish "twist" message.

      // The joystick axes are normalized within [-1, 1].
      for(int i = 0; i < 3; ++i)
        sp_msg.axes[i] = normed_v[i];       // Assign trans components to joystick axes.
      for(int i = 3; i < 6; ++i)
        sp_msg.axes[i] = normed_w[i - 3];   // Assign rot components to joystick axes.

      no_motion_count = 0;
      // update_joystick = false;
      // update_button = true;
    }

    // if(update_button) {
      spnav_flush();                  // Flushes the buffer so no delay occurs when publishing messages in low frequencies.
      sp_pub.publish(sp_msg);         // Publish "sp_msg" message.
    // }

    // if(queue_empty)      // Sleep if the queue is empty (no SPNAV_EVENT is present).
      // std::this_thread::sleep_for(std::chrono::microseconds(queue_sleep_duration__usec));

    loop_rate.sleep();
  }

  spnav_close();
  return 0;
}
