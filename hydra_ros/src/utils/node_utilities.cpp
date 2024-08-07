/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/utils/node_utilities.h"

#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <hydra/utils/timing_utilities.h>
#include <ros/topic_manager.h>
#include <rosgraph_msgs/Clock.h>

namespace hydra {

using timing::ElapsedTimeRecorder;

bool haveClock() {
  size_t num_pubs = ros::TopicManager::instance()->getNumPublishers("/clock");
  return num_pubs > 0;
}

void clockCallback(const rosgraph_msgs::Clock&) {}

void spinWhileClockPresent() {
  ros::NodeHandle nh;
  bool use_sim_time = false;
  nh.getParam("use_sim_time", use_sim_time);

  ServiceFunctor functor;

  ros::NodeHandle pnh("~");
  ros::ServiceServer service =
      nh.advertiseService("shutdown", &ServiceFunctor::callback, &functor);

  ros::Subscriber clock_sub;
  if (!use_sim_time) {
    // required for topic manager to register publisher
    clock_sub = nh.subscribe("/clock", 10, clockCallback);
  }

  ros::WallRate r(50);
  ROS_INFO("Waiting for bag to start");
  while (ros::ok() && !haveClock()) {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Running...");
  while (ros::ok() && haveClock() && !functor.should_exit) {
    ros::spinOnce();
    r.sleep();
  }

  ros::spinOnce();  // make sure all the callbacks are processed
  ROS_WARN("Exiting!");
}

void spinUntilExitRequested() {
  ServiceFunctor functor;

  ros::NodeHandle nh("~");
  ros::ServiceServer service =
      nh.advertiseService("shutdown", &ServiceFunctor::callback, &functor);

  ros::WallRate r(50);
  ROS_INFO("Running...");
  while (ros::ok() && !functor.should_exit) {
    ros::spinOnce();
    r.sleep();
  }

  ros::spinOnce();  // make sure all the callbacks are processed
  ROS_WARN("Exiting!");
}

void spinAndWait(const ros::NodeHandle& nh) {
  bool exit_after_clock = false;
  nh.getParam("exit_after_clock", exit_after_clock);
  if (exit_after_clock) {
    spinWhileClockPresent();
  } else {
    spinUntilExitRequested();
  }
}

}  // namespace hydra
