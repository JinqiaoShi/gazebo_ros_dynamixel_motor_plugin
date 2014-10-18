/*
 * Copyright (c) 2014 Team DIANA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_dynamixel_motor.cpp
 *
 * \brief A configurable plugin that controls one or more joint.
 *
 * \author Vincenzo Comito <clynamen@gmail.com>
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_dynamixel_motor.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Wrench.h>

namespace gazebo {

  const std::string GazeboRosDynamixelMotor::PLUGIN_NAME = "GazeboRosDynamixelMotor";

  GazeboRosDynamixelMotor::GazeboRosDynamixelMotor() : alive_(true) {}

  // Destructor
  GazeboRosDynamixelMotor::~GazeboRosDynamixelMotor() {
    ros_info(" destroying  ");
    delete rosnode_;
  }

  // Load the controller
  void GazeboRosDynamixelMotor::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    using namespace std;
    using namespace sdf;

    ros_info("starting");

    this->parent = _parent;
    this->world = _parent->GetWorld();

    ros_info("searching joint");


    string sdf_joint_name = "joint";
    if (!_sdf->HasElement(sdf_joint_name)) {
      ros_fatal("no joint!");
      return;
    } else {
      ros_info("found joint element");
    }
    string joint_name = _sdf->GetElement(sdf_joint_name)->Get<string>();
    ros_info("joint name is " + joint_name);
    joint = this->parent->GetJoint(joint_name);

    this->robot_namespace_ = "";
    ros_info("searching namespace");
    if (!_sdf->HasElement("robotNamespace")) {
      ros_info(PLUGIN_NAME + "Plugin missing <robotNamespace>, defaults to " + this->robot_namespace_);
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ros_info("Starting " + PLUGIN_NAME + " (ns = " + robot_namespace_ + " )" );

    ros_info("creating publisher");
    force_publisher = rosnode_->advertise<geometry_msgs::Wrench>(robot_namespace_ + "/force", 10);

    ros_info("subscribing to world update ");
    // listen to the update event (broadcast every simulation iteration)
    this->update_connection =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosDynamixelMotor::OnWorldUpdate, this));
  }

  // Finalize the controller
  void GazeboRosDynamixelMotor::Shutdown() {
    ros_info("shutting down");
    alive_ = false;
    rosnode_->shutdown();
  }

  void GazeboRosDynamixelMotor::OnWorldUpdate()
  {
    using MsgType = geometry_msgs::Wrench;
    auto wrench_msg = geometry_msgs::Wrench {};
    auto force = joint->GetForceTorque(0);
    wrench_msg.force.x = wrench_msg.force.y = wrench_msg.force.z = 0;
    auto copy3D = [](auto& l, const auto& r) {
      l.x = r.x;
      l.y = r.y;
      l.z = r.z;
    };
    copy3D(wrench_msg.torque, force.body2Torque);

    auto v3_to_string = [](const auto& v) -> std::string {
      return " (" + std::to_string(v.x) + ", "  + std::to_string(v.y) + ", "+ std::to_string(v.z) + ")";
    };

    auto v3_not_zero  = [](const auto& v) -> bool {
      return (v.x != 0 || v.y != 0 || v.z != 0);
    };

    if(v3_not_zero(force.body1Torque)) {
      ros_info("body 1 torque : " + v3_to_string(force.body1Torque));
    }

    if(v3_not_zero(force.body2Torque)) {
      ros_info("body 2 torque : " + v3_to_string(force.body2Torque));
    }

    force_publisher.publish<MsgType>(wrench_msg);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosDynamixelMotor)
}
