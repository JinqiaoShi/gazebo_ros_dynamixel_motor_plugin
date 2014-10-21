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
#include <functional>
#include <cmath>

#include <gazebo_plugins/gazebo_ros_dynamixel_motor.h>
#include <gazebo_plugins/motor_state.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/math/math.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/SetTorque.h>
#include <dynamixel_controllers/SetTorqueLimit.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <dynamixel_controllers/SetSpeed.h>
#include <dynamixel_controllers/SetThreshold.h>
#include <dynamixel_controllers/SetCompliancePunch.h>
#include <dynamixel_controllers/SetComplianceSlope.h>
#include <dynamixel_controllers/SetComplianceMargin.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <std_msgs/Float64.h>

using namespace Td;
using namespace std;

using MsgType = dynamixel_msgs::JointState;

namespace gazebo {

  const std::string GazeboRosDynamixelMotor::PLUGIN_NAME = "GazeboRosDynamixelMotor";

  GazeboRosDynamixelMotor::GazeboRosDynamixelMotor() : alive_(true) {}

  // Destructor
  GazeboRosDynamixelMotor::~GazeboRosDynamixelMotor() {
    ros_info(" destroying  ");
    delete rosnode;
  }

  // Load the controller
  void GazeboRosDynamixelMotor::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    using namespace std;
    using namespace sdf;

    ros_info("starting");

    this->parent = parent;
    this->world = parent->GetWorld();

    this->robot_namespace = GetValueFromElement<string>(sdf, "robotNamespace", "");

    joint = GetReferencedJoint(parent, sdf, "joint");

    if(joint == nullptr) {
      ros_fatal("No joint was found");
      return;
    }


    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode = new ros::NodeHandle(this->robot_namespace);

    ros_info("Starting " + PLUGIN_NAME + " (ns = " + robot_namespace + " )" );

    current_motor_state.mode = MotorStateMode::Position;
    current_motor_state.demultiply_value = GetValueFromElement<double>(sdf, "demultiply_value", 0.0);
    current_motor_state.current_pos_rad = current_motor_state.goal_pos_rad = GetValueFromElement<double>(sdf, "default_pos", 0.0);
    current_motor_state.velocity_limit_rad_s = GetValueFromElement<double>(sdf, "default_vel_limit", 1);
    current_motor_state.torque_enabled = true;
    motor_allowed_error = GetValueFromElement<double>(sdf, "allowed_error", 0.001);
    current_motor_state.torque_limit = GetValueFromElement<double>(sdf, "default_torque_limit", 10);

    base_topic_name = GetValueFromElement<string>(sdf, "base_topic_name", "dynamixel_motor");

    joint->SetPosition(0, current_motor_state.current_pos_rad);

    ros_info("creating subscribers");

    auto mkTopicName = [&](string s) {
      return toString(robot_namespace, "/", base_topic_name, s);
    };

    command_subscriber = rosnode->subscribe<std_msgs::Float64>(
      mkTopicName("/command"), 10, [&] (const std_msgs::Float64::ConstPtr& msg) {
        ros_info("setting new position");
        current_motor_state.mode = MotorStateMode::Position;
        current_motor_state.goal_pos_rad = msg->data;
      }
    );

    arm_command_subscriber = rosnode->subscribe<std_msgs::Float64>(
      mkTopicName("/arm/command"), 10, [&] (const std_msgs::Float64::ConstPtr& msg) {
        ros_info("setting new arm position");
        current_motor_state.mode = MotorStateMode::Position;
        current_motor_state.goal_pos_rad = msg->data * current_motor_state.demultiply_value;
      }
    );

    vel_command = rosnode->subscribe<std_msgs::Float64>(
      mkTopicName("/vel_tor/command"), 10, [&] (const std_msgs::Float64::ConstPtr& msg) {
        ros_info("setting velocity");
        current_motor_state.mode = MotorStateMode::Velocity;
        current_motor_state.velocity_rad_s = msg->data;
      }
    );


    ros_info("creating publishers");
    dynamixel_joint_state_publisher = rosnode->advertise<MsgType>(
      mkTopicName("/state"), 10);
    dynamixel_arm_joint_state_publisher = rosnode->advertise<MsgType>(
      mkTopicName("arm/state"), 10);

    ros_info("creating services");
    InitServices();

    string motor_name = GetValueFromElement<string>(sdf, "motor_name", joint->GetName());

    ros_info("subscribing to world update ");
    // listen to the update event (broadcast every simulation iteration)
    update_connection =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosDynamixelMotor::OnWorldUpdate, this));
  }


void GazeboRosDynamixelMotor::InitServices()
{
  auto mkServiceName = [&](string s) {
    return toString(robot_namespace, "/", base_topic_name, s);
  };

  set_speed_service = rosnode->advertiseService(mkServiceName("/set_speed"), &GazeboRosDynamixelMotor::SetSpeedService, this);

//   std::function< bool(dynamixel_controllers::SetSpeed::Request&, dynamixel_controllers::SetSpeed::Response& res) > a;
//   a = [&] (
//     dynamixel_controllers::SetSpeed::Request& req,
//     dynamixel_controllers::SetSpeed::Response& res
//   ) -> bool {
//     current_motor_state.velocity_rad_s = req.speed;
//     return true;
//   };
//
//   set_speed_service = rosnode->advertiseService(mkServiceName("/set_speed"), a);
//

  enable_torque_service = rosnode->advertiseService(mkServiceName("/torque_enable"),
        (boost::function<bool(dynamixel_controllers::TorqueEnable::Request&,
            dynamixel_controllers::TorqueEnable::Response&)>) ([&] (
    dynamixel_controllers::TorqueEnable::Request& req,
    dynamixel_controllers::TorqueEnable::Response& res
  ) {
    current_motor_state.torque_enabled = req.torque_enable;
    return true;
  }) );

  boost::function<bool(dynamixel_controllers::SetTorqueLimit::Request&,
      dynamixel_controllers::SetTorqueLimit::Response&)> torque_limit_f =  [&] (
    dynamixel_controllers::SetTorqueLimit::Request& req,
    dynamixel_controllers::SetTorqueLimit::Response& res
  ) {
    current_motor_state.torque_limit = req.torque_limit;
    return true;
  };

  set_torque_limit_service = rosnode->advertiseService(mkServiceName("/set_torque_limit"), torque_limit_f);
//
//   set_torque_limit_service = rosnode->advertiseService(mkServiceName("/set_torque"), [&] (
//     dynamixel_controllers::SetTorque::Request& req,
//     dynamixel_controllers::SetTorque::Response& res
//   ) {
//     ros_error("/set_torque not yet implemented");
//     return false;
//   });

  // TODO: add these
// self.compliance_slope_service = rospy.Service(self.controller_namespace + '/set_compliance_slope', SetComplianceSlope, self.process_set_compliance_slope)
// self.compliance_marigin_service = rospy.Service(self.controller_namespace + '/set_compliance_margin', SetComplianceMargin, self.process_set_compliance_margin)
// self.compliance_punch_service = rospy.Service(self.controller_namespace + '/set_compliance_punch', SetCompliancePunch, self.process_set_compliance_punch)
// self.torque_service = rospy.Service(self.controller_namespace + '/set_torque', SetTorque, self.process_set_torque)

}
bool GazeboRosDynamixelMotor::SetSpeedService(dynamixel_controllers::SetSpeed::Request& req, dynamixel_controllers::SetSpeed::Response& res)
{
  current_motor_state.velocity_rad_s = req.speed;
  return true;
}

  // Finalize the controller
  void GazeboRosDynamixelMotor::Shutdown() {
    ros_info("shutting down");
    alive_ = false;
    rosnode->shutdown();
  }

  dynamixel_msgs::JointState GazeboRosDynamixelMotor::createJointStateMsg(const std::string& name, const MotorState& motor_state)
  {
    dynamixel_msgs::JointState msg;
    msg.name = name;
    msg.motor_ids = std::vector<int>{ motor_state.motor_id };
    msg.motor_temps = std::vector<int>{ motor_state.motor_temp };
    msg.current_pos = motor_state.current_pos_rad;
    msg.goal_pos = motor_state.goal_pos_rad;
    msg.is_moving = motor_state.is_moving;
    msg.error = motor_state.error_rad;
    msg.velocity = motor_state.velocity_rad_s;
    msg.load = motor_state.load;

    return msg;
  }


  dynamixel_msgs::JointState GazeboRosDynamixelMotor::createArmJointStateMsg(const std::string& name, const MotorState& motor_state)
  {
   // TODO: fix data
    return createJointStateMsg(name, motor_state);
  }


  void GazeboRosDynamixelMotor::UpdateMotor()
  {
    current_motor_state.current_pos_rad = joint->GetAngle(0).Radian();

    current_motor_state.error_rad = 0;
    if(current_motor_state.mode == MotorStateMode::Position) {
      double pos_delta_rad = (current_motor_state.goal_pos_rad - current_motor_state.current_pos_rad);
      bool goal_reached =  fabs(pos_delta_rad) < motor_allowed_error;
      current_motor_state.error_rad = pos_delta_rad;

      if(!goal_reached) {
        current_motor_state.velocity_rad_s = Td::sgn(pos_delta_rad) * current_motor_state.velocity_limit_rad_s;
      } else {
        current_motor_state.velocity_rad_s = 0;
      }

    } else if (current_motor_state.mode == MotorStateMode::Velocity) {
      if(!current_motor_state.torque_enabled) {
        current_motor_state.velocity_rad_s = 0;
      }
    }

    if(current_motor_state.torque_enabled) {
      joint->SetMaxForce(0, current_motor_state.torque_limit);
    } else {
      joint->SetMaxForce(0, 0);
    }

    current_motor_state.is_moving = current_motor_state.velocity_rad_s != 0 && current_motor_state.torque_enabled;
    current_motor_state.load = joint->GetForceTorque(0).body2Torque.x;
    joint->SetVelocity(0, current_motor_state.velocity_rad_s);
  }

  void GazeboRosDynamixelMotor::OnWorldUpdate()
  {
    UpdateMotor();

    MsgType joint_state_msg = createJointStateMsg(motor_name, current_motor_state);
    dynamixel_joint_state_publisher.publish<MsgType>(joint_state_msg);

    MsgType arm_joint_state_msg = createArmJointStateMsg(motor_name, current_motor_state);
    dynamixel_arm_joint_state_publisher.publish<MsgType>(arm_joint_state_msg);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosDynamixelMotor)
}
