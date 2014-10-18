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
 * \file  gazebo_ros_dynamixel_motor.h
 *
 * \brief A configurable plugin that controls one or more joint.
 *
 * \author  Vincenzo Comito <clynamen@gmail.com>
 */

#ifndef GAZEBO_ROS_DYNAMIXEL_MOTOR_H_
#define GAZEBO_ROS_DYNAMIXEL_MOTOR_H_

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosDynamixelMotor : public ModelPlugin {

    public:
	  GazeboRosDynamixelMotor();
    ~GazeboRosDynamixelMotor();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnWorldUpdate();

    static const std::string PLUGIN_NAME;

    protected:
      void Shutdown();

    private:
      void publishOdometry(double step_time);
      void getWheelVelocities();

      physics::WorldPtr world;
      physics::ModelPtr parent;
      ros::Publisher force_publisher;

      ros::NodeHandle* rosnode_;

      std::string robot_namespace_;

      bool alive_;
      physics::JointPtr joint;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;
      event::ConnectionPtr update_connection;
  };

}


#endif

