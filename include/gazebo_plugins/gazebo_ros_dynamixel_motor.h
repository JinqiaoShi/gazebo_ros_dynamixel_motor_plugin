/*
 * @file  gazebo_ros_dynamixel_motor.h
 *
 * @brief A configurable plugin that controls one or more joint.
 *
 * @author  Vincenzo Comito <clynamen@gmail.com>
 */

#ifndef GAZEBO_ROS_DYNAMIXEL_MOTOR_H_
#define GAZEBO_ROS_DYNAMIXEL_MOTOR_H_

#include "motor_state.h"

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
#include <dynamixel_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
// #include <dynamixel_controllers/SetTorque.h>
// #include <dynamixel_controllers/SetTorqueLimit.h>
// #include <dynamixel_controllers/TorqueEnable.h>
#include <dynamixel_controllers/SetSpeed.h>
// #include <dynamixel_controllers/SetThreshold.h>
// #include <dynamixel_controllers/SetCompliancePunch.h>
// #include <dynamixel_controllers/SetComplianceSlope.h>
// #include <dynamixel_controllers/SetComplianceMargin.h>

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

    void InitServices();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    MotorState ReadMotor() const;
    void UpdateMotor(const MotorState& read_motor_state);
    void OnWorldUpdate();

    static const std::string PLUGIN_NAME;

    protected:
      void Shutdown();

    private:
      /**
       * Create a dynamixel_msgs JointState given a @b motor_state
       */
      dynamixel_msgs::JointState createJointStateMsg(const std::string& name, const MotorState& motor_state);

      physics::WorldPtr world;
      physics::ModelPtr parent;

      ros::Publisher dynamixel_joint_state_publisher;

      ros::Subscriber command_subscriber;
      ros::Subscriber vel_command;

      ros::ServiceServer set_speed_service;
      ros::ServiceServer enable_torque_service;
      ros::ServiceServer set_compliance_slop_service;
      ros::ServiceServer set_complicance_punch_service;
      ros::ServiceServer set_torque_limit_service;
      ros::ServiceServer set_torque_service;
      ros::ServiceServer set_threshold_service;

      bool SetSpeedService(dynamixel_controllers::SetSpeed::Request& req,
                           dynamixel_controllers::SetSpeed::Response& res);

      ros::NodeHandle* rosnode;

      std::string robot_namespace;
      std::string base_topic_name;

      bool alive_;
      physics::JointPtr joint;

      // Update Rate
      double update_rate_;
      double update_period_;
      double motor_allowed_error;

      common::Time last_update_time_;
      event::ConnectionPtr update_connection;
      MotorState current_motor_state;
      std::string motor_name;
  };

}


#endif

