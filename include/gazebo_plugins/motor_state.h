/*
 * @file  motor_state.h
 *
 * @brief Struct that holds the state of a dynamixel motor
 *
 * @author  Vincenzo Comito <clynamen@gmail.com>
 */

#ifndef GAZEBO_ROS_DYNAMIXEL_MOTOR_STATE_H_
#define GAZEBO_ROS_DYNAMIXEL_MOTOR_STATE_H_

#include <cstdint>

namespace MotorStateMode {
    enum MotorStateMode {
    Position,
    Velocity
  };
}

struct MotorState {
  MotorStateMode::MotorStateMode mode;

  int32_t motor_id;
  int32_t motor_temp;

  double goal_pos_rad;
  double current_pos_rad;
  double error_rad;
  double velocity_rad_s;
  double velocity_limit_rad_s;
  double load;
  double torque_limit;

  bool is_moving;
  bool torque_enabled;

  // this value will be removed.
  double demultiply_value;
};

#endif
