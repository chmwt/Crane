#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor_protocol/rm_motor/rm_motor.hpp"
#include "para_init.hpp"
#include "struct.hpp"
#include "tim.h"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

enum class Mode
{
  zero_force_mode,
  rc_ccontrol_mode,
  up_control_mode
};

uint16_t servo_pwm[2] = {2000, 1400};

motor_protocol::RM_Motor motor(&hcan1, 0X200);
extern io::Dbus rc_ctrl;
Pos_Set pos;
Mode mode = Mode::zero_force_mode;

tools::PID chassis_left_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);
tools::PID chassis_right_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);

tools::PID chassis_left_speed_pid(
  tools::PIDMode::DELTA, chassis_speed_pid_config, chassis_speed_maxout, chassis_speed_maxiout,
  chassis_speed_alpha);
tools::PID chassis_right_speed_pid(
  tools::PIDMode::DELTA, chassis_speed_pid_config, chassis_speed_maxout, chassis_speed_maxiout,
  chassis_speed_alpha);

tools::PID lift_motor_pos_pid(
  tools::PIDMode::POSITION, lift_pos_pid_config, lift_pos_maxout, lift_pos_maxiout, lift_pos_alpha);
tools::PID lift_motor_speed_pid(
  tools::PIDMode::DELTA, lift_speed_pid_config, lift_speed_maxout, lift_speed_maxiout,
  lift_speed_alpha);

tools::PID y_axis_pos_pid(
  tools::PIDMode::POSITION, y_axis_pos_pid_config, y_axis_pos_maxout, y_axis_pos_maxiout,
  y_axis_pos_alpha);
tools::PID y_axis_speed_pid(
  tools::PIDMode::DELTA, y_axis_speed_pid_config, y_axis_speed_maxout, y_axis_speed_maxiout,
  y_axis_speed_alpha);

io::Plotter plotter(&huart1);

void move_init(void) {}

void move_mode_set(void)
{
  switch (rc_ctrl.rc.s[MODE_CHANNEL]) {
    case RC_SW_DOWN:
      mode = Mode::zero_force_mode;
      break;
    case RC_SW_MID:
      mode = Mode::rc_ccontrol_mode;
      break;
    case RC_SW_UP:
      mode = Mode::up_control_mode;
      break;
    default:
      break;
  }
  return;
}

void move_set_control(void)
{
  int16_t rc_vel;
  switch (mode) {
    case Mode::zero_force_mode:
      pos.xl = motor.motor_measure_[chassis_left_motor].rev_angle;
      pos.xr = motor.motor_measure_[chassis_right_motor].rev_angle;
      pos.y = motor.motor_measure_[y_axis_motor].rev_angle;
      pos.z = motor.motor_measure_[lift_motor].rev_angle;
      pos.servo = false;
      break;
    case Mode::rc_ccontrol_mode:
      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[X_CHANNEL], RC_DEADLINE);
      pos.xl += rc_vel * 0.0005;
      pos.xr -= rc_vel * 0.0005;
      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[Y_CHANNEL], RC_DEADLINE);
      pos.y += rc_vel * 0.003;
      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[Z_CHANNEL], RC_DEADLINE);
      pos.z += rc_vel * 0.0005;
      pos.servo = (rc_ctrl.rc.s[SERVO_CHANNEL] == RC_SW_UP);
      break;
    case Mode::up_control_mode:

      break;
  }
}

int num = 0;
void move_control_loop(void)
{
  if (num % 4 == 0)
    plotter.plot(motor.motor_measure_[y_axis_motor].speed_rpm, lift_motor_pos_pid.pid_out_);
  num++;
  chassis_left_pos_pid.pid_calc(pos.xl, motor.motor_measure_[chassis_left_motor].rev_angle);
  chassis_left_speed_pid.pid_calc(
    chassis_left_pos_pid.pid_out_, motor.motor_measure_[chassis_left_motor].speed);

  chassis_right_pos_pid.pid_calc(pos.xr, motor.motor_measure_[chassis_right_motor].rev_angle);
  chassis_right_speed_pid.pid_calc(
    chassis_right_pos_pid.pid_out_, motor.motor_measure_[chassis_right_motor].speed);

  lift_motor_pos_pid.pid_calc(pos.z, motor.motor_measure_[lift_motor].rev_angle);
  lift_motor_speed_pid.pid_calc(
    lift_motor_pos_pid.pid_out_, motor.motor_measure_[lift_motor].speed);
  // lift_motor_speed_pid.pid_calc(-200, motor.motor_measure_[lift_motor].speed);

  y_axis_pos_pid.pid_calc(pos.y, motor.motor_measure_[y_axis_motor].rev_angle);
  y_axis_speed_pid.pid_calc(y_axis_pos_pid.pid_out_, motor.motor_measure_[y_axis_motor].speed);
}

void move_cmd_send(void)
{
  if (mode == Mode::zero_force_mode) {
    motor.motor_cmd(0, 0, 0, 0);
    return;
  }
  motor.motor_cmd(
    chassis_left_speed_pid.pid_out_, chassis_right_speed_pid.pid_out_,
    lift_motor_speed_pid.pid_out_, y_axis_pos_pid.pid_out_);

  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, servo_pwm[pos.servo]);
}

extern "C" {
void move_task()
{
  vTaskDelay(2000);
  move_init();
  while (1) {
    move_mode_set();
    move_set_control();
    move_control_loop();
    move_cmd_send();
    vTaskDelay(1);
  }
}
}