#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor_protocol/rm_motor/rm_motor.hpp"
#include "para_init.hpp"
#include "tools/pid/pid.hpp"

enum class Mode
{
  zero_force_mode,
  rc_ccontrol_mode,
  up_control_mode
};
struct Pos_Set
{
  double xl, xr;
  double y, z;
} pos;

motor_protocol::RM_Motor motor(&hcan1, 0X200);
extern io::Dbus rc_ctrl;
Mode mode = Mode::zero_force_mode;
tools::PID chassis_left_pid(
  tools::PIDMode::POSITION, chassis_pid_config, chassis_maxout, chassis_maxiout, chassis_alpha);
tools::PID chassis_right_pid(
  tools::PIDMode::POSITION, chassis_pid_config, chassis_maxout, chassis_maxiout, chassis_alpha);
tools::PID motor_lift_pid(
  tools::PIDMode::POSITION, lift_pid_config, lift_maxout, lift_maxiout, lift_alpha);
tools::PID motor_y_pid(tools::PIDMode::POSITION, y_pid_config, y_maxout, y_maxiout, y_alpha);

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
  switch (mode) {
    case Mode::zero_force_mode:
      pos.xl = motor.motor_measure_[chassis_left_motor].rev_angle;
      pos.xr = motor.motor_measure_[chassis_right_motor].rev_angle;
      pos.y = motor.motor_measure_[y_motor].rev_angle;
      pos.z = motor.motor_measure_[lift_motor].rev_angle;
      break;
    case Mode::rc_ccontrol_mode:
      pos.xl += rc_ctrl.rc.ch[X_CHANNEL] * 0.0005;
      pos.xr -= rc_ctrl.rc.ch[X_CHANNEL] * 0.0005;
      pos.y += rc_ctrl.rc.ch[Y_CHANNEL] * 0.001;
      pos.z += rc_ctrl.rc.ch[Z_CHANNEL] * 0.0005;
      break;
    case Mode::up_control_mode:

      break;
  }
}

void move_control_loop(void)
{
  chassis_left_pid.pid_calc(pos.xl, motor.motor_measure_[chassis_left_motor].rev_angle);
  chassis_right_pid.pid_calc(pos.xr, motor.motor_measure_[chassis_right_motor].rev_angle);
  motor_lift_pid.pid_calc(pos.z, motor.motor_measure_[lift_motor].rev_angle);
  motor_y_pid.pid_calc(pos.y, motor.motor_measure_[y_motor].rev_angle);
}

void move_cmd_send(void)
{
  if (mode == Mode::zero_force_mode) {
    motor.motor_cmd(0, 0, 0, 0);
    return;
  }
  motor.motor_cmd(
    chassis_left_pid.pid_out_, chassis_right_pid.pid_out_, motor_lift_pid.pid_out_, 0);
  //motor_y_pid.pid_out_);
}

extern "C" {
void move_task()
{
  vTaskDelay(200);
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