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

uint16_t servo_pwm[2] = {1500, 1200};

motor_protocol::RM_Motor motors(&hcan1, 0X200);

extern io::Dbus rc_ctrl;
Pos pos_set, pos_now, pos_start, pos_upcom;
Mode mode = Mode::zero_force_mode;

tools::PID chassis_left_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);
tools::PID chassis_left_speed_pid(
  tools::PIDMode::DELTA, chassis_speed_pid_config, chassis_speed_maxout, chassis_speed_maxiout,
  chassis_speed_alpha);

tools::PID chassis_right_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);
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

void move_init(void)
{
  pos_start.xl = motors.motor_measure_[chassis_left_motor].rev_angle * 0.03 * 187 / 3591;
  pos_start.xr = motors.motor_measure_[chassis_right_motor].rev_angle * 0.03 * 187 / 3591;
  pos_start.y = motors.motor_measure_[y_axis_motor].rev_angle * 0.015 / 36;
  pos_start.z = motors.motor_measure_[lift_motor].rev_angle * 0.02 * 187 / 3591;
}

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

extern void pos_to_uppercom(Pos pos);

void move_motor_update(void)
{
  pos_now.xl = motors.motor_measure_[chassis_left_motor].rev_angle * 0.03 * 187 / 3591;
  pos_now.xr = motors.motor_measure_[chassis_right_motor].rev_angle * 0.03 * 187 / 3591;
  pos_now.y = motors.motor_measure_[y_axis_motor].rev_angle * 0.015 / 36;
  pos_now.z = motors.motor_measure_[lift_motor].rev_angle * 0.02 * 187 / 3591;
  pos_now.servo = (rc_ctrl.rc.s[SERVO_CHANNEL] == RC_SW_UP);
  pos_to_uppercom(pos_now);
}
Pos pos_to = {1.5, -1.5, 0, 0, 0};
void move_set_control(void)
{
  int16_t rc_vel;
  switch (mode) {
    case Mode::zero_force_mode:
      pos_set = pos_now;
      break;
    case Mode::rc_ccontrol_mode:
      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[X_CHANNEL], RC_DEADLINE);
      pos_set.xl += rc_vel * 0.001 * 0.02 * 187 / 3591;
      pos_set.xr -= rc_vel * 0.001 * 0.02 * 187 / 3591;
      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[Y_CHANNEL], RC_DEADLINE);
      pos_set.y += rc_vel * 0.003 * 0.015 / 36;
      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[Z_CHANNEL], RC_DEADLINE);
      pos_set.z += rc_vel * 0.0005 * 0.02 * 187 / 3591;
      pos_set.servo = (rc_ctrl.rc.s[SERVO_CHANNEL] == RC_SW_UP);
      break;
    case Mode::up_control_mode:
      pos_set = pos_upcom;
      pos_set.xl += pos_start.xl;
      pos_set.xr += pos_start.xr;
      pos_set.y += pos_start.y;
      pos_set.z += pos_start.z;
      break;
  }
}

int num = 0;
void move_control_loop(void)
{
  if (num % 4 == 0)
    plotter.plot(
      motors.motor_measure_[chassis_left_motor].speed,
      motors.motor_measure_[chassis_right_motor].speed);
  num++;

  chassis_left_pos_pid.pid_calc(pos_set.xl, pos_now.xl);
  chassis_left_speed_pid.pid_calc(
    chassis_left_pos_pid.pid_out_, motors.motor_measure_[chassis_left_motor].speed);
  // chassis_left_speed_pid.pid_calc(-200, motors.motor_measure_[chassis_left_motor].speed);

  chassis_right_pos_pid.pid_calc(pos_set.xr, pos_now.xr);
  chassis_right_speed_pid.pid_calc(
    chassis_right_pos_pid.pid_out_, motors.motor_measure_[chassis_right_motor].speed);
  // chassis_right_speed_pid.pid_calc(200, motors.motor_measure_[chassis_right_motor].speed);

  lift_motor_pos_pid.pid_calc(pos_set.z, pos_now.z);
  lift_motor_speed_pid.pid_calc(
    lift_motor_pos_pid.pid_out_, motors.motor_measure_[lift_motor].speed);
  // lift_motor_speed_pid.pid_calc(-200, motors.motor_measure_[lift_motor].speed);

  y_axis_pos_pid.pid_calc(pos_set.y, pos_now.y);
  y_axis_speed_pid.pid_calc(y_axis_pos_pid.pid_out_, motors.motor_measure_[y_axis_motor].speed);
}

// bool last_servo;
void move_cmd_send(void)
{
  if (mode == Mode::zero_force_mode) {
    motors.motor_cmd(0, 0, 0, 0);
    return;
  }
  motors.motor_cmd(
    chassis_left_speed_pid.pid_out_, chassis_right_speed_pid.pid_out_,
    lift_motor_speed_pid.pid_out_, y_axis_pos_pid.pid_out_);

  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, servo_pwm[pos_set.servo]);

  // if (pos_set.servo != last_servo) {
  //   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, servo_pwm[pos_set.servo]);
  //   last_servo = pos_set.servo;
  // }
  // else
  //   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

extern "C" {
void move_task()
{
  vTaskDelay(2000);
  move_init();
  while (1) {
    move_mode_set();
    move_motor_update();
    move_set_control();
    move_control_loop();
    move_cmd_send();
    vTaskDelay(1);
  }
}
}