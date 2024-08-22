#include "can.h"
#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
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

uint16_t servo_pwm[2] = {1700, 1100};

motor::M3508 motor_z(3);
motor::M2006 motor_y(4);
motor::M2006 motor_x_left_front(5);
motor::M2006 motor_x_left_back(6);
motor::M2006 motor_x_right_front(7);
motor::M2006 motor_x_right_back(8);

extern io::Dbus rc_ctrl;
Pos pos_set, pos_now, pos_upcom, pos_start;
Mode mode = Mode::zero_force_mode;

io::CAN can1(&hcan1);
io::CAN can2(&hcan2);

tools::PID chassis_left_front_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);
tools::PID chassis_left_front_speed_pid(
  tools::PIDMode::POSITION, chassis_speed_pid_config, chassis_speed_maxout, chassis_speed_maxiout,
  chassis_speed_alpha);

tools::PID chassis_left_back_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);
tools::PID chassis_left_back_speed_pid(
  tools::PIDMode::POSITION, chassis_speed_pid_config, chassis_speed_maxout, chassis_speed_maxiout,
  chassis_speed_alpha);

tools::PID chassis_right_front_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);
tools::PID chassis_right_front_speed_pid(
  tools::PIDMode::POSITION, chassis_speed_pid_config, chassis_speed_maxout, chassis_speed_maxiout,
  chassis_speed_alpha);

tools::PID chassis_right_back_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);
tools::PID chassis_right_back_speed_pid(
  tools::PIDMode::POSITION, chassis_speed_pid_config, chassis_speed_maxout, chassis_speed_maxiout,
  chassis_speed_alpha);

tools::PID lift_motor_pos_pid(
  tools::PIDMode::POSITION, lift_pos_pid_config, lift_pos_maxout, lift_pos_maxiout, lift_pos_alpha);
tools::PID lift_motor_speed_pid(
  tools::PIDMode::POSITION, lift_speed_pid_config, lift_speed_maxout, lift_speed_maxiout,
  lift_speed_alpha);

tools::PID y_axis_pos_pid(
  tools::PIDMode::POSITION, y_axis_pos_pid_config, y_axis_pos_maxout, y_axis_pos_maxiout,
  y_axis_pos_alpha);
tools::PID y_axis_speed_pid(
  tools::PIDMode::POSITION, y_axis_speed_pid_config, y_axis_speed_maxout, y_axis_speed_maxiout,
  y_axis_speed_alpha);

tools::PID y_slow_pos_pid(
  tools::PIDMode::POSITION, y_slow_pos_pid_config, y_slow_pos_maxout, y_slow_pos_maxiout,
  y_slow_pos_alpha);
tools::PID y_slow_speed_pid(
  tools::PIDMode::POSITION, y_slow_speed_pid_config, y_slow_speed_maxout, y_slow_speed_maxiout,
  y_slow_speed_alpha);

io::Plotter plotter(&huart1);

void move_init(void)
{
  // while (!motor_x_left_front.is_open() || !motor_x_left_back.is_open() ||
  //        !motor_x_right_front.is_open() || !motor_x_right_back.is_open() || !motor_y.is_open() ||
  //        !motor_z.is_open()) {
  //   vTaskDelay(1);
  // }
  uint8_t init_time = 0;
  while (1) {
    if (motor_z.speed() * 0.02 < 0.01 && fabs(lift_motor_speed_pid.pid_out_) > 0.2)
      init_time++;
    else
      init_time = 0;

    if (init_time > 100) break;

    lift_motor_speed_pid.pid_calc(0.05, motor_z.speed() * 0.02);

    motor_z.cmd(lift_motor_speed_pid.pid_out_);
    motor_z.write(can2.tx_data_);

    can2.send(motor_z.tx_id());
    vTaskDelay(1);
  }
  pos_start.xl = (motor_x_left_front.angle() + motor_x_left_back.angle()) / 2.0 * 0.032;
  pos_start.xr = (motor_x_right_front.angle() + motor_x_right_back.angle()) / 2.0 * 0.032;

  // pos_start.xl = motor_x_left_front.angle() * 0.032;
  // pos_start.xr = motor_x_right_front.angle() * 0.032;

  pos_start.y = motor_y.angle() * 0.015;
  pos_start.z = motor_z.angle() * 0.02;
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
  // extern io::Buzzer buzzer;
  // buzzer.set(5000);

  // while (!motor_xl.is_alive(osKernelSysTick()) || !motor_xr.is_alive(osKernelSysTick()) |
  //                                                   !motor_y.is_alive(osKernelSysTick()) |
  //                                                   motor_z.is_alive(osKernelSysTick())) {
  // }
  pos_now.xl = (motor_x_left_front.angle() + motor_x_left_back.angle()) / 2.0 * 0.032;
  pos_now.xr = (motor_x_right_front.angle() + motor_x_right_back.angle()) / 2.0 * 0.032;

  // pos_now.xl = motor_x_left_front.angle() * 0.032;
  // pos_now.xr = motor_x_right_front.angle() * 0.032;

  pos_now.y = motor_y.angle() * 0.015;
  pos_now.z = motor_z.angle() * 0.02;

  pos_now.servo = (rc_ctrl.rc.s[SERVO_CHANNEL] == RC_SW_UP);
  pos_now -= pos_start;

  pos_to_uppercom(pos_now);
}
// float set = -2.05;
void move_set_control(void)
{
  int16_t rc_vel;
  switch (mode) {
    case Mode::zero_force_mode:
      pos_set = pos_now;
      break;

    case Mode::rc_ccontrol_mode:
      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[X_CHANNEL], RC_DEADLINE);
      pos_set.xl += rc_vel / 660.0f * 1.0 * 1e-3;  // 1.2m/s * 0.001s
      pos_set.xr -= rc_vel / 660.0f * 1.0 * 1e-3;
      // pos_set.xl = set;
      // pos_set.xr = -set;

      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[Y_CHANNEL], RC_DEADLINE);
      pos_set.y += rc_vel / 660.0f * 0.7 * 1e-3;  // 0.7m/s * 0.001s
      // pos_set.y = set;

      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[Z_CHANNEL], RC_DEADLINE);
      pos_set.z += rc_vel / 660.0f * 0.75 * 1e-3;  // 0.75m/s * 0.001s

      pos_set.servo = (rc_ctrl.rc.s[SERVO_CHANNEL] == RC_SW_UP);
      pos_set.y_mode = false;
      pos_upcom = pos_set;  //
      break;

    case Mode::up_control_mode:
      pos_set = pos_upcom;
      break;
  }
}

int num = 0;
float speed_set = 0.03f, x_set = -0.25;
bool last_servo;
void move_control_loop(void)
{
  if (num % 10 == 0) {
    // plotter.plot(pos_set.y, pos_now.y, y_axis_pos_pid.pid_out_);
    // plotter.plot(pos_set.xl, pos_now.xl);
    // plotter.plot(vy, motor_y.speed() * 0.015, y_axis_speed_pid.pid_out_);
    // plotter.plot(speed_set, motor_z.speed() * 0.02);
    // plotter.plot(speed_set, motor_y.speed() * 0.015);
    // plotter.plot(set, motor_xl.speed() * 0.03, -motor_xr.speed() * 0.03);
    // plotter.plot(
    //   speed_set, motor_x_left_front.speed() * 0.032, motor_x_left_back.speed() * 0.032,
    //   motor_x_right_front.speed() * 0.032, motor_x_right_back.speed() * 0.032);
    // plotter.plot(set, pos_now.xl, -pos_now.xr, chassis_left_front_speed_pid.pid_out_);
    // plotter.plot(pos_upcom.xl, pos_upcom.xr, pos_upcom.y, pos_upcom.z);
    // plotter.plot(motor_z.speed(), motor_z.angle());
    plotter.plot(pos_now.xl, -pos_now.xr);
  }
  num++;

  chassis_left_front_pos_pid.pid_calc(
    (pos_set.xl - pos_set.xr) / 2.f, (pos_now.xl - pos_now.xr) / 2.f);

  chassis_left_front_speed_pid.pid_calc(
    chassis_left_front_pos_pid.pid_out_, motor_x_left_front.speed() * 0.032);
  chassis_left_back_speed_pid.pid_calc(
    chassis_left_front_pos_pid.pid_out_, motor_x_left_back.speed() * 0.032);
  chassis_right_front_speed_pid.pid_calc(
    -chassis_left_front_pos_pid.pid_out_, motor_x_right_front.speed() * 0.032);
  chassis_right_back_speed_pid.pid_calc(
    -chassis_left_front_pos_pid.pid_out_, motor_x_right_back.speed() * 0.032);

  // float chassis_mid_speed = (motor_xl.speed() + motor_xr.speed()) / 2.0f * 0.03;
  // float chassis_mid_speed = 0.0f;
  // chassis_left_front_pos_pid.pid_calc(pos_set.xl, motor_x_left_front.angle() * 0.032);
  // chassis_left_front_speed_pid.pid_calc(
  //   chassis_left_front_pos_pid.pid_out_, motor_x_left_front.speed() * 0.032);
  // chassis_left_front_speed_pid.pid_calc(speed_set, motor_x_left_front.speed() * 0.032);

  // chassis_left_back_speed_pid.pid_calc(
  //   chassis_left_front_pos_pid.pid_out_, motor_x_left_back.speed() * 0.032);

  // chassis_left_back_pos_pid.pid_calc(pos_set.xl, motor_x_left_back.angle() * 0.032);
  // chassis_left_back_speed_pid.pid_calc(
  //   chassis_left_back_pos_pid.pid_out_, motor_x_left_back.speed() * 0.032);
  // chassis_left_back_speed_pid.pid_calc(speed_set, motor_x_left_back.speed() * 0.032);

  // chassis_right_front_pos_pid.pid_calc(pos_set.xr, motor_x_right_front.angle() * 0.032);
  // chassis_right_front_speed_pid.pid_calc(
  //   chassis_right_front_pos_pid.pid_out_, motor_x_right_front.speed() * 0.032);
  // chassis_right_front_speed_pid.pid_calc(-speed_set, motor_x_right_front.speed() * 0.032);

  // chassis_right_back_speed_pid.pid_calc(
  //   chassis_right_front_pos_pid.pid_out_, motor_x_right_back.speed() * 0.032);

  // chassis_right_back_pos_pid.pid_calc(pos_set.xr, motor_x_right_back.angle() * 0.032);
  // chassis_right_back_speed_pid.pid_calc(
  //   chassis_right_back_pos_pid.pid_out_, motor_x_right_back.speed() * 0.032);
  // chassis_right_back_speed_pid.pid_calc(-speed_set, motor_x_right_back.speed() * 0.032);

  if (pos_set.z < -0.3) pos_set.z = -0.3;
  if (pos_set.z > 0.0) pos_set.z = 0.0;

  lift_motor_pos_pid.pid_calc(pos_set.z, pos_now.z);
  lift_motor_speed_pid.pid_calc(lift_motor_pos_pid.pid_out_, motor_z.speed() * 0.02);

  if (pos_set.y > 0.20) pos_set.y = 0.20;
  if (pos_set.y < -0.80) pos_set.y = -0.80;

  if (pos_set.y_mode == false) {
    y_axis_pos_pid.pid_calc(pos_set.y, pos_now.y);
    y_axis_speed_pid.pid_calc(y_axis_pos_pid.pid_out_, motor_y.speed() * 0.015);
    // y_axis_speed_pid.pid_calc(speed_set, motor_y.speed() * 0.015);
  }
  else {
    y_slow_pos_pid.pid_calc(pos_set.y, pos_now.y);
    y_slow_speed_pid.pid_calc(y_slow_pos_pid.pid_out_, motor_y.speed() * 0.015);
    // y_slow_speed_pid.pid_calc(speed_set, motor_y.speed() * 0.015);
  }
}

void move_cmd_send(void)
{
  if (mode == Mode::zero_force_mode) {
    motor_z.cmd(0);
    motor_y.cmd(0);

    motor_x_left_front.cmd(0);
    motor_x_left_back.cmd(0);
    motor_x_right_front.cmd(0);
    motor_x_right_back.cmd(0);
  }
  else {
    motor_x_left_front.cmd(chassis_left_front_speed_pid.pid_out_);
    motor_x_left_back.cmd(chassis_left_back_speed_pid.pid_out_);
    motor_x_right_front.cmd(chassis_right_front_speed_pid.pid_out_);
    motor_x_right_back.cmd(chassis_right_back_speed_pid.pid_out_);

    motor_z.cmd(lift_motor_speed_pid.pid_out_);

    if (pos_set.y_mode == false)
      motor_y.cmd(y_axis_speed_pid.pid_out_);
    else
      motor_y.cmd(y_slow_speed_pid.pid_out_);
  }

  motor_x_left_front.write(can1.tx_data_);
  motor_x_left_back.write(can1.tx_data_);
  motor_x_right_front.write(can1.tx_data_);
  motor_x_right_back.write(can1.tx_data_);

  can1.send(motor_x_left_front.tx_id());

  motor_z.write(can2.tx_data_);
  motor_y.write(can2.tx_data_);

  can2.send(motor_z.tx_id());

  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, servo_pwm[pos_set.servo]);

  if (pos_set.servo != last_servo) {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, servo_pwm[pos_set.servo]);
    last_servo = pos_set.servo;
  }
}

extern "C" void move_task()
{
  vTaskDelay(500);
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
