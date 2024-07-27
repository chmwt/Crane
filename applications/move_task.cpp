#include "can.h"
#include "cmsis_os.h"
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

uint16_t servo_pwm[2] = {1500, 1200};

motor::M3508 motor_xl(1);
motor::M3508 motor_xr(2);
motor::M3508 motor_z(3);
motor::M2006 motor_y(4);

float vxl, vxr, vy, vz;

extern io::Dbus rc_ctrl;
Pos pos_set, pos_now, pos_start, pos_upcom;
Mode mode = Mode::zero_force_mode;

tools::PID chassis_left_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);
tools::PID chassis_left_speed_pid(
  tools::PIDMode::POSITION, chassis_speed_pid_config, chassis_speed_maxout, chassis_speed_maxiout,
  chassis_speed_alpha);

tools::PID chassis_right_pos_pid(
  tools::PIDMode::POSITION, chassis_pos_pid_config, chassis_pos_maxout, chassis_pos_maxiout,
  chassis_pos_alpha);
tools::PID chassis_right_speed_pid(
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

io::Plotter plotter(&huart1);

void move_init(void)
{
  pos_start.xl = motor_xl.angle() * 0.03;
  pos_start.xr = motor_xr.angle() * 0.03;
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
  pos_now.xl = motor_xl.angle() * 0.03;
  pos_now.xr = motor_xr.angle() * 0.03;
  pos_now.y = motor_y.angle() * 0.015;
  pos_now.z = motor_z.angle() * 0.02;
  pos_now.servo = (rc_ctrl.rc.s[SERVO_CHANNEL] == RC_SW_UP);
  pos_to_uppercom(pos_now);
}

void move_set_control(void)
{
  int16_t rc_vel;
  switch (mode) {
    case Mode::zero_force_mode:
      pos_set = pos_now;
      break;

    case Mode::rc_ccontrol_mode:
      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[X_CHANNEL], RC_DEADLINE);
      vxl = rc_vel / 660.0f;
      vxr = -rc_vel / 660.0f;

      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[Y_CHANNEL], RC_DEADLINE);
      vy = rc_vel / 660.0f;

      rc_vel = tools::deadband_limit(rc_ctrl.rc.ch[Z_CHANNEL], RC_DEADLINE);
      vz = rc_vel / 660.0f * 0.5;

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
  if (num % 10 == 0) {
    plotter.plot(vz, motor_z.speed() * 0.02, lift_motor_speed_pid.pid_out_);
  }
  num++;

  chassis_left_pos_pid.pid_calc(pos_set.xl, pos_now.xl);
  chassis_left_speed_pid.pid_calc(vxl, motor_xl.speed() * 0.03);

  chassis_right_pos_pid.pid_calc(pos_set.xr, pos_now.xr);
  chassis_right_speed_pid.pid_calc(vxr, motor_xr.speed() * 0.03);

  lift_motor_pos_pid.pid_calc(pos_set.z, pos_now.z);
  lift_motor_speed_pid.pid_calc(vz, motor_z.speed() * 0.02);

  y_axis_pos_pid.pid_calc(pos_set.y, pos_now.y);
  y_axis_speed_pid.pid_calc(vy, motor_y.speed() * 0.015);
}

// bool last_servo;
void move_cmd_send(void)
{
  if (mode == Mode::zero_force_mode) {
    motor_xl.cmd(0);
    motor_xr.cmd(0);
    motor_z.cmd(0);
    motor_y.cmd(0);
  }

  else {
    motor_xl.cmd(chassis_left_speed_pid.pid_out_);
    motor_xr.cmd(chassis_right_speed_pid.pid_out_);
    motor_z.cmd(lift_motor_speed_pid.pid_out_);
    motor_y.cmd(y_axis_speed_pid.pid_out_);
  }

  CAN_TxHeaderTypeDef motor_tx_message;
  motor_tx_message.StdId = 0x200;
  motor_tx_message.IDE = CAN_ID_STD;
  motor_tx_message.RTR = CAN_RTR_DATA;
  motor_tx_message.DLC = 0x08;

  uint8_t tx_data[8];
  motor_xl.write(tx_data);
  motor_xr.write(tx_data);
  motor_z.write(tx_data);
  motor_y.write(tx_data);

  uint32_t send_mail_box;
  HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, tx_data, &send_mail_box);

  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, servo_pwm[pos_set.servo]);

  // if (pos_set.servo != last_servo) {
  //   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, servo_pwm[pos_set.servo]);
  //   last_servo = pos_set.servo;
  // }
  // else
  //   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

extern "C" void move_task()
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
