#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "para_init.hpp"
#include "struct.hpp"

extern motor::M3508 motor_z;
extern motor::M2006 motor_y;

extern motor::M2006 motor_x_left_front;
extern motor::M2006 motor_x_left_back;
extern motor::M2006 motor_x_right_front;
extern motor::M2006 motor_x_right_back;

extern Pos pos_upcom;

void get_upcommand(uint8_t * data)
{
  pos_upcom.xl = ((int16_t)(data[0] << 8 | data[1])) / 1000.f;
  pos_upcom.xr = -pos_upcom.xl;
  pos_upcom.y = ((int16_t)(data[2] << 8 | data[3])) / 1000.f;
  pos_upcom.z = ((int16_t)(data[4] << 8 | data[5])) / 1000.f;
  pos_upcom.servo = data[6];
  pos_upcom.y_mode = data[7];
}

extern io::CAN can1;
extern io::CAN can2;

extern io::Dbus rc_ctrl;

void pos_to_uppercom(Pos pos)
{
  int16_t x = (pos.xl - pos.xr) / 2.0 * 1000.f;
  int16_t y = pos.y * 1000.f;
  int16_t z = pos.z * 1000.f;

  can1.tx_data_[0] = x >> 8;
  can1.tx_data_[1] = x;
  can1.tx_data_[2] = y >> 8;
  can1.tx_data_[3] = y;
  can1.tx_data_[4] = z >> 8;
  can1.tx_data_[5] = z;
  can1.tx_data_[6] = rc_ctrl.rc.s[MODE_CHANNEL];

  can1.send(left_to_upper);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan == &hcan1) {
    can1.recv();
    switch (can1.rx_header_.StdId) {
      case chassis_left_front_id:
        motor_x_left_front.read(can1.rx_data_, osKernelSysTick());
        break;
      case chassis_left_back_id:
        motor_x_left_back.read(can1.rx_data_, osKernelSysTick());
        break;
      case chassis_right_front_id:
        motor_x_right_front.read(can1.rx_data_, osKernelSysTick());
        break;
      case chassis_right_back_id:
        motor_x_right_back.read(can1.rx_data_, osKernelSysTick());
        break;
      case upper_to_left:
        get_upcommand(can1.rx_data_);
        break;

      default:
        break;
    }
  }
  if (hcan == &hcan2) {
    can2.recv();
    switch (can2.rx_header_.StdId) {
      case lift_id:
        motor_z.read(can2.rx_data_, osKernelSysTick());
        break;
      case y_id:
        motor_y.read(can2.rx_data_, osKernelSysTick());
        break;

      default:
        break;
    }
  }
}
