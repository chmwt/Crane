#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "para_init.hpp"
#include "struct.hpp"

extern motor::M3508 motor_xl;
extern motor::M3508 motor_xr;
extern motor::M3508 motor_z;
extern motor::M2006 motor_y;

extern Pos pos_upcom;

void get_upcommand(uint8_t * data)
{
  pos_upcom.xl = ((int16_t)(data[0] << 8 | data[1])) / 1000.f;
  pos_upcom.xr = -pos_upcom.xl;
  pos_upcom.y = ((int16_t)(data[2] << 8 | data[3])) / 1000.f;
  pos_upcom.z = ((int16_t)(data[4] << 8 | data[5])) / 1000.f;
  pos_upcom.servo = data[6];
}

CAN_TxHeaderTypeDef uppercom_tx_message;
uint8_t uppercom_can_send_data[8];

extern io::CAN can1;

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
  can1.tx_data_[6] = pos.servo;

  can1.send(0X101);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan == &hcan1) {
    can1.recv();
    switch (can1.rx_header_.StdId) {
      case chassis_left_id:
        motor_xl.read(can1.rx_data_, osKernelSysTick());
        break;
      case chassis_right_id:
        motor_xr.read(can1.rx_data_, osKernelSysTick());
        break;
      case lift_id:
        motor_z.read(can1.rx_data_, osKernelSysTick());
        break;
      case y_id:
        motor_y.read(can1.rx_data_, osKernelSysTick());
        break;
      case 0X100:
        get_upcommand(can1.rx_data_);
        break;
      default:
        break;
    }
  }
}
