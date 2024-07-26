#include "can.h"
#include "cmsis_os.h"
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

void pos_to_uppercom(Pos pos)
{
  int16_t x = (pos.xl - pos.xr) / 2 * 1000;
  int16_t y = pos.y * 1000;
  int16_t z = pos.z * 1000;

  uint32_t send_mail_box;

  uppercom_tx_message.StdId = 0x101;
  uppercom_tx_message.IDE = CAN_ID_STD;
  uppercom_tx_message.RTR = CAN_RTR_DATA;
  uppercom_tx_message.DLC = 0x08;

  uppercom_can_send_data[0] = x >> 8;
  uppercom_can_send_data[1] = x;
  uppercom_can_send_data[2] = y >> 8;
  uppercom_can_send_data[3] = y;
  uppercom_can_send_data[4] = z >> 8;
  uppercom_can_send_data[5] = z;

  HAL_CAN_AddTxMessage(&hcan1, &uppercom_tx_message, uppercom_can_send_data, &send_mail_box);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  switch (rx_header.StdId) {
    case chassis_left_id:
      motor_xl.read(rx_data, osKernelSysTick());
      break;
    case chassis_right_id:
      motor_xr.read(rx_data, osKernelSysTick());
      break;
    case lift_id:
      motor_z.read(rx_data, osKernelSysTick());
      break;
    case y_id:
      motor_y.read(rx_data, osKernelSysTick());
      break;
    case 0X100:
      get_upcommand(rx_data);
      break;
    default:
      break;
  }
}
