#include "motor_protocol/rm_motor/rm_motor.hpp"
#include "para_init.hpp"

extern motor_protocol::RM_Motor motor;

extern "C" {

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  switch (rx_header.StdId) {
    case chassis_left_id:
      motor.decode_motor_measure(chassis_left_motor, rx_data);
      break;
    case chassis_right_id:
      motor.decode_motor_measure(chassis_right_motor, rx_data);
      break;
    case lift_id:
      motor.decode_motor_measure(lift_motor, rx_data);
      break;
    case y_id:
      motor.decode_motor_measure(y_axis_motor, rx_data);

    default:
      break;
  }
}
}