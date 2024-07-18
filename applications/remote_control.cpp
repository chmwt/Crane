#include "io/dbus/dbus.hpp"

constexpr static uint16_t SBUS_RX_BUF_NUM = 36;
constexpr static uint16_t RC_FRAME_LENGTH = 18;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

io::Dbus rc_ctrl;

extern "C" {

void rc_init() { rc_ctrl.init(&huart3, &hdma_usart3_rx, SBUS_RX_BUF_NUM); }

void USART3_IRQHandler(void)
{
  if (huart3.Instance->SR & UART_FLAG_RXNE)  //接收到数据
  {
    __HAL_UART_CLEAR_PEFLAG(&huart3);
  }

  else if (USART3->SR & UART_FLAG_IDLE) {
    static uint16_t this_time_rx_len = 0;

    __HAL_UART_CLEAR_PEFLAG(&huart3);

    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
      /* Current memory buffer used is Memory 0 */

      //disable DMA
      //失效DMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);

      //get receive data length, length = set_data_length - remain_length
      //获取接收数据长度,长度 = 设定长度 - 剩余长度
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

      //reset set_data_lenght
      //重新设定数据长度
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

      //set memory buffer 1
      //设定缓冲区1
      hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

      //enable DMA
      //使能DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        rc_ctrl.sbus_to_rc(rc_ctrl.rx1_buf_);
        //记录数据接收时间
        //detect_hook(DBUS_TOE);
      }
    } else {
      /* Current memory buffer used is Memory 1 */
      //disable DMA
      //失效DMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);

      //get receive data length, length = set_data_length - remain_length
      //获取接收数据长度,长度 = 设定长度 - 剩余长度
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

      //reset set_data_lenght
      //重新设定数据长度
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

      //set memory buffer 0
      //设定缓冲区0
      DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

      //enable DMA
      //使能DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        //处理遥控器数据
        rc_ctrl.sbus_to_rc(rc_ctrl.rx2_buf_);
        //记录数据接收时间
        //detect_hook(DBUS_TOE);
      }
    }
  }
}
}