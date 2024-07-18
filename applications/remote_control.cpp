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
  if (huart3.Instance->SR & UART_FLAG_RXNE)  //���յ�����
  {
    __HAL_UART_CLEAR_PEFLAG(&huart3);
  }

  else if (USART3->SR & UART_FLAG_IDLE) {
    static uint16_t this_time_rx_len = 0;

    __HAL_UART_CLEAR_PEFLAG(&huart3);

    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
      /* Current memory buffer used is Memory 0 */

      //disable DMA
      //ʧЧDMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);

      //get receive data length, length = set_data_length - remain_length
      //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

      //reset set_data_lenght
      //�����趨���ݳ���
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

      //set memory buffer 1
      //�趨������1
      hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

      //enable DMA
      //ʹ��DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        rc_ctrl.sbus_to_rc(rc_ctrl.rx1_buf_);
        //��¼���ݽ���ʱ��
        //detect_hook(DBUS_TOE);
      }
    } else {
      /* Current memory buffer used is Memory 1 */
      //disable DMA
      //ʧЧDMA
      __HAL_DMA_DISABLE(&hdma_usart3_rx);

      //get receive data length, length = set_data_length - remain_length
      //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
      this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

      //reset set_data_lenght
      //�����趨���ݳ���
      hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

      //set memory buffer 0
      //�趨������0
      DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

      //enable DMA
      //ʹ��DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        //����ң��������
        rc_ctrl.sbus_to_rc(rc_ctrl.rx2_buf_);
        //��¼���ݽ���ʱ��
        //detect_hook(DBUS_TOE);
      }
    }
  }
}
}