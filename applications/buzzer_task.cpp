#include "io/buzzer/buzzer.hpp"

void user_delay_us(uint16_t us)
{
  for (; us > 0; us--) {
    for (uint8_t i = 50; i > 0; --i) {
      ;
    }
  }
}
void user_delay_ms(uint16_t ms)
{
  for (; ms > 0; --ms) {
    user_delay_us(1000);
  }
}

extern "C" {
void buzzer_task()
{
  io::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);
  buzzer.set(4840, 0.1);
  for (uint8_t i = 0; i < 3; ++i) {
    buzzer.start();
    user_delay_ms(30);
    buzzer.stop();
    user_delay_ms(30);
  }
  return;
}
}