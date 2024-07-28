#include "cmsis_os.h"
#include "io/led/led.hpp"

constexpr float RGB_FLOW_COLOR_CHANGE_TIME = 1000;
constexpr uint8_t RGB_FLOW_COLOR_LENGTH = 6;

struct aRGB
{
  float a, r, g, b;
  // 重载 + 运算符
  aRGB operator+(const aRGB & other) const
  {
    return {a + other.a, r + other.r, g + other.g, b + other.b};
  }
  // 重载 - 运算符
  aRGB operator-(const aRGB & other) const
  {
    return {a - other.a, r - other.r, g - other.g, b - other.b};
  }
  // 重载 / 运算符
  aRGB operator/(float value) const { return {a / value, r / value, g / value, b / value}; }
  // 重载 * 运算符
  aRGB operator*(float value) const { return {a * value, r * value, g * value, b * value}; }

} RGB_flow_color[RGB_FLOW_COLOR_LENGTH + 1] = {
  {255, 0, 0, 255},  // Blue
  {0, 0, 255, 0},    // Green (dark)
  {255, 255, 0, 0},  // Red
  {0, 0, 0, 255},    // Blue (dark)
  {255, 0, 255, 0},  // Green
  {0, 255, 0, 0},    // Red (dark)
  {255, 0, 0, 255}   // Blue
};

extern "C" void led_task()
{
  io::Led led(&htim5);
  while (1) {
    for (uint16_t i = 0; i < RGB_FLOW_COLOR_LENGTH; i++) {
      aRGB start_color = RGB_flow_color[i];
      aRGB end_color = RGB_flow_color[i + 1];
      aRGB delta_color = (end_color - start_color) / RGB_FLOW_COLOR_CHANGE_TIME;
      for (uint16_t j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++) {
        aRGB current_color = start_color + delta_color * j;
        current_color = current_color * current_color.a;
        led.set(current_color.r, current_color.g, current_color.b);
        vTaskDelay(1);
      }
    }
  }
}