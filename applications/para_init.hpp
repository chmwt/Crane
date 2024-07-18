#ifndef _PARA_INIT_HPP_
#define _PARA_INIT_HPP_

// 前后遥控器通道号码
constexpr uint8_t X_CHANNEL = 3;
// 左右遥控器通道号码
constexpr uint8_t Y_CHANNEL = 2;
// 上下遥控器通道号码
constexpr uint8_t Z_CHANNEL = 1;
// 模式选择控器通道号码
constexpr uint8_t MODE_CHANNEL = 1;
// 舵机遥控器通道号码
constexpr uint8_t SERVO_CHANNEL = 0;
// 底盘左右电机编号
constexpr uint8_t chassis_left_motor = 0;
constexpr uint8_t chassis_right_motor = 1;
// 抬升电机编号
constexpr uint8_t lift_motor = 2;
// 平移电机编号
constexpr uint8_t y_motor = 3;
// 遥控器拨杆值
constexpr uint8_t RC_SW_UP = 1;
constexpr uint8_t RC_SW_MID = 3;
constexpr uint8_t RC_SW_DOWN = 2;
// 底盘PID参数 3508
static const float chassis_pid_config[3] = {500.0f, 2.0f, 0.0f};
static const float chassis_maxout = 16000.0f;
static const float chassis_maxiout = 1000.0f;
static const float chassis_alpha = 0.1f;
// 抬升电机PID参数 3058
static const float lift_pid_config[3] = {50.0f, 0.0f, 0.0f};
static const float lift_maxout = 16000.0f;
static const float lift_maxiout = 1000.0f;
static const float lift_alpha = 0.1f;
// 平移电机PID参数 2006
static const float y_pid_config[3] = {500.0f, 2.0f, 10.0f};
static const float y_maxout = 16000.0f;
static const float y_maxiout = 1000.0f;
static const float y_alpha = 0.1f;

typedef enum
{
  chassis_left_id = 0x201,
  chassis_right_id = 0x202,
  lift_id = 0x203,
  y_id = 0x204,
} can_msg_id_e;

#endif  // _PARA_INIT_HPP_