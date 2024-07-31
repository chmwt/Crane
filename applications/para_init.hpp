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
// 遥控器拨杆值
constexpr uint8_t RC_SW_UP = 1;
constexpr uint8_t RC_SW_MID = 3;
constexpr uint8_t RC_SW_DOWN = 2;
// 摇杆死区
constexpr uint8_t RC_DEADLINE = 20;

// 底盘位置环PID参数 3508
static const float chassis_pos_pid_config[3] = {4.0f, 0.005f, 100.0f};
static const float chassis_pos_maxout = 0.6f;
static const float chassis_pos_maxiout = 0.05f;
static const float chassis_pos_alpha = 0.1f;
// 底盘速度环PID参数 3508
static const float chassis_speed_pid_config[3] = {20.0f, 0.15f, 200.0f};
static const float chassis_speed_maxout = 0.6f;
static const float chassis_speed_maxiout = 0.5f;
static const float chassis_speed_alpha = 0.1f;

// 抬升电机位置环PID参数 3508
static const float lift_pos_pid_config[3] = {7.5f, 0.1f, 150.0f};
static const float lift_pos_maxout = 0.5f;  // 抬升电机最大高度为0.75m/s
static const float lift_pos_maxiout = 0.05f;
static const float lift_pos_alpha = 0.1f;
// 抬升电机速度环PID参数 3508
static const float lift_speed_pid_config[3] = {35.0f, 0.15f, 500.0f};
static const float lift_speed_maxout = 3.0f;  // 3508最大扭矩为3 N.m
static const float lift_speed_maxiout = 2.0f;
static const float lift_speed_alpha = 0.1f;

// 平移电机位置环PID参数 2006
static const float y_axis_pos_pid_config[3] = {6.0f, 0.0f, 100.0f};
static const float y_axis_pos_maxout = 0.4f;  // 实测y轴最快能到0.8m/s左右
static const float y_axis_pos_maxiout = 0.0f;
static const float y_axis_pos_alpha = 0.1f;
// 平移电机速度环PID参数 2006
static const float y_axis_speed_pid_config[3] = {10.0f, 0.1f, 500.0f};
static const float y_axis_speed_maxout = 1.0f;  // m2006最大扭矩为1.8N.m
static const float y_axis_speed_maxiout = 0.9f;
static const float y_axis_speed_alpha = 0.1f;

static const float y_slow_pos_pid_config[3] = {3.0f, 0.005f, 100.0f};
static const float y_slow_pos_maxout = 0.1f;
static const float y_slow_pos_maxiout = 0.0245f;
static const float y_slow_pos_alpha = 0.1f;

static const float y_slow_speed_pid_config[3] = {6.0f, 0.1f, 2000.0f};
static const float y_slow_speed_maxout = 0.5f;
static const float y_slow_speed_maxiout = 0.25f;
static const float y_slow_speed_alpha = 0.1f;

typedef enum
{
  chassis_left_id = 0x201,
  chassis_right_id = 0x202,
  lift_id = 0x203,
  y_id = 0x204,
} can_msg_id_e;

#endif  // _PARA_INIT_HPP_