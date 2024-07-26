#ifndef _PARA_INIT_HPP_
#define _PARA_INIT_HPP_

// ǰ��ң����ͨ������
constexpr uint8_t X_CHANNEL = 3;
// ����ң����ͨ������
constexpr uint8_t Y_CHANNEL = 2;
// ����ң����ͨ������
constexpr uint8_t Z_CHANNEL = 1;
// ģʽѡ�����ͨ������
constexpr uint8_t MODE_CHANNEL = 1;
// ���ң����ͨ������
constexpr uint8_t SERVO_CHANNEL = 0;
// ң��������ֵ
constexpr uint8_t RC_SW_UP = 1;
constexpr uint8_t RC_SW_MID = 3;
constexpr uint8_t RC_SW_DOWN = 2;
// ҡ������
constexpr uint8_t RC_DEADLINE = 20;
// ����λ�û�PID���� 3508
static const float chassis_pos_pid_config[3] = {4000.0f, 0.0f, 800000.0f};
static const float chassis_pos_maxout = 500.0f;
static const float chassis_pos_maxiout = 200.0f;
static const float chassis_pos_alpha = 0.1f;
// �����ٶȻ�PID���� 3508
static const float chassis_speed_pid_config[3] = {250.0f, 12.0f, 3000.0f};
static const float chassis_speed_maxout = 6000.0f;
static const float chassis_speed_maxiout = 2000.0f;
static const float chassis_speed_alpha = 0.1f;
// ̧�����λ�û�PID���� 3058
static const float lift_pos_pid_config[3] = {350.0f * 20, 0.0f, 25000.0f * 20};
static const float lift_pos_maxout = 750.0f;
static const float lift_pos_maxiout = 200.0f;
static const float lift_pos_alpha = 0.1f;
// ̧������ٶȻ�PID���� 3058
static const float lift_speed_pid_config[3] = {300.0f, 25.0f, 3000.0f};
static const float lift_speed_maxout = 16000.0f;
static const float lift_speed_maxiout = 8000.0f;
static const float lift_speed_alpha = 0.1f;

// ƽ�Ƶ��λ�û�PID���� 2006
static const float y_axis_pos_pid_config[3] = {800.0f * 36, 0.0f, 100000.0f * 36};
static const float y_axis_pos_maxout = 2500.0f;
static const float y_axis_pos_maxiout = 500.0f;
static const float y_axis_pos_alpha = 0.1f;
// ƽ�Ƶ���ٶȻ�PID���� 2006
static const float y_axis_speed_pid_config[3] = {100.0f, 10.0f, 2000.0f};
static const float y_axis_speed_maxout = 10000.0f;
static const float y_axis_speed_maxiout = 5000.0f;
static const float y_axis_speed_alpha = 0.1f;

typedef enum
{
  chassis_left_id = 0x201,
  chassis_right_id = 0x202,
  lift_id = 0x203,
  y_id = 0x204,
} can_msg_id_e;

#endif  // _PARA_INIT_HPP_