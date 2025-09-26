#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <iostream>

#include "ros/ros.h"
#include <thread>
#include <initializer_list>
#include <fstream>
#include <array>
#include <serial/serial.h>
#include <map>
#include <vector>
#include <string>
#include <atomic>

#include <sensor_msgs/JointState.h>

//4310
#define P_MIN1 -12.5f
#define P_MAX1 12.5f
#define V_MIN1 -30.0f
#define V_MAX1 30.0f
#define KP_MIN1 0.0f
#define KP_MAX1 500.0f
#define KD_MIN1 0.0f
#define KD_MAX1 5.0f
#define T_MIN1 -10.0f
#define T_MAX1 10.0f

//4340
#define P_MIN2 -12.5f
#define P_MAX2 12.5f
#define V_MIN2 -10.0f
#define V_MAX2 10.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -28.0f
#define T_MAX2 28.0f

//6006
#define P_MIN3 -12.5f
#define P_MAX3 12.5f
#define V_MIN3 -45.0f
#define V_MAX3 45.0f
#define KP_MIN3 0.0f
#define KP_MAX3 500.0f
#define KD_MIN3 0.0f
#define KD_MAX3 5.0f
#define T_MIN3 -12.0f
#define T_MAX3 12.0f

//8006
#define P_MIN4 -12.5f
#define P_MAX4 12.5f
#define V_MIN4 -45.0f
#define V_MAX4 45.0f
#define KP_MIN4 0.0f
#define KP_MAX4 500.0f
#define KD_MIN4 0.0f
#define KD_MAX4 5.0f
#define T_MIN4 -20.0f
#define T_MAX4 20.0f 

//6248p
#define P_MIN5 -12.566f
#define P_MAX5 12.566f
#define V_MIN5 -20.0f
#define V_MAX5 20.0f
#define KP_MIN5 0.0f
#define KP_MAX5 500.0f
#define KD_MIN5 0.0f
#define KD_MAX5 5.0f
#define T_MIN5 -120.0f
#define T_MAX5 120.0f 

//10010l
#define P_MIN6 -12.5f
#define P_MAX6 12.5f
#define V_MIN6 -25.0f
#define V_MAX6 25.0f
#define KP_MIN6 0.0f
#define KP_MAX6 500.0f
#define KD_MIN6 0.0f
#define KD_MAX6 5.0f
#define T_MIN6 -200.0f
#define T_MAX6 200.0f 




#define SEND_DATA_CHECK   1          // 发送数据校验标志位
#define READ_DATA_CHECK   0          // 接收数据校验标志位
#define FRAME_HEADER      0X7B       // 帧头
#define FRAME_TAIL        0X7D       // 帧尾

namespace dmbot_serial
{
  struct MotorLayoutEntry
  {
    std::string limb;
    std::string joint;
    std::string type;
  };

  struct motor_data_t
  {
    std::string name;
    std::string limb;
    std::string joint;
    std::string type;// 电机型号
    int index;

    float pos;
    float vel;
    float tor;
    int p_int;
    int v_int;
    int t_int;

    float pos_set;
    float vel_set;
    float tor_set;
    float kp;
    float kd;
  };

  class robot
  {
   private:
    static constexpr std::size_t kFeedbackWordSize = 5;
    static constexpr std::size_t kSendDataSize = 11;

    enum class CheckMode
    {
      Receive,
      Send
    };

    struct SendData
    {
      std::array<uint8_t, kSendDataSize> tx{};
      unsigned char Frame_Tail{FRAME_TAIL};
    };

    std::string robot_name;

    int motor_seial_baud;
    std::string motor_serial_port;
    serial::Serial serial_motor; // 电机串口
    std::thread rec_thread;

    ros::NodeHandle n;
    ros::Publisher joint_state_pub;
    std::thread pub_thread;
    SendData Send_Data;

    std::vector<uint8_t> receive_buffer_;
    std::vector<motor_data_t> motors;
    std::map<std::string, std::vector<std::size_t>> limb_index_map_;

   public:
    robot();
    ~robot();
    void init_motor_serial();
    void get_motor_data_thread();  // 串口接收线程

    void fresh_cmd_motor_data(double pos, double vel,double torque, double kp,double kd,int motor_idx);
    void send_motor_data();

    void get_motor_data(double &pos,double &vel,double &torque, int motor_idx);
    void publishJointStates();
    std::size_t motorCount() const;

    void dm4310_fbdata(motor_data_t& moto,uint8_t *data);
    void dm4340_fbdata(motor_data_t& moto,uint8_t *data);
    void dm6006_fbdata(motor_data_t& moto,uint8_t *data);
    void dm8006_fbdata(motor_data_t& moto,uint8_t *data);
    void dm6248p_fbdata(motor_data_t& moto,uint8_t *data);
    void dm10010l_fbdata(motor_data_t& moto,uint8_t *data);


    unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);
    int16_t float_to_uint(float x_float, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);

    std::atomic<bool> stop_thread_ {false};

   private:
    void initializeMotorConfiguration(const std::vector<MotorLayoutEntry>& layout);
    std::vector<MotorLayoutEntry> defaultMotorLayout() const;
    std::vector<MotorLayoutEntry> loadMotorLayout();
    uint8_t computeChecksum(std::size_t count_number, CheckMode mode) const;
    void ensureReceiveBuffer();
  };
}
#endif
