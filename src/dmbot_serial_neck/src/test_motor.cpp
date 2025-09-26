#include "ros/ros.h"
#include <dmbot_serial/robot_connect.h>
#include <cmath>
#include <vector>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_motor");
  ros::NodeHandle nh("~");
  ros::Rate r(200);  // 200 Hz 下发

  // 安全参数（默认不动）
  double pos_amp = 0.0;    // 正弦幅值 (rad)
  double freq    = 0.2;    // 频率 (Hz)
  double kp      = 2.0;
  double kd      = 0.1;

  nh.param("pos_amp", pos_amp, pos_amp);
  nh.param("freq",    freq,    freq);
  nh.param("kp",      kp,      kp);
  nh.param("kd",      kd,      kd);

  // 创建上位机串口接口
  dmbot_serial::robot rb;
  const std::size_t motor_count = rb.motorCount();
  if (motor_count == 0)
  {
    ROS_ERROR("未配置任何电机，退出 test_motor");
    return 1;
  }

  std::vector<double> pos_cmd(motor_count, 0.0), vel_cmd(motor_count, 0.0), tor_cmd(motor_count, 0.0);

  const double two_pi = 2.0 * M_PI;
  const double dt = 1.0 / 200.0;
  double t = 0.0;

  ROS_INFO("test_motor running with N=%zu, pos_amp=%.3f rad, freq=%.3f Hz, kp=%.3f, kd=%.3f",
           motor_count, pos_amp, freq, kp, kd);
  ROS_INFO("默认不动，若要小幅测试请传参：_pos_amp:=0.05 _kp:=2.0 _kd:=0.1");

  while (ros::ok())
  {
    // 生成多路命令（简单正弦，带相位错开，便于识别）
    for (std::size_t i = 0; i < motor_count; ++i)
    {
      double phase = (two_pi * static_cast<double>(i)) / static_cast<double>(motor_count);
      pos_cmd[i] = pos_amp * std::sin(two_pi * freq * t + phase);
      vel_cmd[i] = 0.0;   // 如固件需要也可给导数
      tor_cmd[i] = 0.0;
      rb.fresh_cmd_motor_data(pos_cmd[i], vel_cmd[i], tor_cmd[i], kp, kd, static_cast<int>(i));
    }

    // 下发给 STM32
    rb.send_motor_data();

    // 可选：打印少量反馈
    if (static_cast<int>(t*10) % 10 == 0) { // 约每 0.1s 打印一次
      double p0 = 0.0, v0 = 0.0, t0 = 0.0;
      double p7 = 0.0, v7 = 0.0, t7 = 0.0;
      double p13 = 0.0, v13 = 0.0, t13 = 0.0;
      rb.get_motor_data(p0,v0,t0,0);
      if (motor_count > 7)
      {
        rb.get_motor_data(p7,v7,t7,7);
      }
      if (motor_count > 13)
      {
        rb.get_motor_data(p13,v13,t13,13);
      }
      ROS_INFO_THROTTLE(0.5, "m0 pos=%.3f, m7 pos=%.3f, m13 pos=%.3f", p0, p7, p13);
    }

    t += dt;
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
