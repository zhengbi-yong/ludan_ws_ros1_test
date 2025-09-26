#include <dmbot_serial/robot_connect.h>

#include <algorithm>
#include <cctype>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ros/console.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace dmbot_serial
{
namespace
{
const std::vector<std::string> kCanonicalLimbs = {"left_arm", "right_arm", "left_leg", "right_leg", "waist", "neck"};

std::string normalizeToken(const std::string& raw)
{
  std::string result;
  result.reserve(raw.size());
  char last = '\0';
  for (char c : raw)
  {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc) != 0)
    {
      result.push_back(static_cast<char>(std::tolower(uc)));
      last = result.back();
    }
    else if (last != '_')
    {
      result.push_back('_');
      last = '_';
    }
  }
  while (!result.empty() && result.back() == '_')
  {
    result.pop_back();
  }
  if (result.empty())
  {
    return "joint";
  }
  return result;
}

std::string buildMotorName(const std::string& robot, const std::string& limb, const std::string& joint,
                           std::map<std::string, int>& usage)
{
  std::string base = robot + "_" + limb + "_" + joint;
  std::string candidate = base;
  int index = 1;
  while (usage.count(candidate) != 0)
  {
    candidate = base + "_" + std::to_string(++index);
  }
  usage[candidate] = 1;
  return candidate;
}

std::vector<MotorLayoutEntry> parseLayout(const XmlRpc::XmlRpcValue& param)
{
  std::vector<MotorLayoutEntry> layout;
  if (param.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    return layout;
  }
  layout.reserve(param.size());
  for (int i = 0; i < param.size(); ++i)
  {
    if (param[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      continue;
    }
    MotorLayoutEntry entry;
    if (param[i].hasMember("limb") && param[i]["limb"].getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      entry.limb = static_cast<std::string>(param[i]["limb"]);
    }
    if (param[i].hasMember("joint") && param[i]["joint"].getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      entry.joint = static_cast<std::string>(param[i]["joint"]);
    }
    if (param[i].hasMember("type") && param[i]["type"].getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      entry.type = static_cast<std::string>(param[i]["type"]);
    }
    if (!entry.limb.empty())
    {
      layout.push_back(entry);
    }
  }
  return layout;
}
}  // namespace

robot::robot()
{
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("robot_name", robot_name, std::string("ludan"));
  n.param("robot_name", robot_name, robot_name);
  nh_private.param<std::string>("port", motor_serial_port, std::string("/dev/mcu_leftarm"));
  nh_private.param("baud", motor_seial_baud, 921600);

  initializeMotorConfiguration(loadMotorLayout());
  ensureReceiveBuffer();

  init_motor_serial();

  rec_thread = std::thread(&robot::get_motor_data_thread, this);

  joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  ros::Duration(2.0).sleep();

  ROS_INFO_STREAM(robot_name << " robot init complete. configured motors: " << motors.size());
}

robot::~robot()
{
  for (std::size_t i = 0; i < motors.size(); ++i)
  {
    fresh_cmd_motor_data(0.0, 0.0, 0.0, 0.0, 0.0, static_cast<int>(i));
  }

  send_motor_data();

  stop_thread_ = true;

  if (rec_thread.joinable())
  {
    rec_thread.join();
  }
  if (serial_motor.isOpen())
  {
    serial_motor.close();
  }
}

void robot::init_motor_serial()
{
  try
  {
    serial_motor.setPort(motor_serial_port);
    serial_motor.setBaudrate(motor_seial_baud);
    serial_motor.setFlowcontrol(serial::flowcontrol_none);
    serial_motor.setParity(serial::parity_none);
    serial_motor.setStopbits(serial::stopbits_one);
    serial_motor.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_motor.setTimeout(time_out);
    serial_motor.open();
  }
  catch (serial::IOException& /*e*/)
  {
    ROS_ERROR_STREAM("In single initialization,Unable to open motor serial port ");
    throw;
  }
  if (serial_motor.isOpen())
  {
    ROS_INFO_STREAM("In single initialization,Motor Serial Port initialized");
  }
  else
  {
    ROS_ERROR_STREAM("In single initialization,Unable to open motor serial port ");
    throw std::runtime_error("Unable to open motor serial port");
  }
}

void robot::ensureReceiveBuffer()
{
  const std::size_t expected = motors.empty() ? 0 : (1 + motors.size() * kFeedbackWordSize + 1);
  receive_buffer_.assign(expected, 0);
}

void robot::initializeMotorConfiguration(const std::vector<MotorLayoutEntry>& layout)
{
  motors.clear();
  limb_index_map_.clear();
  for (const auto& limb : kCanonicalLimbs)
  {
    limb_index_map_[normalizeToken(limb)] = {};
  }

  motors.reserve(layout.size());
  std::map<std::string, int> name_usage;

  for (std::size_t idx = 0; idx < layout.size(); ++idx)
  {
    motor_data_t motor{};
    motor.index = static_cast<int>(idx);
    motor.limb = normalizeToken(layout[idx].limb);
    const std::string joint_token = layout[idx].joint.empty() ? ("joint_" + std::to_string(idx + 1)) : layout[idx].joint;
    motor.joint = normalizeToken(joint_token);
    motor.type = layout[idx].type.empty() ? "4340" : layout[idx].type;
    motor.name = buildMotorName(robot_name, motor.limb, motor.joint, name_usage);

    motor.pos = 0.0f;
    motor.vel = 0.0f;
    motor.tor = 0.0f;
    motor.tor_set = 0.0f;
    motor.pos_set = 0.0f;
    motor.vel_set = 0.0f;
    motor.kp = 0.0f;
    motor.kd = 0.0f;

    limb_index_map_[motor.limb].push_back(idx);
    motors.push_back(motor);
  }

  ensureReceiveBuffer();
}

std::vector<MotorLayoutEntry> robot::defaultMotorLayout() const
{
  using JointSpec = std::pair<std::string, std::string>;
  const std::vector<std::pair<std::string, std::vector<JointSpec>>> limbs = {
      {"right_arm",
       {{"shoulder_yaw", "10010l"},
        {"shoulder_pitch", "10010l"},
        {"shoulder_roll", "10010l"},
        {"elbow_pitch", "6248p"},
        {"wrist_pitch", "4340"},
        {"wrist_roll", "4340"},
        {"wrist_yaw", "4340"}}},
      {"left_arm",
       {{"shoulder_yaw", "6248p"},
        {"shoulder_pitch", "6248p"},
        {"shoulder_roll", "6248p"},
        {"elbow_pitch", "4340"},
        {"wrist_pitch", "4340"},
        {"wrist_roll", "4340"},
        {"wrist_yaw", "4340"}}},
      {"right_leg",
       {{"hip_yaw", "10010l"},
        {"hip_roll", "10010l"},
        {"hip_pitch", "10010l"},
        {"knee_pitch", "6248p"},
        {"ankle_pitch", "4340"},
        {"ankle_roll", "4340"}}},
      {"left_leg",
       {{"hip_yaw", "10010l"},
        {"hip_roll", "10010l"},
        {"hip_pitch", "10010l"},
        {"knee_pitch", "6248p"},
        {"ankle_pitch", "4340"},
        {"ankle_roll", "4340"}}},
      {"neck",
       {{"yaw", "4340"},
        {"pitch", "4340"},
        {"roll", "4340"}}},
      {"waist",
       {{"yaw", "10010l"}}}};

  std::size_t total_joints = 0;
  for (const auto& limb : limbs)
  {
    total_joints += limb.second.size();
  }

  std::vector<MotorLayoutEntry> layout;
  layout.reserve(total_joints);

  for (const auto& limb : limbs)
  {
    for (const auto& joint : limb.second)
    {
      layout.push_back({limb.first, joint.first, joint.second});
    }
  }

  return layout;
}

std::vector<MotorLayoutEntry> robot::loadMotorLayout()
{
  ros::NodeHandle nh_private("~");
  XmlRpc::XmlRpcValue param;
  std::vector<MotorLayoutEntry> layout;

  if (nh_private.getParam("motor_layout", param))
  {
    layout = parseLayout(param);
  }
  else if (n.getParam(robot_name + "/motor_layout", param))
  {
    layout = parseLayout(param);
  }
  else if (n.getParam("motor_layout", param))
  {
    layout = parseLayout(param);
  }

  if (layout.empty())
  {
    ROS_WARN_STREAM("motor_layout parameter not found or empty. Using default layout for " << robot_name);
    layout = defaultMotorLayout();
  }
  return layout;
}

std::size_t robot::motorCount() const
{
  return motors.size();
}

void robot::get_motor_data_thread()
{
  std::size_t count = 0;
  while (ros::ok() && !stop_thread_)
  {
    if (motors.empty() || receive_buffer_.empty())
    {
      ros::Duration(0.05).sleep();
      continue;
    }

    if (!serial_motor.isOpen())
    {
      ROS_WARN_THROTTLE(1.0, "In get_motor_data_thread, motor serial port unopen");
      ros::Duration(0.05).sleep();
      continue;
    }

    uint8_t byte = 0;
    try
    {
      serial_motor.read(&byte, 1);
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_THROTTLE(1.0, "Failed to read from motor serial port: %s", e.what());
      continue;
    }

    if (count == 0)
    {
      if (byte != FRAME_HEADER)
      {
        continue;
      }
    }

    receive_buffer_[count++] = byte;

    if (count == receive_buffer_.size())
    {
      count = 0;
      const uint8_t checksum = computeChecksum(receive_buffer_.size() - 1, CheckMode::Receive);
      if (checksum != receive_buffer_.back())
      {
        ROS_WARN_THROTTLE(1.0, "Checksum mismatch while parsing motor feedback");
        continue;
      }

      for (std::size_t i = 0; i < motors.size(); ++i)
      {
        auto& motor = motors[i];
        uint8_t* p = receive_buffer_.data() + 1 + i * kFeedbackWordSize;
        if      (motor.type == "4340")   dm4340_fbdata(motor, p);
        else if (motor.type == "4310")   dm4310_fbdata(motor, p);
        else if (motor.type == "6006")   dm6006_fbdata(motor, p);
        else if (motor.type == "8006")   dm8006_fbdata(motor, p);
        else if (motor.type == "6248p")  dm6248p_fbdata(motor, p);
        else if (motor.type == "10010l") dm10010l_fbdata(motor, p);
        else                               dm4340_fbdata(motor, p);
      }
    }
  }
}

void robot::publishJointStates()
{
  ros::Rate rate(1000);
  while (ros::ok())
  {
    sensor_msgs::JointState joint_state_msg;

    joint_state_msg.name.reserve(motors.size());
    joint_state_msg.position.reserve(motors.size());
    joint_state_msg.velocity.reserve(motors.size());
    joint_state_msg.effort.reserve(motors.size());

    for (const auto& motor : motors)
    {
      joint_state_msg.name.push_back(motor.name);
      joint_state_msg.position.push_back(motor.pos);
      joint_state_msg.velocity.push_back(motor.vel);
      joint_state_msg.effort.push_back(motor.tor);
    }

    joint_state_pub.publish(joint_state_msg);

    rate.sleep();
  }
}

void robot::send_motor_data()
{
  if (motors.empty())
  {
    return;
  }

  for (auto& motor : motors)
  {
    uint16_t pos_tmp = 0;
    uint16_t vel_tmp = 0;
    uint16_t kp_tmp = 0;
    uint16_t kd_tmp = 0;
    uint16_t tor_tmp = 0;

    if (motor.type == "8006")
    {
      pos_tmp = float_to_uint(motor.pos_set,  P_MIN4,  P_MAX4,  16);
      vel_tmp = float_to_uint(motor.vel_set,  V_MIN4,  V_MAX4,  12);
      kp_tmp  = float_to_uint(motor.kp,       KP_MIN4, KP_MAX4, 12);
      kd_tmp  = float_to_uint(motor.kd,       KD_MIN4, KD_MAX4, 12);
      tor_tmp = float_to_uint(motor.tor_set,  T_MIN4,  T_MAX4,  12);
    }
    else if (motor.type == "6248p")
    {
      pos_tmp = float_to_uint(motor.pos_set,  P_MIN5,  P_MAX5,  16);
      vel_tmp = float_to_uint(motor.vel_set,  V_MIN5,  V_MAX5,  12);
      kp_tmp  = float_to_uint(motor.kp,       KP_MIN5, KP_MAX5, 12);
      kd_tmp  = float_to_uint(motor.kd,       KD_MIN5, KD_MAX5, 12);
      tor_tmp = float_to_uint(motor.tor_set,  T_MIN5,  T_MAX5,  12);
    }
    else if (motor.type == "6006")
    {
      pos_tmp = float_to_uint(motor.pos_set,  P_MIN3,  P_MAX3,  16);
      vel_tmp = float_to_uint(motor.vel_set,  V_MIN3,  V_MAX3,  12);
      kp_tmp  = float_to_uint(motor.kp,       KP_MIN3, KP_MAX3, 12);
      kd_tmp  = float_to_uint(motor.kd,       KD_MIN3, KD_MAX3, 12);
      tor_tmp = float_to_uint(motor.tor_set,  T_MIN3,  T_MAX3,  12);
    }
    else if (motor.type == "4310")
    {
      pos_tmp = float_to_uint(motor.pos_set,  P_MIN1,  P_MAX1,  16);
      vel_tmp = float_to_uint(motor.vel_set,  V_MIN1,  V_MAX1,  12);
      kp_tmp  = float_to_uint(motor.kp,       KP_MIN1, KP_MAX1, 12);
      kd_tmp  = float_to_uint(motor.kd,       KD_MIN1, KD_MAX1, 12);
      tor_tmp = float_to_uint(motor.tor_set,  T_MIN1,  T_MAX1,  12);
    }
    else if (motor.type == "10010l")
    {
      pos_tmp = float_to_uint(motor.pos_set,  P_MIN6,  P_MAX6,  16);
      vel_tmp = float_to_uint(motor.vel_set,  V_MIN6,  V_MAX6,  12);
      kp_tmp  = float_to_uint(motor.kp,       KP_MIN6, KP_MAX6, 12);
      kd_tmp  = float_to_uint(motor.kd,       KD_MIN6, KD_MAX6, 12);
      tor_tmp = float_to_uint(motor.tor_set,  T_MIN6,  T_MAX6,  12);
    }
    else
    {
      pos_tmp = float_to_uint(motor.pos_set,  P_MIN2,  P_MAX2,  16);
      vel_tmp = float_to_uint(motor.vel_set,  V_MIN2,  V_MAX2,  12);
      kp_tmp  = float_to_uint(motor.kp,       KP_MIN2, KP_MAX2, 12);
      kd_tmp  = float_to_uint(motor.kd,       KD_MIN2, KD_MAX2, 12);
      tor_tmp = float_to_uint(motor.tor_set,  T_MIN2,  T_MAX2,  12);
    }

    Send_Data.tx[0] = FRAME_HEADER;
    Send_Data.tx[1] = static_cast<uint8_t>(motor.index);

    Send_Data.tx[2] = static_cast<uint8_t>(pos_tmp >> 8);
    Send_Data.tx[3] = static_cast<uint8_t>(pos_tmp);
    Send_Data.tx[4] = static_cast<uint8_t>(vel_tmp >> 4);
    Send_Data.tx[5] = static_cast<uint8_t>(((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8));
    Send_Data.tx[6] = static_cast<uint8_t>(kp_tmp);
    Send_Data.tx[7] = static_cast<uint8_t>(kd_tmp >> 4);
    Send_Data.tx[8] = static_cast<uint8_t>(((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8));
    Send_Data.tx[9] = static_cast<uint8_t>(tor_tmp);

    Send_Data.tx[10] = Check_Sum(10, SEND_DATA_CHECK);

    try
    {
      serial_motor.write(Send_Data.tx.data(), Send_Data.tx.size());
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("In send_motor_data,Unable to send data through motor serial port: " << e.what());
    }
  }
}

void robot::fresh_cmd_motor_data(double pos, double vel,double torque, double kp,double kd,int motor_idx)
{
  if (motor_idx < 0 || static_cast<std::size_t>(motor_idx) >= motors.size())
  {
    ROS_WARN_THROTTLE(1.0, "fresh_cmd_motor_data motor index %d out of range", motor_idx);
    return;
  }
  auto& motor = motors[static_cast<std::size_t>(motor_idx)];
  motor.pos_set = static_cast<float>(pos);
  motor.vel_set = static_cast<float>(vel);
  motor.tor_set = static_cast<float>(torque);
  motor.kp = static_cast<float>(kp);
  motor.kd = static_cast<float>(kd);
}

void robot::get_motor_data(double &pos,double &vel,double &torque, int motor_idx)
{
  if (motor_idx < 0 || static_cast<std::size_t>(motor_idx) >= motors.size())
  {
    ROS_WARN_THROTTLE(1.0, "get_motor_data motor index %d out of range", motor_idx);
    pos = 0.0;
    vel = 0.0;
    torque = 0.0;
    return;
  }
  const auto& motor = motors[static_cast<std::size_t>(motor_idx)];
  pos = motor.pos;
  vel = motor.vel;
  torque = motor.tor;
}

unsigned char robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  const CheckMode check_mode = (mode == READ_DATA_CHECK) ? CheckMode::Receive : CheckMode::Send;
  return computeChecksum(static_cast<std::size_t>(Count_Number), check_mode);
}

uint8_t robot::computeChecksum(std::size_t count_number, CheckMode mode) const
{
  uint8_t check_sum = 0;
  if (mode == CheckMode::Receive)
  {
    const std::size_t max_count = std::min(count_number, receive_buffer_.size());
    for (std::size_t k = 0; k < max_count; ++k)
    {
      check_sum = static_cast<uint8_t>(check_sum ^ receive_buffer_[k]);
    }
  }
  else
  {
    const std::size_t max_count = std::min(count_number, Send_Data.tx.size());
    for (std::size_t k = 0; k < max_count; ++k)
    {
      check_sum = static_cast<uint8_t>(check_sum ^ Send_Data.tx[k]);
    }
  }
  return check_sum;
}

void robot::dm4310_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN1, P_MAX1, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN1, V_MAX1, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN1, T_MAX1, 12);  // (-10.0,10.0)
}

void robot::dm4340_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN2, P_MAX2, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN2, V_MAX2, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN2, T_MAX2, 12);  // (-10.0,10.0)
}

void robot::dm6006_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN3, P_MAX3, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN3, V_MAX3, 12); // (-45.0,45.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN3, T_MAX3, 12);  // (-12.0,12.0)
}

void robot::dm8006_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN4, P_MAX4, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN4, V_MAX4, 12); // (-45.0,45.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN4, T_MAX4, 12);  // (-20.0,20.0)
}

void robot::dm6248p_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN5, P_MAX5, 16); // (-12.566,12.566)
  moto.vel = uint_to_float(moto.v_int, V_MIN5, V_MAX5, 12); // (-20.0,20.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN5, T_MAX5, 12);  // (-120.0,120.0)
}

void robot::dm10010l_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.pos = uint_to_float(moto.p_int, P_MIN6, P_MAX6, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN6, V_MAX6, 12); // (-25.0,25.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN6, T_MAX6, 12);  // (-200.0,200.0)
}

int16_t robot::float_to_uint(float x_float, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return static_cast<int16_t>((x_float - offset) * ((static_cast<float>((1 << bits) - 1)) / span));
}

float robot::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + x_min;
}

}  // namespace dmbot_serial
