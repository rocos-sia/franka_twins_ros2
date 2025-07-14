/**
  ******************************************************************************
  * @file           : dh_gripper.cpp
  * @author         : sun
  * @brief          : None
  * @attention      : None
  * @date           : 11/25/24
  ******************************************************************************
  */

#include "dh_gripper.h"

std::mutex gripper_position_mutex; // 用于保护位置数据的互斥锁
double current_position = 0.0; // 用于存储当前夹爪位置


namespace Tosor {
  bool dh_gripper::Open(const std::string& port, uint32_t baudrate)
  {
    port_ = port;
    baudrate_ = baudrate;

    serial_ = std::make_shared<serial::Serial>(port,
                                               baudrate,
                                               serial::Timeout::simpleTimeout(timeout_ms_),
                                               data_bit_,
                                               parity_,
                                               stop_bit_,
                                               flow_control_);

    if (!serial_->isOpen()) {
      RCLCPP_ERROR(this->get_logger(), "gripper >%s< failed to open.", port_.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "gripper >%s< opened successfully.", port_.c_str());
    return true;
  }

  uint16_t dh_gripper::calculateCRC(const std::vector<uint8_t>& data)
  {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < data.size(); i++) {
      crc ^= data[i];
      for (size_t j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc;
  }

  void dh_gripper::printHex(const std::vector<uint8_t>& data)
  {
    for (auto byte : data) {
      std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
    }
    std::cout << std::dec << std::endl;
  }

  std::vector<uint8_t> dh_gripper::createModbusMessage(uint8_t address, uint8_t functionCode, uint16_t registerAddress,
    uint16_t value)
  {
    std::vector<uint8_t> message = {
      address,
      functionCode,
      static_cast<uint8_t>(registerAddress >> 8),
      static_cast<uint8_t>(registerAddress & 0xFF),
      static_cast<uint8_t>(value >> 8),
      static_cast<uint8_t>(value & 0xFF)
  };
    uint16_t crc = calculateCRC(message);
    message.push_back(static_cast<uint8_t>(crc & 0xFF));      // CRC 低字节
    message.push_back(static_cast<uint8_t>(crc >> 8));        // CRC 高字节
    return message;
  }

  int  dh_gripper::initializeGripper(uint8_t address)
  {
    std::vector<uint8_t> command;
    command=createModbusMessage(address, WRITE_REGISTER, INIT_REGISTER, 0x00A5);
    serial_->write(command);
    readResponse(8);
    usleep(5000000);
    is_init_=true;
    return 0;
  }

  int dh_gripper::setGripperForce(uint8_t address, int forcePercent)
  {
    // std::vector<uint8_t> command;
    if (forcePercent < 0 || forcePercent > 1000) {
      RCLCPP_ERROR(this->get_logger(),"Error: input value out of range. Must be between 0 and 1000.");
      return -1;
    }
      // command=createModbusMessage(address, WRITE_REGISTER, FORCE_REGISTER, static_cast<uint16_t>forcePercent);
      // serial_->write(command);
    serial_->write(createModbusMessage(address, WRITE_REGISTER, FORCE_REGISTER, static_cast<uint16_t>(forcePercent)));
    readResponse(8);
    return 0;


  }

  int dh_gripper::setGripperPosition(uint8_t address, int position)
  {
    if (position < 0 || position > 1000) {
      RCLCPP_ERROR(this->get_logger(),"Error: input value out of range. Must be between 0 and 1000.");
      return -1;
    }
    serial_->write(createModbusMessage(address, WRITE_REGISTER, POSITION_REGISTER, static_cast<uint16_t>(position)));
    readResponse(8);
    return 0;
  }

  std::vector<uint8_t> dh_gripper::readRegister(uint8_t address, uint16_t registerAddress, uint16_t numRegisters)
  {
    std::vector<uint8_t> message = {
      address,
      0x03,
      static_cast<uint8_t>(registerAddress >> 8),
      static_cast<uint8_t>(registerAddress & 0xFF),
      static_cast<uint8_t>(numRegisters >> 8),
      static_cast<uint8_t>(numRegisters & 0xFF)
  };
    uint16_t crc = calculateCRC(message);
    message.push_back(static_cast<uint8_t>(crc & 0xFF));
    message.push_back(static_cast<uint8_t>(crc >> 8));
    return message;
  }


  double dh_gripper::readGripperPosition(uint8_t address)
  {
    auto readPositionMessage = readRegister(address, FB_POSITION_REGISTER);
    // printHex(readPositionMessage);
    serial_->write(readPositionMessage);
    usleep(1000);
    auto response=readResponse();
    // printHex(response);
    return parseData(response);

  }

  int dh_gripper::readGripperStatus(uint8_t address)
  {
    auto readStatusMessage = readRegister(address, FB_GRIPPER_STATUS_REGISTER);
    serial_->write(readStatusMessage);
    usleep(1000);
    auto response=readResponse();
    double  status=parseData(response);
    // 检查状态是否为有效值（可选，根据您的业务逻辑决定）
    if (std::isnan(status)) {
      RCLCPP_ERROR(this->get_logger(), "Error: Parsed data is NaN.");
      return -1; // 使用特定的错误码表示失败
    }
    return static_cast<int>(status) ;

  }

  double dh_gripper::parseData(const std::vector<uint8_t>& response)
  {

    if (response.size() < 7) {
      printHex(response);
      std::cerr << "Invalid response size" << std::endl;
      return -1;
    }

    // 假设返回数据的结构是：地址 + 功能码 + 数据长度 + 数据（2字节位置值）+ CRC（2字节）
    uint16_t valueRaw = (response[3] << 8) | response[4];


    auto value = static_cast<double>(valueRaw);

    // 返回读取值
    return value;
  }

  std::vector<uint8_t> dh_gripper::readResponse(int num)
  {
    std::vector<uint8_t> response(0);

    serial_->read(response,num);
    return response;
  }

  void dh_gripper::handle_gripper_distance(const tosor_msgs::msg::GripperDistance::SharedPtr msg)
  {
    //distance是百分比
    setGripperPosition(0x01,msg->distance);
    //RCLCPP_INFO(this->get_logger(), "Set gripper distance to %d", msg->distance);
    //TODO是否需要延时
  }

  void dh_gripper::pub_gripper_states()
  {
    auto gripper_states_msg = tosor_msgs::msg::GripperStates();
    if(is_init_)
    {
      gripper_states_msg.position = readGripperPosition(0x01);
      usleep(1000);
      int status = readGripperStatus(0x01);
      if(status<0 || status>3)
      {
        RCLCPP_ERROR(this->get_logger(),"Error: Invalid status value.%f",status);

        gripper_states_msg.status=IS_ERRORED;
      }
      else
      {
        gripper_states_msg.status=status;
      }
      pub_gripper_states_->publish(gripper_states_msg);

    }
    else
    {
      usleep(1000000);
    }





  }
} // tosor
void readPosition(Tosor::dh_gripper* gripper) {
  usleep(20000);
  while (rclcpp::ok()) {
    // 获取当前夹爪的位置
    double position = gripper->readGripperPosition(0x01);

    // 加锁并更新位置
    // std::lock_guard<std::mutex> lock(gripper_position_mutex);
    current_position = position;

    std::cout << "Current position: " << current_position << std::endl;

    // 休眠
    usleep(100000);
  }
}

int main(int argc, char *argv[]) {
  std::cout<<"hello world"<<std::endl;
  rclcpp::init(argc, argv);  // 初始化ROS 2环境，确保上下文被创建
  //建立类对象
  // auto node =
  rclcpp::spin(std::make_shared<Tosor::dh_gripper>("dh_gripper_node"));
  //打开串口
  // node->Open("/dev/ttyUSB0",115200);
  //初始化夹爪
  // node->initializeGripper();
  // usleep(5000000);
  // 创建并启动读取夹爪位置的线程
  // std::thread position_thread(readPosition, node.get());
  //设置位置

  // node->setGripperPosition(0x01,500);
  // node->setGripperPosition(0x01,300);
  //读取实时位置
  // double position = node->readGripperPosition(0x01);
  // std::cout<<"position:"<<position<<std::endl;

  // usleep(5000000);
  rclcpp::shutdown();
  usleep(5000000);
  return 0;
}
