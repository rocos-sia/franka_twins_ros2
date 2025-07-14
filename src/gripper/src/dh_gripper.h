/**
  ******************************************************************************
  * @file           : dh_gripper.h
  * @author         : sun
  * @brief          : None
  * @attention      : None
  * @date           : 11/25/24
  ******************************************************************************
  */

#ifndef DH_GRIPPER_H
#define DH_GRIPPER_H
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "tosor_msgs/msg/gripper_distance.hpp"
#include "tosor_msgs/msg/gripper_states.hpp"
#include <iostream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <unistd.h>
#define WRITE_REGISTER 0x06
#define READ_REGISTER 0x03
#define POSITION_REGISTER 0x0103
#define FORCE_REGISTER 0x0101
#define INIT_REGISTER 0x0100
#define FB_POSITION_REGISTER 0x0202
#define FB_GRIPPER_STATUS_REGISTER 0x0201
#define FB_GRIPPER_INIT_REGISTER 0x0200
#define IS_RUNNING 0  // 表示夹爪正在运行
#define IS_STOPPED 1  // 表示夹爪已停止
#define IS_HOLDING 2  // 表示夹爪正在加持物体
#define IS_DROPPED 3  // 表示夹爪已掉落物体
#define IS_ERRORED 4  // 表示夹爪出现错误
namespace Tosor {

class dh_gripper :public rclcpp::Node {
    typedef std::shared_ptr<serial::Serial> SerialPtr;
public:
  explicit dh_gripper(const std::string &nodeName, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node(nodeName, options) {


      // sub_gripper_diatance_=this->create_subscription<tosor_msgs::msg::GripperDistance>("gripper/command",10,std::bind(&dh_gripper::handle_gripper_distance,this,std::placeholders::_1));
      // pub_gripper_states_=this->create_publisher<tosor_msgs::msg::GripperStates>("gripper/states",10);
      // timer_=this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&dh_gripper::pub_gripper_states,this));
      // Open("/dev/ttyUSB0",115200);
      // initializeGripper();
      // 声明参数
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 115200);

    // 读取参数
    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();

    // 创建话题（会自动加上命名空间）
    sub_gripper_diatance_ = this->create_subscription<tosor_msgs::msg::GripperDistance>(
        "gripper/command", 10, std::bind(&dh_gripper::handle_gripper_distance, this, std::placeholders::_1));

    pub_gripper_states_ = this->create_publisher<tosor_msgs::msg::GripperStates>("gripper/states", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                std::bind(&dh_gripper::pub_gripper_states, this));

    // 初始化串口
    Open(port, baudrate);
    initializeGripper();
  }



public:
    // 打开串口
    bool Open(const std::string &port = "/dev/ttyUSB0", uint32_t baudrate = 115200);
    // CRC 计算函数
    uint16_t calculateCRC(const std::vector<uint8_t>& data);
    // 打印十六进制数据
    void printHex(const std::vector<uint8_t>& data);
    std::vector<uint8_t> createModbusMessage(uint8_t address, uint8_t functionCode, uint16_t registerAddress, uint16_t value);
    // 示例函数：初始化夹爪
    int  initializeGripper(uint8_t address = 0x01);
    // 示例函数：设置夹爪力
    int setGripperForce(uint8_t address, int forcePercent);
    // 示例函数：设置夹爪位置
    int setGripperPosition(uint8_t address, int position);
    // 示例函数：读取寄存器
    std::vector<uint8_t> readRegister(uint8_t address, uint16_t registerAddress, uint16_t numRegisters = 1) ;
    // 示例函数：读取夹爪距离
    double readGripperPosition(uint8_t address = 0x01);
    //示例函数: 读取夹爪状态
    int readGripperStatus(uint8_t address = 0x01);
    // 示例函数： 解析读取的当前位置数据并返回 double 类型的位置值
    double parseData(const std::vector<uint8_t>& response) ;
    // 示例函数：read
    std::vector<uint8_t> readResponse(int num=7);
    //handle gripper distance
    void handle_gripper_distance(const tosor_msgs::msg::GripperDistance::SharedPtr msg);
    //pub gripper states
    void pub_gripper_states();
    ~dh_gripper()
    {

    };
private:
    SerialPtr serial_; // 串口对象
    std::string port_ = "/dev/ttyUSB0"; // 串口号
    uint32_t baudrate_ = 115200; // 波特率
    serial::bytesize_t data_bit_ = serial::eightbits;
    serial::stopbits_t stop_bit_ = serial::stopbits_one;
    serial::parity_t parity_ = serial::parity_none;
    serial::flowcontrol_t flow_control_ = serial::flowcontrol_none;
    int timeout_ms_ = 1000; // 超时时间
    std::mutex mtx_; // 互斥锁
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<tosor_msgs::msg::GripperDistance>::SharedPtr sub_gripper_diatance_;
    rclcpp::Publisher<tosor_msgs::msg::GripperStates>::SharedPtr pub_gripper_states_;
    bool is_init_{false};

};



} // tosor

#endif //DH_GRIPPER_H
