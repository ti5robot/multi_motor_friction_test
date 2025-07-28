
//
// Pure C++ Friction Test for Real Robot
// 纯C++摩擦力测试系统 - 真实机器人版本
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <vector>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <numeric>
#include <memory>

// 包含CAN协议定义
#include "can_protocol.h"

namespace friction_test {

// 日志级别枚举 - 避免与宏冲突
enum class LogLevel {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARN = 2,
    LOG_ERROR = 3
};

// 简单的日志类
class Logger {
public:
    static void setLevel(LogLevel level) { log_level_ = level; }
    
    static void debug(const std::string& msg) { log(LogLevel::LOG_DEBUG, "DEBUG", msg); }
    static void info(const std::string& msg) { log(LogLevel::LOG_INFO, "INFO", msg); }
    static void warn(const std::string& msg) { log(LogLevel::LOG_WARN, "WARN", msg); }
    static void error(const std::string& msg) { log(LogLevel::LOG_ERROR, "ERROR", msg); }
    
private:
    static LogLevel log_level_;
    static void log(LogLevel level, const std::string& prefix, const std::string& msg);
};

// 电机数据结构
struct MotorData {
    double kp = 0.0;
    double kd = 0.0;
    double pos_des = 0.0;
    double vel_des = 0.0;
    double ff = 0.0;
    
    // 反馈数据
    double angle_actual_rad = 0.0;
    double speed_actual_rad = 0.0;
    double current_actual_float = 0.0;
    double temperature = 0.0;
    
    // 时间戳
    std::chrono::high_resolution_clock::time_point timestamp;
};

// 电机类型定义
enum class ActuatorType {
    LSG_20_90_7090 = 0,     // 髋关节电机
    LSG_10_414 = 1,         // 膝关节电机  
    LSG_17_80_6070_new = 2, // 踝关节电机
    LSG_14_70_5060 = 3      // 其他关节电机
};

// 电机规格信息
struct ActuatorSpec {
    double tMaxX;    // 最大扭矩 (Nm)
    double tMinX;    // 最小扭矩 (Nm)
    double iMaxX;    // 最大电流 (A)
    double iMinX;    // 最小电流 (A)
    double vMaxX;    // 最大速度 (rad/s)
    double vMinX;    // 最小速度 (rad/s)
    double pMaxX;    // 最大位置 (rad)
    double pMinX;    // 最小位置 (rad)
};

// 测试参数结构体
struct TestParams {
    double test_velocity = 0.5;         // 测试速度 (rad/s)
    double test_duration = 10.0;        // 测试持续时间 (s)
    int samples_per_second = 100;       // 采样频率 (Hz)
    double position_amplitude = 1.0;    // 位置摆动幅度 (rad)
    double kp_test = 50.0;              // 测试时的Kp值
    double kd_test = 1.0;               // 测试时的Kd值
    std::string output_file = "friction_test_results.txt";
    
    // 安全参数
    double max_test_torque_ratio = 0.3;  // 最大测试扭矩比例
    double max_test_current = 10.0;      // 最大测试电流 (A)
    double max_temperature = 80.0;       // 最大温度 (°C)
    double position_tolerance = 0.1;     // 位置容差 (rad)
    
    // 测试算法参数
    double static_friction_step = 0.1;   // 静摩擦测试步长 (Nm)
    double velocity_threshold = 0.05;    // 运动检测阈值 (rad/s)
    double steady_state_time = 2.0;      // 稳态时间 (s)
};

// 单个电机的摩擦力测试结果
struct MotorFrictionResult {
    int motor_id;
    ActuatorType motor_type;
    double static_friction = 0.0;       // 静摩擦力 (Nm)
    double kinetic_friction = 0.0;      // 动摩擦力 (Nm)
    double viscous_coefficient = 0.0;   // 粘性摩擦系数
    double coulomb_friction = 0.0;      // 库伦摩擦力
    double test_duration = 0.0;         // 实际测试时间
    int data_points = 0;                // 数据点数量
    bool test_passed = false;           // 测试是否通过
    std::string error_message;          // 错误信息
    
    // 详细统计信息
    double max_current = 0.0;           // 最大电流
    double max_temperature = 0.0;       // 最大温度
    double position_error_rms = 0.0;    // 位置误差RMS
};

// 测试数据点
struct TestDataPoint {
    double timestamp;
    double position;
    double velocity;
    double torque;
    double current;
    double temperature;
    double command_torque;
};

// CAN通信管理器
class CANManager {
public:
    CANManager();
    ~CANManager();
    
    // 初始化CAN设备
    bool initialize();
    bool saveRawData(const std::string& filename);
    
    // 发送电机命令
    bool sendMotorCommand(int motor_index, const MotorData& cmd);
    
    // 读取电机反馈
    bool readMotorFeedback(int motor_index, MotorData& feedback);
    
    // 批量发送命令
    bool sendAllMotorCommands(const std::vector<MotorData>& commands);
    
    // 紧急停止所有电机
    void emergencyStopAll();
    
    // 关闭CAN设备
    void shutdown();
    
    // 检查连接状态
    bool isConnected() const { return is_connected_; }

private:
    bool is_connected_;
    int armr_device_;  // 右腿CAN设备
    int arml_device_;  // 左腿CAN设备
    int body_device_;  // 身体CAN设备
    
    std::mutex can_mutex_;
    
    // CAN设备初始化
    bool initializeCanDevice();
    bool findAndBindDevices();
    
    // 数据转换函数
    void motorDataToCanMessage(const MotorData& data, int motor_id, VCI_CAN_OBJ& msg);
    void canMessageToMotorData(const VCI_CAN_OBJ& msg, int motor_id, MotorData& data);
    
    // 获取电机设备索引
    int getDeviceIndex(int motor_index);
};

// 摩擦力测试类
class FrictionTester {
public:
    FrictionTester();
    ~FrictionTester();
    
    // 初始化测试系统
    bool initialize();
    bool saveRawData(const std::string& filename);
    
    // 设置测试参数
    void setTestParams(const TestParams& params);
    
    // 执行单个电机的摩擦力测试
    MotorFrictionResult testSingleMotor(int motor_index);
    
    // 执行所有电机的摩擦力测试
    std::vector<MotorFrictionResult> testAllMotors();
    
    // 生成测试报告
    bool generateReport(const std::vector<MotorFrictionResult>& results);
    
    // 安全检查
    bool safetyCheck();
    
    // 紧急停止
    void emergencyStop();
    
    // 设置日志级别
    void setLogLevel(LogLevel level) { Logger::setLevel(level); }
    
    // 公有静态访问函数
    static int getMotorIdByIndex(int motor_index);
    static const std::vector<int>& getMotorIdList();
    static int findMotorIndexById(int motor_id);

private:
    TestParams params_;
    std::vector<TestDataPoint> test_data_;
    std::vector<MotorData> motor_commands_;
    std::unique_ptr<CANManager> can_manager_;
    
    std::atomic<bool> is_initialized_;
    std::atomic<bool> emergency_stop_flag_;
    std::atomic<bool> test_running_;
    
    std::mutex data_mutex_;
    std::condition_variable data_ready_;
    
    // 电机规格数据
    static const std::vector<ActuatorSpec> actuator_specs_;
    static const std::vector<int> motor_id_list_;
    
    // 内部测试方法
    bool performStaticFrictionTest(int motor_index, double& static_friction);
    bool performKineticFrictionTest(int motor_index, double& kinetic_friction, double& viscous_coeff);
    bool performCoulombFrictionTest(int motor_index, double& coulomb_friction);
    
    // 数据采集和处理
    bool collectTestData(int motor_index, double duration);
    void clearTestData();
    
    // 数学分析方法
    double calculateMeanTorque(const std::vector<double>& torques);
    double calculateStandardDeviation(const std::vector<double>& data);
    bool linearRegression(const std::vector<double>& x, const std::vector<double>& y, 
                         double& slope, double& intercept, double& r_squared);
    
    // 安全监控
    bool checkMotorSafety(int motor_index, const MotorData& feedback);
    bool checkTemperature(int motor_index, double temperature);
    bool checkCurrent(int motor_index, double current);
    bool checkPosition(int motor_index, double position);
    
    // 电机控制辅助方法
    void initializeMotorCommands();
    bool setMotorToHome(int motor_index, double timeout = 10.0);
    void disableAllMotors();
    bool waitForMotorReady(int motor_index, double timeout = 5.0);
    
    // 数据处理
    double currentToTorque(double current, ActuatorType type);
    ActuatorType getMotorType(int motor_id);
    ActuatorSpec getMotorSpec(ActuatorType type);
    
    // 文件I/O
    bool loadTestConfiguration(const std::string& filename);
};

// 测试配置和阈值
namespace TestConfig {
    // 摩擦力测试阈值
    constexpr double MAX_STATIC_FRICTION = 50.0;      // 最大静摩擦力 (Nm)
    constexpr double MAX_KINETIC_FRICTION = 30.0;     // 最大动摩擦力 (Nm)  
    constexpr double MAX_VISCOUS_COEFFICIENT = 5.0;   // 最大粘性摩擦系数
    
    // 物理限制
    constexpr double MAX_POSITION_RANGE = 6.28;       // 最大位置范围 (2π rad)
    constexpr double MAX_VELOCITY_RANGE = 10.0;       // 最大速度范围 (rad/s)
    constexpr double MIN_TEST_DURATION = 1.0;         // 最小测试时间 (s)
    constexpr double MAX_TEST_DURATION = 60.0;        // 最大测试时间 (s)
    
    // 数据质量要求
    constexpr int MIN_DATA_POINTS = 50;               // 最小数据点数
    constexpr double MIN_R_SQUARED = 0.7;             // 最小相关系数
    constexpr double MAX_NOISE_RATIO = 0.2;           // 最大噪声比
}

// 辅助函数
bool isMotorIdValid(int motor_id);
std::string getMotorTypeName(ActuatorType type);
std::string getCurrentTimeString();
void printTestProgress(int current_motor, int total_motors, double progress);
double clamp(double value, double min_val, double max_val);

// 数据转换辅助函数（从您的原始代码移植）
int float_to_uint(double x, double x_min, double x_max, int bits);
double uint_to_float(int x_int, double x_min, double x_max, int bits);

// 调试模式设置函数
void setDebugMode(bool enabled, bool show_raw = false);

} // namespace friction_test
