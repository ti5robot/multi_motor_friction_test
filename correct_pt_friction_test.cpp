
//
// 正确PT协议的摩擦力测试程序 - 32关节版本
// 基于电机端实际代码实现的正确PT模式协议
//

#include "controlcan.h"
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <getopt.h>
#include <chrono>
#include <sstream>

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEVICE_TYPE VCI_USBCAN2
#define DEVICE_INDEX 0
#define CAN_INDEX 0

// 32个关节的ID定义 (1-40, 覆盖32个实际关节)
const std::vector<int> ALL_JOINT_IDS = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40
};

// 电机参数定义 (根据提供的电机型号表)
struct MotorParams {
    string model;
    float def_ratio;
    float KT;
    float T_MINX, T_MAXX;
    float I_MINX, I_MAXX;
    float KP_MINX, KP_MAXX;
    float KD_MINX, KD_MAXX;
    float POS_MINX, POS_MAXX;
    float SPD_MINX, SPD_MAXX;
};

// 预定义的电机参数
MotorParams motorParams[] = {
    {"30-40",   101, 0.024f, -30.0f, 30.0f,   -30.0f, 30.0f,   0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f},
    {"40-52",   101, 0.05f,  -30.0f, 30.0f,   -30.0f, 30.0f,   0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f},
    {"50-60",   51,  0.089f, -13.2f, 13.2f,   -9.0f,  9.0f,    0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f},
    {"60-70",   51,  0.096f, -39.6f, 39.6f,   -20.0f, 20.0f,   0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f},
    {"70-80",   101, 0.118f, -30.0f, 30.0f,   -30.0f, 30.0f,   0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f},
    {"70-90",   51,  0.118f, -64.0f, 64.0f,   -22.0f, 22.0f,   0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f},
    {"80-110",  101, 0.143f, -30.0f, 30.0f,   -30.0f, 30.0f,   0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f},
    {"100-120", 51,  0.175f, -188.0f, 188.0f, -40.0f, 40.0f,   0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f},
    {"100-142", 101, 0.175f, -30.0f, 30.0f,   -30.0f, 30.0f,   0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f},
    {"110-170", 101, 0.293f, -30.0f, 30.0f,   -30.0f, 30.0f,   0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f}
};

struct TestConfig {
    vector<int> motor_ids = {1};     // 支持多个电机ID
    int motor_type = 0;              // 电机型号索引
    float torque_start = 0.0f;
    float torque_step = 0.1f;
    float torque_max = 4.0f;
    float position_threshold = 0.02f;
    int wait_time_ms = 500;
    bool debug_mode = true;
    bool test_all_joints = false;
    string output_file = "pt_friction_results.txt";
};

// 单个关节的测试结果
struct JointResult {
    int joint_id;
    bool test_passed = false;
    float friction_positive = 0.0f;
    float friction_negative = 0.0f;
    float avg_friction = 0.0f;
    string error_message;
    double test_duration = 0.0;
};

class CorrectPTTester {
private:
    TestConfig config;
    MotorParams currentMotor;
    bool can_initialized = false;
    
    void Sleep(int ms) { usleep(ms * 1000); }
    
    void InitCANConfig(VCI_INIT_CONFIG& can_config) {
        can_config.AccCode = 0x00000000;
        can_config.AccMask = 0xFFFFFFFF;
        can_config.Reserved = 0;
        can_config.Filter = 1;
        can_config.Timing0 = 0x00;
        can_config.Timing1 = 0x14;
        can_config.Mode = 0;
    }
    
    bool SendCANFrame(const VCI_CAN_OBJ& frame) {
        if (config.debug_mode) {
            cout << "[发送] ID: 0x" << hex << setfill('0') << setw(3) << frame.ID << " 数据: ";
            for (int i = 0; i < frame.DataLen; i++) {
                cout << hex << setfill('0') << setw(2) << (int)frame.Data[i] << " ";
            }
            cout << dec << endl;
        }
        
        DWORD result = VCI_Transmit(DEVICE_TYPE, DEVICE_INDEX, CAN_INDEX, 
                                    const_cast<VCI_CAN_OBJ*>(&frame), 1);
        return (result == 1);
    }
    
    vector<VCI_CAN_OBJ> ReceiveCANFrames() {
        vector<VCI_CAN_OBJ> frames;
        VCI_CAN_OBJ buffer[10];
        DWORD count = VCI_Receive(DEVICE_TYPE, DEVICE_INDEX, CAN_INDEX, buffer, 10, 0);
        
        for (DWORD i = 0; i < count; i++) {
            frames.push_back(buffer[i]);
            if (config.debug_mode) {
                cout << "[接收] ID: 0x" << hex << setfill('0') << setw(3) << buffer[i].ID << " 数据: ";
                for (int j = 0; j < buffer[i].DataLen; j++) {
                    cout << hex << setfill('0') << setw(2) << (int)buffer[i].Data[j] << " ";
                }
                cout << dec << endl;
            }
        }
        return frames;
    }
    
    // 根据电机代码实现的转换函数
    int float_to_uint(float x, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }
    
    float uint_to_float(int x_int, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }
    
    // 正确的PT模式命令发送 (基于电机端代码)
    bool SendPTCommand(int motor_id, float kp, float kd, float target_pos_rad, float target_speed_rads, float target_torque_nm) {
        VCI_CAN_OBJ frame;
        memset(&frame, 0, sizeof(frame));
        frame.ID = motor_id;
        frame.DataLen = 8;
        
        // 将浮点参数转换为整数 (根据电机端代码)
        int INThPT_KP = float_to_uint(kp, currentMotor.KP_MINX, currentMotor.KP_MAXX, 12);
        int INThPT_KD = float_to_uint(kd, currentMotor.KD_MINX, currentMotor.KD_MAXX, 9);
        int INTtargetpos_rad = float_to_uint(target_pos_rad, currentMotor.POS_MINX, currentMotor.POS_MAXX, 16);
        int INTtargetspeed_rads = float_to_uint(target_speed_rads, currentMotor.SPD_MINX, currentMotor.SPD_MAXX, 12);
        int INTtargettorque_NM = float_to_uint(target_torque_nm, currentMotor.T_MINX, currentMotor.T_MAXX, 12);
        
        // 根据电机端代码的解析方式编码数据
        frame.Data[0] = (INThPT_KP >> 7) & 0xFF;
        frame.Data[1] = ((INThPT_KP & 0x7F) << 1) | ((INThPT_KD >> 8) & 0x1);
        frame.Data[2] = INThPT_KD & 0xFF;
        frame.Data[3] = (INTtargetpos_rad >> 8) & 0xFF;
        frame.Data[4] = INTtargetpos_rad & 0xFF;
        frame.Data[5] = (INTtargetspeed_rads >> 4) & 0xFF;
        frame.Data[6] = ((INTtargetspeed_rads & 0xF) << 4) | ((INTtargettorque_NM >> 8) & 0xF);
        frame.Data[7] = INTtargettorque_NM & 0xFF;
        
        if (config.debug_mode) {
            cout << "[PT命令] Motor:" << motor_id << " KP:" << kp << " KD:" << kd << " Pos:" << target_pos_rad 
                 << " Spd:" << target_speed_rads << " Torque:" << target_torque_nm << "NM" << endl;
        }
        
        return SendCANFrame(frame);
    }
    
    // 解析PT模式反馈数据
    struct PTFeedback {
        bool valid = false;
        int motor_id = 0;
        float position_rad = 0.0f;
        float speed_rads = 0.0f;
        float current_A = 0.0f;
        float coil_temp = 0.0f;
        float board_temp = 0.0f;
        uint8_t motor_error = 0;
    };
    
    PTFeedback ParsePTFeedback(const VCI_CAN_OBJ& frame) {
        PTFeedback feedback;
        feedback.motor_id = frame.ID;
        
        if (frame.DataLen != 8) {
            return feedback;
        }
        
        // 根据电机端反馈代码解析数据
        feedback.motor_error = frame.Data[0] - 0x01;
        
        int INTpos_rad = (frame.Data[1] << 8) | frame.Data[2];
        int INTspeed_rads = (frame.Data[3] << 4) | ((frame.Data[4] >> 4) & 0xF);
        int INTcurrent_A = ((frame.Data[4] & 0xF) << 8) | frame.Data[5];
        
        feedback.position_rad = uint_to_float(INTpos_rad, currentMotor.POS_MINX, currentMotor.POS_MAXX, 16);
        feedback.speed_rads = uint_to_float(INTspeed_rads, currentMotor.SPD_MINX, currentMotor.SPD_MAXX, 12);
        feedback.current_A = uint_to_float(INTcurrent_A, currentMotor.I_MINX, currentMotor.I_MAXX, 12);
        
        feedback.coil_temp = (frame.Data[6] - 50) / 2.0f;
        feedback.board_temp = (frame.Data[7] - 50) / 2.0f;
        
        feedback.valid = true;
        return feedback;
    }
    
    // 获取特定电机的PT模式反馈
    PTFeedback GetPTFeedback(int motor_id) {
        PTFeedback feedback;
        
        auto frames = ReceiveCANFrames();
        for (const auto& frame : frames) {
            if (frame.ID == motor_id) {
                PTFeedback temp = ParsePTFeedback(frame);
                if (temp.valid) {
                    if (config.debug_mode) {
                        cout << "PT反馈 Motor" << motor_id << ": Pos=" << fixed << setprecision(4) << temp.position_rad 
                             << "rad, Spd=" << temp.speed_rads << "rad/s, I=" << temp.current_A 
                             << "A, Err=" << (int)temp.motor_error << endl;
                    }
                    return temp;
                }
            }
        }
        
        return feedback;
    }
    
    // 等待稳定的位置反馈
    float GetStablePosition(int motor_id) {
        vector<float> positions;
        
        for (int i = 0; i < 5; i++) {
            SendPTCommand(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Sleep(50);
            
            PTFeedback feedback = GetPTFeedback(motor_id);
            if (feedback.valid) {
                positions.push_back(feedback.position_rad);
            }
            Sleep(50);
        }
        
        if (positions.empty()) {
            return NAN;
        }
        
        // 计算平均值
        float mean = 0;
        for (float pos : positions) {
            mean += pos;
        }
        mean /= positions.size();
        
        if (config.debug_mode) {
            cout << "Motor" << motor_id << " 稳定位置: " << fixed << setprecision(4) << mean << " rad" << endl;
        }
        
        return mean;
    }
    
    // 测试单个电机的摩擦力
    float TestFrictionInDirection(int motor_id, float direction) {
        cout << "\n测试Motor" << motor_id << " " << (direction > 0 ? "正" : "负") << "向摩擦力..." << endl;
        
        // 获取初始位置
        vector<float> initial_positions;
        for (int i = 0; i < 3; i++) {
            float pos = GetStablePosition(motor_id);
            if (!isnan(pos)) {
                initial_positions.push_back(pos);
            }
            Sleep(200);
        }
        
        if (initial_positions.empty()) {
            cout << "无法获取Motor" << motor_id << "初始位置！" << endl;
            return 0.0f;
        }
        
        float initial_pos = 0;
        for (float pos : initial_positions) {
            initial_pos += pos;
        }
        initial_pos /= initial_positions.size();
        
        cout << "Motor" << motor_id << " 初始位置: " << fixed << setprecision(4) << initial_pos << " rad" << endl;
        
        float test_torque = config.torque_start;
        vector<float> recent_positions;
        
        while (test_torque <= config.torque_max) {
            float actual_torque = test_torque * direction;
            
            // 限制扭矩在电机范围内
            if (actual_torque < currentMotor.T_MINX || actual_torque > currentMotor.T_MAXX) {
                test_torque += config.torque_step;
                continue;
            }
            
            cout << "Motor" << motor_id << " 测试扭矩: " << fixed << setprecision(3) << actual_torque << " NM" << endl;
            
            if (!SendPTCommand(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, actual_torque)) {
                cout << "发送PT命令失败！" << endl;
                continue;
            }
            
            Sleep(config.wait_time_ms);
            
            // 获取反馈
            vector<PTFeedback> feedbacks;
            for (int i = 0; i < 3; i++) {
                PTFeedback feedback = GetPTFeedback(motor_id);
                if (feedback.valid) {
                    feedbacks.push_back(feedback);
                }
                Sleep(50);
            }
            
            if (feedbacks.empty()) {
                cout << "获取Motor" << motor_id << "反馈失败！" << endl;
                continue;
            }
            
            PTFeedback current_feedback = feedbacks.back();
            float position_change = fabs(current_feedback.position_rad - initial_pos);
            
            recent_positions.push_back(current_feedback.position_rad);
            if (recent_positions.size() > 5) {
                recent_positions.erase(recent_positions.begin());
            }
            
            cout << "位置变化: " << fixed << setprecision(4) << position_change << " rad";
            cout << ", 电流: " << current_feedback.current_A << " A" << endl;
            
            // 检查是否超过阈值
            if (position_change > config.position_threshold) {
                if (recent_positions.size() >= 3) {
                    float trend = recent_positions.back() - recent_positions[0];
                    float expected_direction = (direction > 0) ? 1.0f : -1.0f;
                    
                    if (trend * expected_direction > 0 && fabs(trend) > config.position_threshold * 0.5f) {
                        cout << "🎯 Motor" << motor_id << " 检测到显著移动！静摩擦力约为: " << test_torque << " NM" << endl;
                        
                        SendPTCommand(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                        Sleep(500);
                        return test_torque;
                    }
                }
            }
            
            test_torque += config.torque_step;
        }
        
        cout << "Motor" << motor_id << " 达到最大扭矩，未检测到明显移动" << endl;
        SendPTCommand(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        return config.torque_max;
    }
    
public:
    bool Initialize() {
        cout << "初始化CAN通信..." << endl;
        
        if (VCI_OpenDevice(DEVICE_TYPE, DEVICE_INDEX, 0) != 1) {
            cout << "打开CAN设备失败！" << endl;
            return false;
        }
        
        VCI_INIT_CONFIG can_config;
        InitCANConfig(can_config);
        if (VCI_InitCAN(DEVICE_TYPE, DEVICE_INDEX, CAN_INDEX, &can_config) != 1) {
            cout << "初始化CAN失败！" << endl;
            VCI_CloseDevice(DEVICE_TYPE, DEVICE_INDEX);
            return false;
        }
        
        if (VCI_StartCAN(DEVICE_TYPE, DEVICE_INDEX, CAN_INDEX) != 1) {
            cout << "启动CAN失败！" << endl;
            VCI_CloseDevice(DEVICE_TYPE, DEVICE_INDEX);
            return false;
        }
        
        VCI_ClearBuffer(DEVICE_TYPE, DEVICE_INDEX, CAN_INDEX);
        can_initialized = true;
        cout << "CAN通信初始化成功！" << endl;
        return true;
    }
    
    void SetConfig(const TestConfig& new_config) {
        config = new_config;
        currentMotor = motorParams[config.motor_type];
        
        cout << "选择电机: " << currentMotor.model << endl;
        cout << "减速比: " << currentMotor.def_ratio << ", KT: " << currentMotor.KT << endl;
        cout << "扭矩范围: " << currentMotor.T_MINX << " ~ " << currentMotor.T_MAXX << " NM" << endl;
        
        if (config.test_all_joints) {
            cout << "测试模式: 全部" << config.motor_ids.size() << "个关节" << endl;
        } else {
            cout << "测试关节: ";
            for (size_t i = 0; i < config.motor_ids.size(); i++) {
                cout << config.motor_ids[i];
                if (i < config.motor_ids.size() - 1) cout << ", ";
            }
            cout << endl;
        }
    }
    
    // 测试单个关节
    JointResult TestSingleJoint(int motor_id) {
        JointResult result;
        result.joint_id = motor_id;
        
        auto start_time = chrono::steady_clock::now();
        
        try {
            cout << "\n=== 测试关节 " << motor_id << " ===" << endl;
            
            // 测试PT模式基本功能
            cout << "测试PT模式功能..." << endl;
            if (!SendPTCommand(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f)) {
                result.error_message = "发送PT命令失败";
                return result;
            }
            
            Sleep(200);
            PTFeedback feedback = GetPTFeedback(motor_id);
            if (!feedback.valid) {
                result.error_message = "没有收到PT模式反馈";
                return result;
            }
            
            cout << "✅ PT模式正常工作！" << endl;
            SendPTCommand(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Sleep(500);
            
            // 测试摩擦力
            result.friction_positive = TestFrictionInDirection(motor_id, 1.0f);
            
            // 复位
            cout << "复位关节到中性位置..." << endl;
            SendPTCommand(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Sleep(2000);
            
            result.friction_negative = TestFrictionInDirection(motor_id, -1.0f);
            
            // 计算平均摩擦力
            if (result.friction_negative < 0.05f && result.friction_positive > 0.5f) {
                result.avg_friction = result.friction_positive;
            } else {
                result.avg_friction = (result.friction_positive + result.friction_negative) / 2.0f;
            }
            
            result.test_passed = true;
            
        } catch (const exception& e) {
            result.error_message = e.what();
        }
        
        // 停止电机
        SendPTCommand(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        
        auto end_time = chrono::steady_clock::now();
        result.test_duration = chrono::duration<double>(end_time - start_time).count();
        
        return result;
    }
    
    // 运行摩擦力测试
    vector<JointResult> RunFrictionTest() {
        vector<JointResult> results;
        
        cout << "\n=== PT模式摩擦力测试 - " << config.motor_ids.size() << "个关节 ===" << endl;
        
        auto overall_start = chrono::steady_clock::now();
        
        for (size_t i = 0; i < config.motor_ids.size(); i++) {
            int motor_id = config.motor_ids[i];
            
            cout << "\n[" << (i + 1) << "/" << config.motor_ids.size() << "] ";
            
            JointResult result = TestSingleJoint(motor_id);
            results.push_back(result);
            
            // 显示结果
            if (result.test_passed) {
                cout << "✅ 关节 " << motor_id << " 测试完成" << endl;
                cout << "正向摩擦力: " << result.friction_positive << " NM" << endl;
                cout << "负向摩擦力: " << result.friction_negative << " NM" << endl;
                cout << "平均摩擦力: " << result.avg_friction << " NM" << endl;
            } else {
                cout << "❌ 关节 " << motor_id << " 测试失败: " << result.error_message << endl;
            }
            
            // 显示进度
            if (i < config.motor_ids.size() - 1) {
                auto current_time = chrono::steady_clock::now();
                auto elapsed = chrono::duration<double>(current_time - overall_start).count();
                double avg_time = elapsed / (i + 1);
                double remaining = avg_time * (config.motor_ids.size() - i - 1);
                
                cout << "进度: " << fixed << setprecision(1) 
                     << (100.0 * (i + 1) / config.motor_ids.size()) << "%, "
                     << "预计剩余: " << static_cast<int>(remaining / 60) 
                     << "m " << static_cast<int>(remaining) % 60 << "s" << endl;
                
                // 关节间休息
                cout << "冷却 5 秒..." << endl;
                Sleep(5000);
            }
        }
        
        return results;
    }
    
    // 保存结果
    bool SaveResults(const vector<JointResult>& results) {
        ofstream file(config.output_file);
        if (!file.is_open()) {
            cout << "无法创建输出文件: " << config.output_file << endl;
            return false;
        }
        
        file << "=== PT模式摩擦力测试结果 ===" << endl;
        file << "电机型号: " << currentMotor.model << endl;
        file << "减速比: " << currentMotor.def_ratio << endl;
        file << "扭矩常数KT: " << currentMotor.KT << endl;
        file << "测试时间: " << chrono::duration_cast<chrono::seconds>(chrono::system_clock::now().time_since_epoch()).count() << endl;
        file << endl;
        
        // 统计信息
        int passed = 0, failed = 0;
        double total_time = 0.0;
        double avg_friction = 0.0;
        
        for (const auto& result : results) {
            if (result.test_passed) {
                passed++;
                avg_friction += result.avg_friction;
            } else {
                failed++;
            }
            total_time += result.test_duration;
        }
        
        if (passed > 0) {
            avg_friction /= passed;
        }
        
        file << "=== 测试统计 ===" << endl;
        file << "总关节数: " << results.size() << endl;
        file << "通过: " << passed << endl;
        file << "失败: " << failed << endl;
        file << "成功率: " << fixed << setprecision(1) << (results.empty() ? 0.0 : passed * 100.0 / results.size()) << "%" << endl;
        file << "总测试时间: " << fixed << setprecision(1) << total_time / 60.0 << " 分钟" << endl;
        if (passed > 0) {
            file << "平均摩擦力: " << fixed << setprecision(3) << avg_friction << " NM" << endl;
        }
        file << endl;
        
        // 详细结果
        file << "=== 详细结果 ===" << endl;
        for (const auto& result : results) {
            file << "关节 " << result.joint_id << ": ";
            if (result.test_passed) {
                file << "通过 - 正向:" << result.friction_positive 
                     << "NM, 负向:" << result.friction_negative 
                     << "NM, 平均:" << result.avg_friction << "NM";
            } else {
                file << "失败 - " << result.error_message;
            }
            file << " (耗时:" << fixed << setprecision(1) << result.test_duration << "s)" << endl;
        }
        
        file << endl;
        file << "=== 测试参数 ===" << endl;
        file << "位置阈值: " << config.position_threshold << " rad" << endl;
        file << "扭矩步进: " << config.torque_step << " NM" << endl;
        file << "最大扭矩: " << config.torque_max << " NM" << endl;
        file << "等待时间: " << config.wait_time_ms << " ms" << endl;
        
        file.close();
        return true;
    }
    
    void Cleanup() {
        if (can_initialized) {
            // 停止所有电机
            for (int motor_id : config.motor_ids) {
                SendPTCommand(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            }
            Sleep(100);
            VCI_CloseDevice(DEVICE_TYPE, DEVICE_INDEX);
            can_initialized = false;
        }
    }
    
    ~CorrectPTTester() {
        Cleanup();
    }
};

// 解析关节列表 (支持 "1,2,3" 和 "1-8" 格式)
vector<int> parseJointList(const string& joint_str) {
    vector<int> joints;
    
    if (joint_str.find('-') != string::npos) {
        // 范围格式 "1-8"
        size_t dash_pos = joint_str.find('-');
        int start = stoi(joint_str.substr(0, dash_pos));
        int end = stoi(joint_str.substr(dash_pos + 1));
        
        if (start > end) swap(start, end);
        
        for (int i = start; i <= end; i++) {
            if (i >= 1 && i <= 40) {
                joints.push_back(i);
            }
        }
    } else {
        // 逗号分隔格式 "1,2,3"
        stringstream ss(joint_str);
        string token;
        
        while (getline(ss, token, ',')) {
            try {
                int joint_id = stoi(token);
                if (joint_id >= 1 && joint_id <= 40) {
                    joints.push_back(joint_id);
                }
            } catch (const exception& e) {
                cerr << "Warning: Invalid joint ID '" << token << "' ignored\n";
            }
        }
    }
    
    // 去重并排序
    sort(joints.begin(), joints.end());
    joints.erase(unique(joints.begin(), joints.end()), joints.end());
    
    return joints;
}

void printUsage(const char* program_name) {
    cout << "PT协议摩擦力测试程序 v2.0 - 32关节版本\n";
    cout << "用法: " << program_name << " [选项]\n\n";
    cout << "选项:\n";
    cout << "  -h, --help                显示此帮助信息\n";
    cout << "  -m, --motor ID            测试单个关节 (1-40)\n";
    cout << "  -j, --joints LIST         测试指定关节 (例如: \"1,2,3\" 或 \"1-8\")\n";
    cout << "  -A, --all-joints          测试所有32个关节 (1-40)\n";
    cout << "  -t, --motor-type TYPE     电机型号 (0-9, 默认: 0)\n";
    cout << "  --max-torque VALUE        最大测试扭矩 (默认: 4.0 NM)\n";
    cout << "  --torque-step VALUE       扭矩步进 (默认: 0.1 NM)\n";
    cout << "  --threshold VALUE         位置阈值 (默认: 0.02 rad)\n";
    cout << "  --wait-time VALUE         等待时间 (默认: 500 ms)\n";
    cout << "  -o, --output FILE         输出文件 (默认: pt_friction_results.txt)\n";
    cout << "  --debug                   启用调试输出\n";
    cout << "  --quiet                   静默模式\n";
    cout << "\n关节组:\n";
    cout << "  --left-arm                测试左臂关节 (1-8)\n";
    cout << "  --right-arm               测试右臂关节 (9-16)\n";
    cout << "  --left-leg                测试左腿关节 (17-24)\n";
    cout << "  --right-leg               测试右腿关节 (25-32)\n";
    cout << "  --upper-body              测试上半身关节 (1-16)\n";
    cout << "  --lower-body              测试下半身关节 (17-32)\n";
    cout << "\n电机型号:\n";
    for (size_t i = 0; i < sizeof(motorParams)/sizeof(motorParams[0]); i++) {
        cout << "  " << i << ": " << motorParams[i].model 
             << " (扭矩范围: " << motorParams[i].T_MINX << " ~ " << motorParams[i].T_MAXX << " NM)" << endl;
    }
    cout << "\n示例:\n";
    cout << "  " << program_name << " -A                        # 测试所有关节\n";
    cout << "  " << program_name << " -m 1                      # 测试关节1\n";
    cout << "  " << program_name << " -j \"1,2,3,4\"             # 测试指定关节\n";
    cout << "  " << program_name << " -j \"1-8\"                 # 测试关节1-8\n";
    cout << "  " << program_name << " --left-arm                # 测试左臂\n";
    cout << "  " << program_name << " --debug --max-torque 2.0  # 调试模式，限制扭矩\n";
    cout << "\n安全提醒:\n";
    cout << "  确保机器人处于安全位置，关节可自由移动\n";
    cout << "  测试过程中电机会运动！\n";
    cout << "  按 Ctrl+C 可紧急停止\n";
}

// 获取预定义关节组
vector<int> getJointGroup(const string& group_name) {
    if (group_name == "left-arm") {
        return {1, 2, 3, 4, 5, 6, 7, 8};
    } else if (group_name == "right-arm") {
        return {9, 10, 11, 12, 13, 14, 15, 16};
    } else if (group_name == "left-leg") {
        return {17, 18, 19, 20, 21, 22, 23, 24};
    } else if (group_name == "right-leg") {
        return {25, 26, 27, 28, 29, 30, 31, 32};
    } else if (group_name == "upper-body") {
        return {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    } else if (group_name == "lower-body") {
        return {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    }
    return {};
}

int main(int argc, char* argv[]) {
    TestConfig config;
    bool test_all_joints = false;
    bool quiet_mode = false;
    
    // 定义长选项
    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"motor", required_argument, 0, 'm'},
        {"joints", required_argument, 0, 'j'},
        {"all-joints", no_argument, 0, 'A'},
        {"motor-type", required_argument, 0, 't'},
        {"max-torque", required_argument, 0, 1001},
        {"torque-step", required_argument, 0, 1002},
        {"threshold", required_argument, 0, 1003},
        {"wait-time", required_argument, 0, 1004},
        {"output", required_argument, 0, 'o'},
        {"debug", no_argument, 0, 1005},
        {"quiet", no_argument, 0, 1006},
        {"left-arm", no_argument, 0, 1007},
        {"right-arm", no_argument, 0, 1008},
        {"left-leg", no_argument, 0, 1009},
        {"right-leg", no_argument, 0, 1010},
        {"upper-body", no_argument, 0, 1011},
        {"lower-body", no_argument, 0, 1012},
        {0, 0, 0, 0}
    };
    
    // 解析命令行参数
    int c;
    while ((c = getopt_long(argc, argv, "hm:j:At:o:", long_options, nullptr)) != -1) {
        switch (c) {
            case 'h':
                printUsage(argv[0]);
                return 0;
                
            case 'm':
                try {
                    int motor_id = stoi(optarg);
                    if (motor_id >= 1 && motor_id <= 40) {
                        config.motor_ids = {motor_id};
                    } else {
                        cerr << "错误: 关节ID必须在1-40范围内\n";
                        return 1;
                    }
                } catch (const exception& e) {
                    cerr << "错误: 无效的关节ID\n";
                    return 1;
                }
                break;
                
            case 'j':
                config.motor_ids = parseJointList(optarg);
                if (config.motor_ids.empty()) {
                    cerr << "错误: 没有有效的关节ID在 '" << optarg << "'\n";
                    return 1;
                }
                break;
                
            case 'A':
                test_all_joints = true;
                config.motor_ids = ALL_JOINT_IDS;
                config.test_all_joints = true;
                break;
                
            case 't':
                try {
                    config.motor_type = stoi(optarg);
                    if (config.motor_type < 0 || config.motor_type >= sizeof(motorParams)/sizeof(motorParams[0])) {
                        cerr << "错误: 电机型号必须在0-9范围内\n";
                        return 1;
                    }
                } catch (const exception& e) {
                    cerr << "错误: 无效的电机型号\n";
                    return 1;
                }
                break;
                
            case 'o':
                config.output_file = optarg;
                break;
                
            case 1001: // --max-torque
                try {
                    config.torque_max = stof(optarg);
                    if (config.torque_max <= 0 || config.torque_max > 50.0) {
                        cerr << "错误: 最大扭矩必须在0-50NM范围内\n";
                        return 1;
                    }
                } catch (const exception& e) {
                    cerr << "错误: 无效的最大扭矩值\n";
                    return 1;
                }
                break;
                
            case 1002: // --torque-step
                try {
                    config.torque_step = stof(optarg);
                    if (config.torque_step <= 0 || config.torque_step > 1.0) {
                        cerr << "错误: 扭矩步进必须在0-1.0NM范围内\n";
                        return 1;
                    }
                } catch (const exception& e) {
                    cerr << "错误: 无效的扭矩步进值\n";
                    return 1;
                }
                break;
                
            case 1003: // --threshold
                try {
                    config.position_threshold = stof(optarg);
                    if (config.position_threshold <= 0 || config.position_threshold > 1.0) {
                        cerr << "错误: 位置阈值必须在0-1.0rad范围内\n";
                        return 1;
                    }
                } catch (const exception& e) {
                    cerr << "错误: 无效的位置阈值\n";
                    return 1;
                }
                break;
                
            case 1004: // --wait-time
                try {
                    config.wait_time_ms = stoi(optarg);
                    if (config.wait_time_ms < 100 || config.wait_time_ms > 5000) {
                        cerr << "错误: 等待时间必须在100-5000ms范围内\n";
                        return 1;
                    }
                } catch (const exception& e) {
                    cerr << "错误: 无效的等待时间\n";
                    return 1;
                }
                break;
                
            case 1005: // --debug
                config.debug_mode = true;
                break;
                
            case 1006: // --quiet
                quiet_mode = true;
                config.debug_mode = false;
                break;
                
            case 1007: // --left-arm
                config.motor_ids = getJointGroup("left-arm");
                break;
                
            case 1008: // --right-arm
                config.motor_ids = getJointGroup("right-arm");
                break;
                
            case 1009: // --left-leg
                config.motor_ids = getJointGroup("left-leg");
                break;
                
            case 1010: // --right-leg
                config.motor_ids = getJointGroup("right-leg");
                break;
                
            case 1011: // --upper-body
                config.motor_ids = getJointGroup("upper-body");
                break;
                
            case 1012: // --lower-body
                config.motor_ids = getJointGroup("lower-body");
                break;
                
            case '?':
                cerr << "错误: 未知选项。使用 --help 查看帮助信息。\n";
                return 1;
                
            default:
                cerr << "错误: 参数解析异常\n";
                return 1;
        }
    }
    
    // 如果没有指定关节，使用交互模式
    if (config.motor_ids.empty() && !test_all_joints) {
        cout << "=== 正确PT协议摩擦力测试程序 v2.0 ===" << endl;
        
        // 显示可用电机型号
        cout << "\n可用电机型号:" << endl;
        for (size_t i = 0; i < sizeof(motorParams)/sizeof(motorParams[0]); i++) {
            cout << i << ": " << motorParams[i].model 
                 << " (扭矩范围: " << motorParams[i].T_MINX << " ~ " << motorParams[i].T_MAXX << " NM)" << endl;
        }
        
        cout << "\n配置测试参数:" << endl;
        
        // 交互式配置
        string input;
        
        cout << "关节选择:\n";
        cout << "  输入关节ID (例如: 1) 或\n";
        cout << "  输入关节列表 (例如: 1,2,3 或 1-8) 或\n";
        cout << "  输入 'all' 测试所有关节\n";
        cout << "选择 [1]: ";
        getline(cin, input);
        
        if (input.empty()) {
            config.motor_ids = {1};
        } else if (input == "all" || input == "ALL") {
            config.motor_ids = ALL_JOINT_IDS;
            config.test_all_joints = true;
        } else {
            config.motor_ids = parseJointList(input);
            if (config.motor_ids.empty()) {
                cout << "无效输入，默认测试关节1" << endl;
                config.motor_ids = {1};
            }
        }
        
        cout << "电机型号 (0-9) [" << config.motor_type << "]: ";
        getline(cin, input);
        if (!input.empty()) {
            try {
                int type = stoi(input);
                if (type >= 0 && type < sizeof(motorParams)/sizeof(motorParams[0])) {
                    config.motor_type = type;
                }
            } catch (const exception& e) {
                cout << "无效输入，使用默认值" << endl;
            }
        }
        
        cout << "最大测试扭矩 [" << config.torque_max << "]: ";
        getline(cin, input);
        if (!input.empty()) {
            try {
                config.torque_max = stof(input);
            } catch (const exception& e) {
                cout << "无效输入，使用默认值" << endl;
            }
        }
        
        cout << "扭矩步进 [" << config.torque_step << "]: ";
        getline(cin, input);
        if (!input.empty()) {
            try {
                config.torque_step = stof(input);
            } catch (const exception& e) {
                cout << "无效输入，使用默认值" << endl;
            }
        }
    }
    
    // 显示测试配置
    if (!quiet_mode) {
        cout << "\n=== 测试配置 ===" << endl;
        cout << "关节数量: " << config.motor_ids.size() << endl;
        cout << "关节列表: ";
        for (size_t i = 0; i < config.motor_ids.size(); i++) {
            cout << config.motor_ids[i];
            if (i < config.motor_ids.size() - 1) cout << ", ";
            if ((i + 1) % 10 == 0) cout << "\n          ";
        }
        cout << endl;
        cout << "电机型号: " << motorParams[config.motor_type].model << endl;
        cout << "最大扭矩: " << config.torque_max << " NM" << endl;
        cout << "扭矩步进: " << config.torque_step << " NM" << endl;
        cout << "位置阈值: " << config.position_threshold << " rad" << endl;
        cout << "输出文件: " << config.output_file << endl;
        
        cout << "\n⚠️ 安全提醒：确保关节可以自由移动，周围无障碍物" << endl;
        if (config.motor_ids.size() > 10) {
            double estimated_time = config.motor_ids.size() * 2.0; // 每个关节约2分钟
            cout << "预计测试时间: " << fixed << setprecision(1) << estimated_time << " 分钟" << endl;
        }
        cout << "按回车开始测试: ";
        string input;  // 声明input变量
        getline(cin, input);
    }
    
    CorrectPTTester tester;
    
    if (!tester.Initialize()) {
        cout << "初始化失败！" << endl;
        return -1;
    }
    
    tester.SetConfig(config);
    
    auto results = tester.RunFrictionTest();
    
    // 显示结果摘要
    cout << "\n=== 测试完成 ===" << endl;
    
    int passed = 0, failed = 0;
    double total_time = 0.0;
    for (const auto& result : results) {
        if (result.test_passed) passed++;
        else failed++;
        total_time += result.test_duration;
    }
    
    cout << "╔═══ 测试摘要 ═══╗" << endl;
    cout << "║ 总关节数: " << setw(6) << results.size() << " ║" << endl;
    cout << "║ 通过:     " << setw(6) << passed << " ║" << endl;
    cout << "║ 失败:     " << setw(6) << failed << " ║" << endl;
    cout << "║ 成功率:   " << setw(5) << fixed << setprecision(1) 
         << (results.empty() ? 0.0 : passed * 100.0 / results.size()) << "% ║" << endl;
    cout << "║ 总时间:   " << setw(5) << fixed << setprecision(1) 
         << total_time / 60.0 << "m ║" << endl;
    cout << "╚════════════════╝" << endl;
    
    if (failed > 0) {
        cout << "\n❌ 失败关节:" << endl;
        for (const auto& result : results) {
            if (!result.test_passed) {
                cout << "  关节 " << result.joint_id << ": " << result.error_message << endl;
            }
        }
    } else {
        cout << "\n✅ 所有关节测试通过！" << endl;
    }
    
    // 保存结果
    if (tester.SaveResults(results)) {
        cout << "结果已保存到: " << config.output_file << endl;
    } else {
        cout << "保存结果失败！" << endl;
    }
    
    return 0;
}
