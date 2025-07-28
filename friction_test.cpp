
//
// 正确PT协议的摩擦力测试程序
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

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEVICE_TYPE VCI_USBCAN2
#define DEVICE_INDEX 0
#define CAN_INDEX 0

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
    int motor_id = 1;
    int motor_type = 0;              // 电机型号索引
    float torque_start = 0.0f;
    float torque_step = 0.1f;
    float torque_max = 4.0f;
    float position_threshold = 0.02f;
    int wait_time_ms = 500;
    bool debug_mode = true;
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
    bool SendPTCommand(float kp, float kd, float target_pos_rad, float target_speed_rads, float target_torque_nm) {
        VCI_CAN_OBJ frame;
        memset(&frame, 0, sizeof(frame));
        frame.ID = config.motor_id;
        frame.DataLen = 8;
        
        // 将浮点参数转换为整数 (根据电机端代码)
        int INThPT_KP = float_to_uint(kp, currentMotor.KP_MINX, currentMotor.KP_MAXX, 12);
        int INThPT_KD = float_to_uint(kd, currentMotor.KD_MINX, currentMotor.KD_MAXX, 9);
        int INTtargetpos_rad = float_to_uint(target_pos_rad, currentMotor.POS_MINX, currentMotor.POS_MAXX, 16);
        int INTtargetspeed_rads = float_to_uint(target_speed_rads, currentMotor.SPD_MINX, currentMotor.SPD_MAXX, 12);
        int INTtargettorque_NM = float_to_uint(target_torque_nm, currentMotor.T_MINX, currentMotor.T_MAXX, 12);
        
        // 根据电机端代码的解析方式编码数据
        // PT_mode.INThPT_KP = (RxData[0] << 7) + (RxData[1] >> 1);
        frame.Data[0] = (INThPT_KP >> 7) & 0xFF;
        frame.Data[1] = ((INThPT_KP & 0x7F) << 1) | ((INThPT_KD >> 8) & 0x1);
        
        // PT_mode.INThPT_KD = ((RxData[1] & 0X1) << 8) + RxData[2];
        frame.Data[2] = INThPT_KD & 0xFF;
        
        // PT_mode.INTtargetpos_rad = (RxData[3] << 8) + RxData[4];
        frame.Data[3] = (INTtargetpos_rad >> 8) & 0xFF;
        frame.Data[4] = INTtargetpos_rad & 0xFF;
        
        // PT_mode.INTtargetspeed_rads = ((RxData[5]) << 4) + (RxData[6] >> 4);
        frame.Data[5] = (INTtargetspeed_rads >> 4) & 0xFF;
        frame.Data[6] = ((INTtargetspeed_rads & 0xF) << 4) | ((INTtargettorque_NM >> 8) & 0xF);
        
        // PT_mode.INTtargettorque_NM = ((RxData[6] & 0XF) << 8) + RxData[7];
        frame.Data[7] = INTtargettorque_NM & 0xFF;
        
        if (config.debug_mode) {
            cout << "[PT命令] KP:" << kp << " KD:" << kd << " Pos:" << target_pos_rad 
                 << " Spd:" << target_speed_rads << " Torque:" << target_torque_nm << "NM" << endl;
            cout << "编码值: KP=" << INThPT_KP << " KD=" << INThPT_KD 
                 << " Pos=" << INTtargetpos_rad << " Spd=" << INTtargetspeed_rads 
                 << " Torque=" << INTtargettorque_NM << endl;
        }
        
        return SendCANFrame(frame);
    }
    
    // 解析PT模式反馈数据
    struct PTFeedback {
        bool valid = false;
        float position_rad = 0.0f;
        float speed_rads = 0.0f;
        float current_A = 0.0f;
        float coil_temp = 0.0f;
        float board_temp = 0.0f;
        uint8_t motor_error = 0;
    };
    
    PTFeedback ParsePTFeedback(const VCI_CAN_OBJ& frame) {
        PTFeedback feedback;
        
        if (frame.ID != config.motor_id || frame.DataLen != 8) {
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
    
    // 获取PT模式反馈
    PTFeedback GetPTFeedback() {
        PTFeedback feedback;
        
        auto frames = ReceiveCANFrames();
        for (const auto& frame : frames) {
            PTFeedback temp = ParsePTFeedback(frame);
            if (temp.valid) {
                if (config.debug_mode) {
                    cout << "PT反馈: Pos=" << fixed << setprecision(4) << temp.position_rad 
                         << "rad, Spd=" << temp.speed_rads << "rad/s, I=" << temp.current_A 
                         << "A, Err=" << (int)temp.motor_error << endl;
                }
                return temp;
            }
        }
        
        return feedback;
    }
    
    // 等待稳定的位置反馈
    float GetStablePosition() {
        vector<float> positions;
        
        for (int i = 0; i < 5; i++) {
            // 发送一个查询命令或PT命令来获取反馈
            SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Sleep(50);
            
            PTFeedback feedback = GetPTFeedback();
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
            cout << "稳定位置: " << fixed << setprecision(4) << mean << " rad" << endl;
        }
        
        return mean;
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
    }
    
    // 测试PT模式基本功能
    bool TestPTMode() {
        cout << "\n=== 测试PT模式功能 ===" << endl;
        
        // 发送一个测试PT命令
        cout << "发送测试PT命令 (KP=0, KD=0, Torque=0.5NM)..." << endl;
        if (!SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.5f)) {
            cout << "发送PT命令失败！" << endl;
            return false;
        }
        
        Sleep(200);
        
        // 检查反馈
        PTFeedback feedback = GetPTFeedback();
        if (!feedback.valid) {
            cout << "❌ 没有收到PT模式反馈" << endl;
            return false;
        }
        
        cout << "✅ PT模式正常工作！" << endl;
        cout << "当前位置: " << feedback.position_rad << " rad" << endl;
        cout << "当前电流: " << feedback.current_A << " A" << endl;
        
        // 停止
        SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        Sleep(200);
        
        return true;
    }
    
    // 执行摩擦力测试
    float TestFrictionInDirection(float direction) {
        cout << "\n测试" << (direction > 0 ? "正" : "负") << "向摩擦力..." << endl;
        
        // 多次获取稳定位置，取平均值
        vector<float> initial_positions;
        for (int i = 0; i < 3; i++) {
            float pos = GetStablePosition();
            if (!isnan(pos)) {
                initial_positions.push_back(pos);
            }
            Sleep(200);
        }
        
        if (initial_positions.empty()) {
            cout << "无法获取初始位置！" << endl;
            return 0.0f;
        }
        
        // 计算平均初始位置
        float initial_pos = 0;
        for (float pos : initial_positions) {
            initial_pos += pos;
        }
        initial_pos /= initial_positions.size();
        
        cout << "初始位置: " << fixed << setprecision(4) << initial_pos << " rad" << endl;
        
        float test_torque = config.torque_start;
        vector<float> recent_positions;
        
        while (test_torque <= config.torque_max) {
            float actual_torque = test_torque * direction;
            
            // 限制扭矩在电机范围内
            if (actual_torque < currentMotor.T_MINX || actual_torque > currentMotor.T_MAXX) {
                cout << "扭矩超出电机范围，跳过: " << actual_torque << " NM" << endl;
                test_torque += config.torque_step;
                continue;
            }
            
            cout << "\n测试扭矩: " << fixed << setprecision(3) << actual_torque << " NM" << endl;
            
            // 发送PT命令：KP=0, KD=0, 只施加扭矩
            if (!SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, actual_torque)) {
                cout << "发送PT命令失败！" << endl;
                continue;
            }
            
            Sleep(config.wait_time_ms);
            
            // 获取多次反馈，确保稳定
            vector<PTFeedback> feedbacks;
            for (int i = 0; i < 3; i++) {
                PTFeedback feedback = GetPTFeedback();
                if (feedback.valid) {
                    feedbacks.push_back(feedback);
                }
                Sleep(50);
            }
            
            if (feedbacks.empty()) {
                cout << "获取反馈失败！" << endl;
                continue;
            }
            
            // 使用最新的反馈
            PTFeedback current_feedback = feedbacks.back();
            float position_change = fabs(current_feedback.position_rad - initial_pos);
            
            // 记录最近的位置
            recent_positions.push_back(current_feedback.position_rad);
            if (recent_positions.size() > 5) {
                recent_positions.erase(recent_positions.begin());
            }
            
            cout << "位置变化: " << fixed << setprecision(4) << position_change << " rad";
            cout << ", 电流: " << current_feedback.current_A << " A" << endl;
            
            // 检查是否超过阈值，并且确认是持续的移动
            if (position_change > config.position_threshold) {
                // 额外验证：检查是否有持续的移动趋势
                if (recent_positions.size() >= 3) {
                    float trend = recent_positions.back() - recent_positions[0];
                    float expected_direction = (direction > 0) ? 1.0f : -1.0f;
                    
                    if (trend * expected_direction > 0 && fabs(trend) > config.position_threshold * 0.5f) {
                        cout << "🎯 检测到显著且持续的移动！静摩擦力约为: " << test_torque << " NM" << endl;
                        cout << "移动趋势: " << trend << " rad (符合预期方向)" << endl;
                        
                        // 停止施加扭矩
                        SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                        Sleep(500);
                        
                        return test_torque;
                    } else {
                        cout << "位置变化可能是噪声，继续测试..." << endl;
                    }
                } else {
                    cout << "需要更多数据点确认移动，继续测试..." << endl;
                }
            }
            
            test_torque += config.torque_step;
        }
        
        cout << "达到最大扭矩，未检测到明显移动" << endl;
        SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        return config.torque_max;
    }
    
    void RunFrictionTest() {
        cout << "\n=== PT模式摩擦力测试 ===" << endl;
        
        // 先测试PT模式是否工作
        if (!TestPTMode()) {
            cout << "PT模式测试失败，无法进行摩擦力测试" << endl;
            return;
        }
        
        Sleep(1000);
        
        // 测试正向摩擦力
        float friction_positive = TestFrictionInDirection(1.0f);
        
        // 重要：让关节回到中性位置
        cout << "\n=== 复位关节到中性位置 ===" << endl;
        SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        Sleep(2000);
        
        // 可选：尝试用小的反向扭矩帮助复位
        cout << "应用小的复位扭矩..." << endl;
        float reset_torque = (friction_positive > 0.5f) ? -0.3f : -0.1f;
        SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, reset_torque);
        Sleep(1000);
        SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        Sleep(2000);
        
        // 测试负向摩擦力
        float friction_negative = TestFrictionInDirection(-1.0f);
        
        // 显示结果
        cout << "\n=== 摩擦力测试结果 ===" << endl;
        cout << "电机型号: " << currentMotor.model << endl;
        cout << "正向静摩擦力: " << friction_positive << " NM" << endl;
        cout << "负向静摩擦力: " << friction_negative << " NM" << endl;
        
        // 修正负向结果的显示
        if (friction_negative < 0.05f && friction_positive > 0.5f) {
            cout << "⚠️ 注意：负向结果可能受位置偏移影响" << endl;
            cout << "建议负向摩擦力约为: " << friction_positive * 0.8f << " ~ " << friction_positive * 1.2f << " NM" << endl;
        }
        
        float avg_friction = (friction_positive + friction_negative) / 2.0f;
        if (friction_negative < 0.05f) {
            avg_friction = friction_positive;  // 使用正向结果作为参考
        }
        
        cout << "估计平均静摩擦力: " << avg_friction << " NM" << endl;
        
        // 保存结果
        ofstream file("pt_friction_results.txt");
        if (file.is_open()) {
            file << "=== PT模式摩擦力测试结果 ===" << endl;
            file << "电机型号: " << currentMotor.model << endl;
            file << "减速比: " << currentMotor.def_ratio << endl;
            file << "扭矩常数KT: " << currentMotor.KT << endl;
            file << "正向静摩擦力: " << friction_positive << " NM" << endl;
            file << "负向静摩擦力: " << friction_negative << " NM" << endl;
            file << "估计平均静摩擦力: " << avg_friction << " NM" << endl;
            
            // 添加测试过程的详细信息
            file << "\n=== 测试参数 ===" << endl;
            file << "位置阈值: " << config.position_threshold << " rad" << endl;
            file << "扭矩步进: " << config.torque_step << " NM" << endl;
            file << "等待时间: " << config.wait_time_ms << " ms" << endl;
            
            file << "\n=== 建议 ===" << endl;
            if (avg_friction > 2.0f) {
                file << "摩擦力较大，建议检查关节润滑状态" << endl;
            } else if (avg_friction < 0.5f) {
                file << "摩擦力较小，关节状态良好" << endl;
            } else {
                file << "摩擦力正常范围" << endl;
            }
            
            file.close();
            cout << "结果已保存到: pt_friction_results.txt" << endl;
        }
    }
    
    void Cleanup() {
        if (can_initialized) {
            SendPTCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Sleep(100);
            VCI_CloseDevice(DEVICE_TYPE, DEVICE_INDEX);
            can_initialized = false;
        }
    }
    
    ~CorrectPTTester() {
        Cleanup();
    }
};

int main() {
    cout << "=== 正确PT协议摩擦力测试程序 ===" << endl;
    
    // 显示可用电机型号
    cout << "\n可用电机型号:" << endl;
    for (size_t i = 0; i < sizeof(motorParams)/sizeof(motorParams[0]); i++) {
        cout << i << ": " << motorParams[i].model 
             << " (扭矩范围: " << motorParams[i].T_MINX << " ~ " << motorParams[i].T_MAXX << " NM)" << endl;
    }
    
    CorrectPTTester tester;
    TestConfig config;
    
    // 配置参数
    cout << "\n配置测试参数:" << endl;
    cout << "电机ID [" << config.motor_id << "]: ";
    string input;
    getline(cin, input);
    if (!input.empty()) config.motor_id = stoi(input);
    
    cout << "电机型号 (0-9) [" << config.motor_type << "]: ";
    getline(cin, input);
    if (!input.empty()) {
        int type = stoi(input);
        if (type >= 0 && type < sizeof(motorParams)/sizeof(motorParams[0])) {
            config.motor_type = type;
        }
    }
    
    cout << "最大测试扭矩 [" << config.torque_max << "]: ";
    getline(cin, input);
    if (!input.empty()) config.torque_max = stof(input);
    
    cout << "扭矩步进 [" << config.torque_step << "]: ";
    getline(cin, input);
    if (!input.empty()) config.torque_step = stof(input);
    
    tester.SetConfig(config);
    
    cout << "\n⚠️ 安全提醒：确保关节可以自由移动，周围无障碍物" << endl;
    cout << "按回车开始测试: ";
    getline(cin, input);
    
    if (!tester.Initialize()) {
        cout << "初始化失败！" << endl;
        return -1;
    }
    
    tester.RunFrictionTest();
    
    return 0;
}
