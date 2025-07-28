
//
// Pure C++ Friction Test Main Program
// 纯C++摩擦力测试主程序 - 32关节机器人版本
//

#include "friction_test.h"
#include <signal.h>
#include <iostream>
#include <string>
#include <vector>
#include <getopt.h>

using namespace friction_test;

// 全局变量用于信号处理
FrictionTester* g_tester = nullptr;
std::atomic<bool> g_shutdown_requested(false);

// 32个关节的ID定义 (1-40, 覆盖32个实际关节)
const std::vector<int> ALL_JOINT_IDS = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40
};

// 信号处理函数
void signalHandler(int sig) {
    std::cout << "\nReceived signal " << sig << ", initiating emergency stop..." << std::endl;
    g_shutdown_requested = true;
    if (g_tester) {
        g_tester->emergencyStop();
    }
    exit(sig);
}

// 打印使用说明
void printUsage(const char* program_name) {
    std::cout << "Robot Friction Test System v2.0 - 32 Joint Version\n";
    std::cout << "Usage: " << program_name << " [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -h, --help                Show this help message\n";
    std::cout << "  -v, --velocity SPEED      Test velocity in rad/s (default: 0.5)\n";
    std::cout << "  -d, --duration TIME       Test duration in seconds (default: 10.0)\n";
    std::cout << "  -s, --sample-rate RATE    Sample rate in Hz (default: 100)\n";
    std::cout << "  -a, --amplitude AMP       Position amplitude in rad (default: 1.0)\n";
    std::cout << "  -o, --output FILE         Output file path (default: friction_test_results.txt)\n";
    std::cout << "  -m, --motor ID            Test single motor by ID (1-40)\n";
    std::cout << "  -j, --joints LIST         Test specific joints (e.g., \"1,2,3\" or \"1-8\")\n";
    std::cout << "  -A, --all-joints          Test all 32 joints (1-40)\n";
    std::cout << "  --kp VALUE                Kp gain for testing (default: 50.0)\n";
    std::cout << "  --kd VALUE                Kd gain for testing (default: 1.0)\n";
    std::cout << "  --max-torque-ratio RATIO  Max test torque ratio (default: 0.3)\n";
    std::cout << "  --max-current AMPS        Max test current in A (default: 10.0)\n";
    std::cout << "  --max-temp CELSIUS        Max temperature in °C (default: 80.0)\n";
    std::cout << "  --interactive             Interactive mode for parameter adjustment\n";
    std::cout << "  --debug                   Enable debug logging\n";
    std::cout << "  --quiet                   Minimize output (errors only)\n";
    std::cout << "  --save-raw FILE           Save raw test data to file\n";
    std::cout << "  --parallel                Enable parallel testing (multiple joints)\n";
    std::cout << "  --batch-size N            Number of joints to test in parallel (default: 4)\n";
    std::cout << "\nJoint Groups:\n";
    std::cout << "  --left-arm                Test left arm joints (1-8)\n";
    std::cout << "  --right-arm               Test right arm joints (9-16)\n";
    std::cout << "  --left-leg                Test left leg joints (17-24)\n";
    std::cout << "  --right-leg               Test right leg joints (25-32)\n";
    std::cout << "  --upper-body              Test upper body joints (1-16)\n";
    std::cout << "  --lower-body              Test lower body joints (17-32)\n";
    std::cout << "\nMotor IDs: 1-40 (32 active joints)\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name << " -A                        # Test all 32 joints\n";
    std::cout << "  " << program_name << " -m 1                      # Test only joint ID 1\n";
    std::cout << "  " << program_name << " -j \"1,2,3,4\"             # Test specific joints\n";
    std::cout << "  " << program_name << " -j \"1-8\"                 # Test joint range 1-8\n";
    std::cout << "  " << program_name << " --left-arm                # Test left arm\n";
    std::cout << "  " << program_name << " --parallel --batch-size 8 # Parallel testing\n";
    std::cout << "  " << program_name << " --interactive             # Interactive setup\n";
    std::cout << "\nSafety Notice:\n";
    std::cout << "  Ensure the robot is in a safe position before testing.\n";
    std::cout << "  Motors will move during the test!\n";
    std::cout << "  Press Ctrl+C for emergency stop.\n";
}

// 解析关节列表 (支持 "1,2,3" 和 "1-8" 格式)
std::vector<int> parseJointList(const std::string& joint_str) {
    std::vector<int> joints;
    
    if (joint_str.find('-') != std::string::npos) {
        // 范围格式 "1-8"
        size_t dash_pos = joint_str.find('-');
        int start = std::stoi(joint_str.substr(0, dash_pos));
        int end = std::stoi(joint_str.substr(dash_pos + 1));
        
        if (start > end) std::swap(start, end);
        
        for (int i = start; i <= end; i++) {
            if (i >= 1 && i <= 40) {
                joints.push_back(i);
            }
        }
    } else {
        // 逗号分隔格式 "1,2,3"
        std::stringstream ss(joint_str);
        std::string token;
        
        while (std::getline(ss, token, ',')) {
            try {
                int joint_id = std::stoi(token);
                if (joint_id >= 1 && joint_id <= 40) {
                    joints.push_back(joint_id);
                }
            } catch (const std::exception& e) {
                std::cerr << "Warning: Invalid joint ID '" << token << "' ignored\n";
            }
        }
    }
    
    // 去重并排序
    std::sort(joints.begin(), joints.end());
    joints.erase(std::unique(joints.begin(), joints.end()), joints.end());
    
    return joints;
}

// 获取预定义关节组
std::vector<int> getJointGroup(const std::string& group_name) {
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

// 交互式参数设置
TestParams interactiveSetup() {
    TestParams params;
    std::string input;
    
    std::cout << "\n=== Interactive Parameter Setup ===\n";
    
    // 测试速度
    std::cout << "Enter test velocity [rad/s] (default: 0.5): ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        try {
            params.test_velocity = std::stod(input);
            if (params.test_velocity <= 0 || params.test_velocity > 5.0) {
                std::cout << "Warning: Velocity should be between 0 and 5.0 rad/s\n";
                params.test_velocity = std::clamp(params.test_velocity, 0.1, 5.0);
            }
        } catch (const std::exception& e) {
            std::cout << "Invalid input, using default value.\n";
            params.test_velocity = 0.5;
        }
    } else {
        params.test_velocity = 0.5;
    }
    
    // 测试持续时间
    std::cout << "Enter test duration [s] (default: 10.0): ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        try {
            params.test_duration = std::stod(input);
            if (params.test_duration < 1.0 || params.test_duration > 60.0) {
                std::cout << "Warning: Duration should be between 1.0 and 60.0 seconds\n";
                params.test_duration = std::clamp(params.test_duration, 1.0, 60.0);
            }
        } catch (const std::exception& e) {
            std::cout << "Invalid input, using default value.\n";
            params.test_duration = 10.0;
        }
    } else {
        params.test_duration = 10.0;
    }
    
    // 采样频率
    std::cout << "Enter sample rate [Hz] (default: 100): ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        try {
            params.samples_per_second = std::stoi(input);
            if (params.samples_per_second < 10 || params.samples_per_second > 1000) {
                std::cout << "Warning: Sample rate should be between 10 and 1000 Hz\n";
                params.samples_per_second = std::clamp(params.samples_per_second, 10, 1000);
            }
        } catch (const std::exception& e) {
            std::cout << "Invalid input, using default value.\n";
            params.samples_per_second = 100;
        }
    } else {
        params.samples_per_second = 100;
    }
    
    // Kp增益
    std::cout << "Enter Kp gain (default: 50.0): ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        try {
            params.kp_test = std::stod(input);
            if (params.kp_test < 0 || params.kp_test > 500.0) {
                std::cout << "Warning: Kp should be between 0 and 500\n";
                params.kp_test = std::clamp(params.kp_test, 0.0, 500.0);
            }
        } catch (const std::exception& e) {
            std::cout << "Invalid input, using default value.\n";
            params.kp_test = 50.0;
        }
    } else {
        params.kp_test = 50.0;
    }
    
    // Kd增益
    std::cout << "Enter Kd gain (default: 1.0): ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        try {
            params.kd_test = std::stod(input);
            if (params.kd_test < 0 || params.kd_test > 10.0) {
                std::cout << "Warning: Kd should be between 0 and 10\n";
                params.kd_test = std::clamp(params.kd_test, 0.0, 10.0);
            }
        } catch (const std::exception& e) {
            std::cout << "Invalid input, using default value.\n";
            params.kd_test = 1.0;
        }
    } else {
        params.kd_test = 1.0;
    }
    
    // 最大扭矩比例
    std::cout << "Enter max torque ratio (default: 0.3): ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        try {
            params.max_test_torque_ratio = std::stod(input);
            if (params.max_test_torque_ratio <= 0 || params.max_test_torque_ratio > 0.5) {
                std::cout << "Warning: Torque ratio should be between 0 and 0.5\n";
                params.max_test_torque_ratio = std::clamp(params.max_test_torque_ratio, 0.1, 0.5);
            }
        } catch (const std::exception& e) {
            std::cout << "Invalid input, using default value.\n";
            params.max_test_torque_ratio = 0.3;
        }
    } else {
        params.max_test_torque_ratio = 0.3;
    }
    
    // 输出文件
    std::cout << "Enter output file path (default: friction_test_results.txt): ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        params.output_file = input;
    } else {
        params.output_file = "friction_test_results.txt";
    }
    
    return params;
}

// 显示测试前确认信息
bool confirmTestStart(const TestParams& params, const std::vector<int>& test_joints, bool parallel_mode = false) {
    std::cout << "\n=== Test Configuration ===\n";
    std::cout << "Test Velocity: " << params.test_velocity << " rad/s\n";
    std::cout << "Test Duration: " << params.test_duration << " s\n";
    std::cout << "Sample Rate: " << params.samples_per_second << " Hz\n";
    std::cout << "Kp Gain: " << params.kp_test << "\n";
    std::cout << "Kd Gain: " << params.kd_test << "\n";
    std::cout << "Max Torque Ratio: " << params.max_test_torque_ratio << "\n";
    std::cout << "Max Current: " << params.max_test_current << " A\n";
    std::cout << "Max Temperature: " << params.max_temperature << " °C\n";
    std::cout << "Output File: " << params.output_file << "\n";
    std::cout << "Parallel Mode: " << (parallel_mode ? "Enabled" : "Disabled") << "\n";
    
    std::cout << "Target Joints (" << test_joints.size() << "): ";
    for (size_t i = 0; i < test_joints.size(); i++) {
        std::cout << test_joints[i];
        if (i < test_joints.size() - 1) std::cout << ", ";
        if ((i + 1) % 10 == 0) std::cout << "\n                    ";
    }
    std::cout << "\n";
    
    std::cout << "\n⚠ WARNING: SAFETY NOTICE ⚠\n";
    std::cout << "• Ensure the robot is in a safe position and ready for testing\n";
    std::cout << "• The motors WILL MOVE during the test\n";
    std::cout << "• Keep hands and objects away from moving parts\n";
    std::cout << "• Press Ctrl+C for emergency stop at any time\n";
    std::cout << "• Ensure adequate ventilation for motor cooling\n";
    
    if (test_joints.size() > 10) {
        std::cout << "• Large number of joints - test may take significant time\n";
        double estimated_time = test_joints.size() * (params.test_duration + 5.0);
        if (!parallel_mode) {
            std::cout << "• Estimated test time: " << std::fixed << std::setprecision(1) 
                      << estimated_time / 60.0 << " minutes\n";
        }
    }
    
    std::cout << "\nDo you want to proceed with the test? (y/N): ";
    
    std::string input;
    std::getline(std::cin, input);
    return (input == "y" || input == "Y" || input == "yes" || input == "YES");
}

// 验证关节ID是否有效
bool isValidJointId(int joint_id) {
    return joint_id >= 1 && joint_id <= 40;
}

// 显示可用关节ID
void printAvailableJointIds() {
    std::cout << "Available joint IDs: 1-40 (32 active joints)\n";
    std::cout << "Joint Groups:\n";
    std::cout << "  Left Arm:   1-8\n";
    std::cout << "  Right Arm:  9-16\n";
    std::cout << "  Left Leg:   17-24\n";
    std::cout << "  Right Leg:  25-32\n";
    std::cout << "  Spare:      33-40\n";
}

int main(int argc, char** argv) {
    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 默认参数
    TestParams params;
    std::vector<int> test_joints;
    bool test_all_joints = false;
    bool interactive_mode = false;
    bool debug_mode = false;
    bool quiet_mode = false;
    bool parallel_mode = false;
    int batch_size = 4;
    std::string raw_data_file;
    
    // 定义长选项
    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"velocity", required_argument, 0, 'v'},
        {"duration", required_argument, 0, 'd'},
        {"sample-rate", required_argument, 0, 's'},
        {"amplitude", required_argument, 0, 'a'},
        {"output", required_argument, 0, 'o'},
        {"motor", required_argument, 0, 'm'},
        {"joints", required_argument, 0, 'j'},
        {"all-joints", no_argument, 0, 'A'},
        {"kp", required_argument, 0, 1001},
        {"kd", required_argument, 0, 1002},
        {"max-torque-ratio", required_argument, 0, 1003},
        {"max-current", required_argument, 0, 1004},
        {"max-temp", required_argument, 0, 1005},
        {"interactive", no_argument, 0, 1006},
        {"debug", no_argument, 0, 1007},
        {"quiet", no_argument, 0, 1008},
        {"save-raw", required_argument, 0, 1009},
        {"parallel", no_argument, 0, 1010},
        {"batch-size", required_argument, 0, 1011},
        {"left-arm", no_argument, 0, 1012},
        {"right-arm", no_argument, 0, 1013},
        {"left-leg", no_argument, 0, 1014},
        {"right-leg", no_argument, 0, 1015},
        {"upper-body", no_argument, 0, 1016},
        {"lower-body", no_argument, 0, 1017},
        {0, 0, 0, 0}
    };
    
    // 解析命令行参数
    int c;
    while ((c = getopt_long(argc, argv, "hv:d:s:a:o:m:j:A", long_options, nullptr)) != -1) {
        switch (c) {
            case 'h':
                printUsage(argv[0]);
                return 0;
                
            case 'v':
                try {
                    params.test_velocity = std::stod(optarg);
                    if (params.test_velocity <= 0 || params.test_velocity > 10.0) {
                        std::cerr << "Error: Velocity must be between 0 and 10.0 rad/s\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid velocity value\n";
                    return 1;
                }
                break;
                
            case 'd':
                try {
                    params.test_duration = std::stod(optarg);
                    if (params.test_duration < 1.0 || params.test_duration > 60.0) {
                        std::cerr << "Error: Duration must be between 1.0 and 60.0 seconds\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid duration value\n";
                    return 1;
                }
                break;
                
            case 's':
                try {
                    params.samples_per_second = std::stoi(optarg);
                    if (params.samples_per_second < 10 || params.samples_per_second > 1000) {
                        std::cerr << "Error: Sample rate must be between 10 and 1000 Hz\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid sample rate value\n";
                    return 1;
                }
                break;
                
            case 'a':
                try {
                    params.position_amplitude = std::stod(optarg);
                    if (params.position_amplitude <= 0 || params.position_amplitude > 6.28) {
                        std::cerr << "Error: Amplitude must be between 0 and 6.28 rad\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid amplitude value\n";
                    return 1;
                }
                break;
                
            case 'o':
                params.output_file = optarg;
                break;
                
            case 'm':
                try {
                    int joint_id = std::stoi(optarg);
                    if (!isValidJointId(joint_id)) {
                        std::cerr << "Error: Invalid joint ID " << joint_id << "\n";
                        printAvailableJointIds();
                        return 1;
                    }
                    test_joints = {joint_id};
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid joint ID\n";
                    return 1;
                }
                break;
                
            case 'j':
                test_joints = parseJointList(optarg);
                if (test_joints.empty()) {
                    std::cerr << "Error: No valid joint IDs in '" << optarg << "'\n";
                    return 1;
                }
                break;
                
            case 'A':
                test_all_joints = true;
                test_joints = ALL_JOINT_IDS;
                break;
                
            case 1001: // --kp
                try {
                    params.kp_test = std::stod(optarg);
                    if (params.kp_test < 0 || params.kp_test > 500.0) {
                        std::cerr << "Error: Kp must be between 0 and 500\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid Kp value\n";
                    return 1;
                }
                break;
                
            case 1002: // --kd
                try {
                    params.kd_test = std::stod(optarg);
                    if (params.kd_test < 0 || params.kd_test > 10.0) {
                        std::cerr << "Error: Kd must be between 0 and 10\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid Kd value\n";
                    return 1;
                }
                break;
                
            case 1003: // --max-torque-ratio
                try {
                    params.max_test_torque_ratio = std::stod(optarg);
                    if (params.max_test_torque_ratio <= 0 || params.max_test_torque_ratio > 0.5) {
                        std::cerr << "Error: Max torque ratio must be between 0 and 0.5\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid max torque ratio value\n";
                    return 1;
                }
                break;
                
            case 1004: // --max-current
                try {
                    params.max_test_current = std::stod(optarg);
                    if (params.max_test_current <= 0 || params.max_test_current > 50.0) {
                        std::cerr << "Error: Max current must be between 0 and 50 A\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid max current value\n";
                    return 1;
                }
                break;
                
            case 1005: // --max-temp
                try {
                    params.max_temperature = std::stod(optarg);
                    if (params.max_temperature < 30.0 || params.max_temperature > 120.0) {
                        std::cerr << "Error: Max temperature must be between 30 and 120 °C\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid max temperature value\n";
                    return 1;
                }
                break;
                
            case 1006: // --interactive
                interactive_mode = true;
                break;
                
            case 1007: // --debug
                debug_mode = true;
                break;
                
            case 1008: // --quiet
                quiet_mode = true;
                break;
                
            case 1009: // --save-raw
                raw_data_file = optarg;
                break;
                
            case 1010: // --parallel
                parallel_mode = true;
                break;
                
            case 1011: // --batch-size
                try {
                    batch_size = std::stoi(optarg);
                    if (batch_size < 1 || batch_size > 16) {
                        std::cerr << "Error: Batch size must be between 1 and 16\n";
                        return 1;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: Invalid batch size value\n";
                    return 1;
                }
                break;
                
            case 1012: // --left-arm
                test_joints = getJointGroup("left-arm");
                break;
                
            case 1013: // --right-arm
                test_joints = getJointGroup("right-arm");
                break;
                
            case 1014: // --left-leg
                test_joints = getJointGroup("left-leg");
                break;
                
            case 1015: // --right-leg
                test_joints = getJointGroup("right-leg");
                break;
                
            case 1016: // --upper-body
                test_joints = getJointGroup("upper-body");
                break;
                
            case 1017: // --lower-body
                test_joints = getJointGroup("lower-body");
                break;
                
            case '?':
                std::cerr << "Error: Unknown option. Use --help for usage information.\n";
                return 1;
                
            default:
                std::cerr << "Error: Unexpected argument parsing result\n";
                return 1;
        }
    }
    
    // 默认测试所有关节
    if (test_joints.empty() && !test_all_joints) {
        test_joints = ALL_JOINT_IDS;
        test_all_joints = true;
    }
    
    // 检查冲突的选项
    if (debug_mode && quiet_mode) {
        std::cerr << "Error: Cannot use --debug and --quiet together\n";
        return 1;
    }
    
    // 设置日志级别
    if (debug_mode) {
        Logger::setLevel(LogLevel::LOG_DEBUG);
    } else if (quiet_mode) {
        Logger::setLevel(LogLevel::LOG_ERROR);
    } else {
        Logger::setLevel(LogLevel::LOG_INFO);
    }
    
    // 交互式模式
    if (interactive_mode) {
        params = interactiveSetup();
    }
    
    // 确认测试配置
    if (!quiet_mode && !confirmTestStart(params, test_joints, parallel_mode)) {
        std::cout << "Test cancelled by user.\n";
        return 0;
    }
    
    try {
        // 创建测试器
        FrictionTester tester;
        g_tester = &tester;
        
        // 设置日志级别
        if (debug_mode) {
            tester.setLogLevel(LogLevel::LOG_DEBUG);
        } else if (quiet_mode) {
            tester.setLogLevel(LogLevel::LOG_ERROR);
        }
        
        // 初始化系统
        std::cout << "\nInitializing friction test system for " << test_joints.size() << " joints...\n";
        
        // 设置调试模式
        if (debug_mode) {
            Logger::setLevel(LogLevel::LOG_DEBUG);
            std::cout << "🐛 DEBUG MODE: Will show detailed test process\n";
        }
        
        if (!tester.initialize()) {
            Logger::error("Failed to initialize friction test system");
            return 1;
        }
        
        // 设置测试参数
        tester.setTestParams(params);
        
        std::vector<MotorFrictionResult> results;
        
        // 执行测试
        if (test_joints.size() == 1) {
            // 单关节测试
            std::cout << "\nStarting single joint test...\n";
            std::cout << "Testing Joint ID " << test_joints[0] << "\n";
            
            // 需要将关节ID转换为电机索引
            int motor_index = -1;
            for (size_t i = 0; i < FrictionTester::getMotorIdList().size(); i++) {
                if (FrictionTester::getMotorIdByIndex(static_cast<int>(i)) == test_joints[0]) {
                    motor_index = static_cast<int>(i);
                    break;
                }
            }
            
            if (motor_index >= 0) {
                MotorFrictionResult result = tester.testSingleMotor(motor_index);
                results.push_back(result);
            } else {
                Logger::error("Joint ID " + std::to_string(test_joints[0]) + " not found in motor list");
                return 1;
            }
        } else if (parallel_mode && test_joints.size() > 1) {
            // 并行测试模式
            std::cout << "\nStarting parallel friction test for " << test_joints.size() << " joints...\n";
            std::cout << "Batch size: " << batch_size << "\n";
            std::cout << "Press Ctrl+C to emergency stop at any time.\n\n";
            
            // 分批并行测试
            for (size_t i = 0; i < test_joints.size(); i += batch_size) {
                size_t end_idx = std::min(i + batch_size, test_joints.size());
                std::vector<int> batch_joints(test_joints.begin() + i, test_joints.begin() + end_idx);
                
                std::cout << "Testing batch " << (i / batch_size + 1) << " (joints ";
                for (size_t j = 0; j < batch_joints.size(); j++) {
                    std::cout << batch_joints[j];
                    if (j < batch_joints.size() - 1) std::cout << ", ";
                }
                std::cout << ")...\n";
                
                // 转换为电机索引
                std::vector<int> motor_indices;
                for (int joint_id : batch_joints) {
                    for (size_t k = 0; k < FrictionTester::getMotorIdList().size(); k++) {
                        if (FrictionTester::getMotorIdByIndex(static_cast<int>(k)) == joint_id) {
                            motor_indices.push_back(static_cast<int>(k));
                            break;
                        }
                    }
                }
                
                // 执行并行测试 (需要实现并行测试功能)
                auto batch_results = tester.testMotorsBatch(motor_indices);
                results.insert(results.end(), batch_results.begin(), batch_results.end());
                
                // 批次间休息
                if (i + batch_size < test_joints.size()) {
                    std::cout << "Batch completed. Cooling down for 30 seconds...\n";
                    std::this_thread::sleep_for(std::chrono::seconds(30));
                }
            }
        } else {
            // 顺序测试所有关节
            std::cout << "\nStarting sequential friction test for " << test_joints.size() << " joints...\n";
            std::cout << "Press Ctrl+C to emergency stop at any time.\n\n";
            
            // 显示进度信息
            auto start_time = std::chrono::steady_clock::now();
            
            for (size_t i = 0; i < test_joints.size(); i++) {
                int joint_id = test_joints[i];
                
                std::cout << "\n[" << (i + 1) << "/" << test_joints.size() << "] ";
                std::cout << "Testing Joint ID " << joint_id << "...\n";
                
                // 转换为电机索引
                int motor_index = -1;
                for (size_t j = 0; j < FrictionTester::getMotorIdList().size(); j++) {
                    if (FrictionTester::getMotorIdByIndex(static_cast<int>(j)) == joint_id) {
                        motor_index = static_cast<int>(j);
                        break;
                    }
                }
                
                if (motor_index >= 0) {
                    MotorFrictionResult result = tester.testSingleMotor(motor_index);
                    results.push_back(result);
                    
                    // 显示简要结果
                    if (result.test_passed) {
                        std::cout << "✅ Joint " << joint_id << " PASSED"
                                  << " (Static: " << std::fixed << std::setprecision(2) << result.static_friction << " Nm"
                                  << ", Kinetic: " << result.kinetic_friction << " Nm)\n";
                    } else {
                        std::cout << "❌ Joint " << joint_id << " FAILED: " << result.error_message << "\n";
                    }
                    
                    // 显示进度和预估时间
                    if (i < test_joints.size() - 1) {
                        auto current_time = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
                        double avg_time_per_joint = static_cast<double>(elapsed.count()) / (i + 1);
                        double estimated_remaining = avg_time_per_joint * (test_joints.size() - i - 1);
                        
                        std::cout << "Progress: " << std::fixed << std::setprecision(1) 
                                  << (100.0 * (i + 1) / test_joints.size()) << "%, "
                                  << "Estimated remaining: " << static_cast<int>(estimated_remaining / 60) 
                                  << "m " << static_cast<int>(estimated_remaining) % 60 << "s\n";
                        
                        // 关节间冷却时间
                        if (params.test_duration > 5.0) {
                            std::cout << "Cooling down for 10 seconds...\n";
                            std::this_thread::sleep_for(std::chrono::seconds(10));
                        }
                    }
                } else {
                    Logger::error("Joint ID " + std::to_string(joint_id) + " not found in motor list");
                    // 创建失败结果
                    MotorFrictionResult failed_result;
                    failed_result.motor_id = joint_id;
                    failed_result.test_passed = false;
                    failed_result.error_message = "Joint not found in motor list";
                    results.push_back(failed_result);
                }
                
                // 检查紧急停止
                if (g_shutdown_requested) {
                    std::cout << "\nEmergency stop requested. Aborting test...\n";
                    break;
                }
            }
        }
        
        // 保存原始数据
        if (!raw_data_file.empty()) {
            std::cout << "\nSaving raw test data...\n";
            if (tester.saveRawData(raw_data_file)) {
                std::cout << "Raw data saved to: " << raw_data_file << "\n";
            } else {
                Logger::warn("Failed to save raw data");
            }
        }
        
        // 生成报告
        std::cout << "\nGenerating test report...\n";
        if (tester.generateReport(results)) {
            std::cout << "Test completed successfully!\n";
            std::cout << "Report saved to: " << params.output_file << "\n";
            
            // 显示详细结果统计
            int passed = 0, failed = 0;
            double total_time = 0.0;
            double avg_static = 0.0, avg_kinetic = 0.0;
            double min_static = 1000.0, max_static = 0.0;
            double min_kinetic = 1000.0, max_kinetic = 0.0;
            
            for (const auto& result : results) {
                if (result.test_passed) {
                    passed++;
                    avg_static += result.static_friction;
                    avg_kinetic += result.kinetic_friction;
                    min_static = std::min(min_static, result.static_friction);
                    max_static = std::max(max_static, result.static_friction);
                    min_kinetic = std::min(min_kinetic, result.kinetic_friction);
                    max_kinetic = std::max(max_kinetic, result.kinetic_friction);
                } else {
                    failed++;
                }
                total_time += result.test_duration;
            }
            
            if (passed > 0) {
                avg_static /= passed;
                avg_kinetic /= passed;
            }
            
            std::cout << "\n╔═══════════ TEST SUMMARY ════════════╗\n";
            std::cout << "║ Total Joints:     " << std::setw(6) << results.size() << "            ║\n";
            std::cout << "║ Passed:           " << std::setw(6) << passed << "            ║\n";
            std::cout << "║ Failed:           " << std::setw(6) << failed << "            ║\n";
            std::cout << "║ Success Rate:     " << std::setw(5) << std::fixed << std::setprecision(1) 
                      << (results.empty() ? 0.0 : passed * 100.0 / results.size()) << "%           ║\n";
            std::cout << "║ Total Time:       " << std::setw(5) << std::fixed << std::setprecision(1) 
                      << total_time / 60.0 << " min        ║\n";
            
            if (passed > 0) {
                std::cout << "║                                     ║\n";
                std::cout << "║ Friction Statistics (Passed):      ║\n";
                std::cout << "║ Avg Static:       " << std::setw(5) << std::fixed << std::setprecision(2) 
                          << avg_static << " Nm        ║\n";
                std::cout << "║ Avg Kinetic:      " << std::setw(5) << std::fixed << std::setprecision(2) 
                          << avg_kinetic << " Nm        ║\n";
                std::cout << "║ Static Range:     " << std::setw(4) << std::fixed << std::setprecision(2) 
                          << min_static << "-" << std::setw(4) << max_static << " Nm   ║\n";
                std::cout << "║ Kinetic Range:    " << std::setw(4) << std::fixed << std::setprecision(2) 
                          << min_kinetic << "-" << std::setw(4) << max_kinetic << " Nm   ║\n";
            }
            
            std::cout << "╚═════════════════════════════════════╝\n";
            
            if (failed > 0) {
                std::cout << "\n⚠ Failed Joints:\n";
                for (const auto& result : results) {
                    if (!result.test_passed) {
                        std::cout << "  • Joint ID " << result.motor_id 
                                  << ": " << result.error_message << "\n";
                    }
                }
                
                std::cout << "\n📋 Recommendations:\n";
                double fail_rate = failed * 100.0 / results.size();
                if (fail_rate < 10.0) {
                    std::cout << "  • Low failure rate - check failed joints individually\n";
                    std::cout << "  • Consider adjusting test parameters for specific joint types\n";
                } else if (fail_rate < 25.0) {
                    std::cout << "  • Moderate failure rate - review assembly and calibration\n";
                    std::cout << "  • Check joint lubrication and mechanical alignment\n";
                } else {
                    std::cout << "  • High failure rate - major system issues suspected\n";
                    std::cout << "  • Do not deploy robot until issues are resolved\n";
                    std::cout << "  • Contact technical support for assistance\n";
                }
            } else {
                std::cout << "\n✅ All " << results.size() << " joints passed friction testing!\n";
                std::cout << "   Robot is ready for deployment.\n";
                
                // 额外的质量评估
                if (passed >= 30) {
                    std::cout << "\n🏆 EXCELLENT: Full 32-joint system tested successfully!\n";
                } else if (passed >= 20) {
                    std::cout << "\n👍 GOOD: Major joint groups tested successfully.\n";
                } else {
                    std::cout << "\n🔍 PARTIAL: Limited joint testing completed.\n";
                }
            }
            
        } else {
            Logger::error("Failed to generate test report");
            return 1;
        }
        
    } catch (const std::exception& e) {
        Logger::error("Exception occurred: " + std::string(e.what()));
        if (g_tester) {
            g_tester->emergencyStop();
        }
        return 1;
    } catch (...) {
        Logger::error("Unknown exception occurred");
        if (g_tester) {
            g_tester->emergencyStop();
        }
        return 1;
    }
    
    std::cout << "\nFriction test program completed.\n";
    std::cout << "Thank you for using BridgeDP 32-Joint Friction Test System!\n";
    
    return 0;
}
