# Robot Friction Test System

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)](https://github.com/your-username/robot-friction-test)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Linux-lightgrey.svg)](https://www.linux.org/)
[![Language](https://img.shields.io/badge/language-C++-00599C.svg)](https://isocpp.org/)

一个专为32关节机器人设计的摩擦力测试系统，基于CAN总线通信和PT协议，用于检测关节摩擦力特性，确保机器人部署前的质量控制。

## 🎯 主要特性

- **32关节全覆盖** - 支持1-40个关节ID，覆盖完整机器人系统
- **PT协议测试** - 基于电机端实际代码实现的正确PT模式协议
- **灵活测试模式** - 支持单关节、关节组、批量和全部关节测试
- **实时监控** - 提供详细的测试进度、温度和电流监控
- **安全保护** - 内置安全限制和紧急停止功能
- **数据记录** - 自动生成详细测试报告和原始数据
- **并行测试** - 支持多关节并行测试以提高效率

## 📋 系统要求

### 硬件要求
- CAN-USB转换器 (支持USBCAN2协议)
- 32关节机器人系统
- Linux操作系统 (Ubuntu 18.04+推荐)

### 软件依赖
```bash
# 基础编译工具
sudo apt-get install build-essential cmake

# CAN通信库
sudo apt-get install libcan-dev

# 线程支持
sudo apt-get install libpthread-stubs0-dev
```

## 🚀 快速开始

### 1. 克隆仓库
```bash
git clone https://github.com/your-username/robot-friction-test.git
cd robot-friction-test
```

### 2. 编译程序
```bash
# 编译PT协议测试程序
g++ -o correct_pt_test correct_pt_friction_test.cpp -lcontrolcan -lpthread

# 或者使用Makefile
make all
```

### 3. 运行测试
```bash
# 测试所有32个关节
./correct_pt_test --all-joints

# 测试单个关节
./correct_pt_test -m 1

# 交互模式
./correct_pt_test
```

## 📖 使用指南

### 基本命令

| 命令 | 说明 | 示例 |
|------|------|------|
| `--all-joints` | 测试所有关节 | `./correct_pt_test --all-joints` |
| `-m, --motor ID` | 测试单个关节 | `./correct_pt_test -m 1` |
| `-j, --joints LIST` | 测试指定关节 | `./correct_pt_test -j "1,2,3"` |
| `--left-arm` | 测试左臂关节 | `./correct_pt_test --left-arm` |
| `--debug` | 启用调试模式 | `./correct_pt_test --debug` |

### 关节组定义

| 关节组 | 关节ID范围 | 描述 |
|--------|------------|------|
| 左臂 | 1-8 | 左臂7个关节 + 左手 |
| 右臂 | 9-16 | 右臂7个关节 + 右手 |
| 左腿 | 17-24 | 左腿6个关节 + 左脚 |
| 右腿 | 25-32 | 右腿6个关节 + 右脚 |
| 上半身 | 1-16 | 双臂关节 |
| 下半身 | 17-32 | 双腿关节 |

### 高级用法

```bash
# 并行测试提高效率
./correct_pt_test --all-joints --parallel --batch-size 8

# 自定义测试参数
./correct_pt_test -j "1-8" --max-torque 2.0 --torque-step 0.05

# 保存原始数据
./correct_pt_test --left-arm --save-raw left_arm_data.csv

# 静默模式批量测试
./correct_pt_test --all-joints --quiet --output batch_results.txt
```

## 📊 测试结果

### 输出文件
- **测试报告** - `pt_friction_results.txt` (默认)
- **原始数据** - 可选的CSV格式详细数据
- **日志文件** - 测试过程的详细日志

### 结果解读
```
╔═══ 测试摘要 ═══╗
║ 总关节数:    32 ║
║ 通过:        30 ║
║ 失败:         2 ║
║ 成功率:    93.8% ║
║ 总时间:    45.2m ║
╚════════════════╝
```

**质量评估标准：**
- 🏆 **优秀** (95%+) - 可直接部署
- 👍 **良好** (85-95%) - 轻微调整后部署
- ⚠️ **需关注** (70-85%) - 需要检查和维护
- ❌ **不合格** (<70%) - 需要重大维修

## ⚙️ 配置选项

### 电机型号支持
| 型号 | 扭矩范围 (NM) | 适用关节 |
|------|---------------|----------|
| 30-40 | -30 ~ 30 | 小型关节 |
| 100-120 | -188 ~ 188 | 大型关节 |
| 70-90 | -64 ~ 64 | 中型关节 |

### 测试参数
```bash
--max-torque VALUE     # 最大测试扭矩 (默认: 4.0 NM)
--torque-step VALUE    # 扭矩步进 (默认: 0.1 NM)
--threshold VALUE      # 位置检测阈值 (默认: 0.02 rad)
--wait-time VALUE      # 稳定等待时间 (默认: 500 ms)
```

## 🔧 故障排除

### 常见问题

**1. CAN设备连接失败**
```bash
# 检查设备权限
sudo chmod 666 /dev/ttyUSB*

# 检查CAN驱动
lsmod | grep can
```

**2. 编译错误**
```bash
# 检查库文件
ls -la lib/libcontrolcan.so

# 添加库路径
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
```

**3. 关节无响应**
- 检查关节ID配置
- 验证CAN总线连接
- 确认电源供应
- 检查关节是否处于使能状态

**4. 测试数据异常**
- 确认关节可自由移动
- 检查周围是否有障碍物
- 验证关节标定状态
- 调整测试参数阈值

## 🛡️ 安全须知

⚠️ **重要安全提醒**

1. **测试前准备**
   - 确保机器人处于安全位置
   - 清除关节周围障碍物
   - 检查紧急停止按钮功能
   - 确保有足够的活动空间

2. **测试过程中**
   - 禁止手动触碰移动关节
   - 保持安全距离
   - 随时准备按下Ctrl+C紧急停止
   - 监控电机温度，避免过热

3. **异常处理**
   - 立即按下紧急停止
   - 检查关节状态
   - 记录异常现象
   - 联系技术支持

## 📈 开发计划

### 当前版本 (v2.0)
- ✅ 32关节全覆盖测试
- ✅ PT协议完整实现
- ✅ 并行测试支持
- ✅ 详细结果报告

### 计划功能 (v2.1)
- 🔄 Web界面支持
- 🔄 实时数据可视化
- 🔄 云端数据同步
- 🔄 AI异常检测

### 未来规划 (v3.0)
- 📋 多机器人管理
- 📋 历史数据分析
- 📋 预测性维护
- 📋 移动端监控

## 🤝 贡献指南

我们欢迎社区贡献！请遵循以下步骤：

1. Fork本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建Pull Request

### 开发环境设置
```bash
# 开发依赖
sudo apt-get install valgrind gdb

# 代码格式化
sudo apt-get install clang-format

# 运行测试
make test
```

## 📄 许可证

本项目采用MIT许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 📞 支持与联系

- **技术支持**: [support@bridgedp.com](mailto:support@bridgedp.com)
- **问题反馈**: [GitHub Issues](https://github.com/your-username/robot-friction-test/issues)
- **功能请求**: [GitHub Discussions](https://github.com/your-username/robot-friction-test/discussions)
- **文档Wiki**: [Project Wiki](https://github.com/your-username/robot-friction-test/wiki)

## 🙏 致谢

- 感谢BridgeDP机器人团队的技术支持
- 感谢开源社区提供的CAN通信库
- 感谢所有测试人员和反馈用户

---

<div align="center">

**[⬆ 回到顶部](#robot-friction-test-system)**

Made with ❤️ by BridgeDP Robotics Team

</div>
