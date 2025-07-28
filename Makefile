
# Makefile for CAN test program on Linux

# 编译器
CXX = g++

# 编译选项
CXXFLAGS = -std=c++11 -Wall -O2

# 目标文件名
TARGET = can_test

# 源文件
SOURCES = can_test.cpp

# 库文件路径 (根据你的实际情况修改)
# 如果你有 .so 文件，请修改路径
LIBPATH = ./

# 库文件 (根据你的CAN库文件名修改)
# 常见的可能是 libcontrolcan.so 或者 libcanapi.so
LIBS = -lcontrolcan

# 头文件路径
INCLUDES = -I./

# 默认目标
all: $(TARGET)

# 编译目标
$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -L$(LIBPATH) -o $(TARGET) $(SOURCES) $(LIBS)

# 清理
clean:
	rm -f $(TARGET)

# 安装依赖 (如果需要)
install:
	@echo "请确保以下文件存在："
	@echo "1. controlcan.h - CAN接口头文件"
	@echo "2. libcontrolcan.so - CAN接口库文件"
	@echo "3. USB-CAN设备驱动已安装"

# 帮助
help:
	@echo "可用目标："
	@echo "  all     - 编译程序"
	@echo "  clean   - 清理编译文件"
	@echo "  install - 显示安装说明"
	@echo "  help    - 显示此帮助"

.PHONY: all clean install help
