/**
 * @file test_steering.cpp
 * @brief 独立转向舵机测试程序
 *
 * 直接通过串口控制底盘转向舵机，不依赖 ROS2 节点。
 *
 * 编译:
 *   g++ -o test_steering test_steering.cpp -lserial -std=c++17
 *   (需要先安装 serial 库: sudo apt install libserial-dev 或从源码编译)
 *
 * 或使用 ROS2 的 serial 包:
 *   cd dev_ws
 *   source /opt/tros/humble/setup.bash
 *   source install/setup.bash
 *   g++ -o test_steering src/racing/racing_control_2026/scripts/test_steering.cpp \
 *       -I install/serial/include -L install/serial/lib -lserial -std=c++17 \
 *       -Wl,-rpath,$(pwd)/install/serial/lib
 *
 * 用法:
 *   ./test_steering                     # 交互模式
 *   ./test_steering --sweep             # 自动扫描（左→中→右→中）
 *   ./test_steering --angle 0.5         # 设置固定转向角（弧度）
 *   ./test_steering --port /dev/ttyACM0 # 指定串口
 */

#include <iostream>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>
#include <unistd.h>

#include <serial/serial.h>

// ========== 协议常量（与 origincar_base 一致）==========
#define FRAME_HEADER    0x7B
#define FRAME_TAIL      0x7D
#define SEND_DATA_SIZE  11
#define SERIAL_PORT     "/dev/ttyACM0"
#define BAUD_RATE       921600

static volatile bool g_running = true;

void sig_handler(int sig) {
    (void)sig;
    g_running = false;
    std::cout << "\n正在退出..." << std::endl;
}

// ========== 串口通信 ==========

class SteeringTester {
public:
    SteeringTester(const std::string& port = SERIAL_PORT, int baud = BAUD_RATE)
        : port_(port), baud_(baud) {}

    bool open() {
        try {
            serial_.setPort(port_);
            serial_.setBaudrate(baud_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
            serial_.setTimeout(timeout);
            serial_.open();
        } catch (serial::IOException& e) {
            std::cerr << "无法打开串口 " << port_ << ": " << e.what() << std::endl;
            return false;
        }
        if (serial_.isOpen()) {
            std::cout << "串口已打开: " << port_ << " @ " << baud_ << " bps" << std::endl;
            return true;
        }
        return false;
    }

    void close() {
        if (serial_.isOpen()) {
            // 发送零速 + 零转向 确保安全
            send_command(0.0, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            serial_.close();
            std::cout << "串口已关闭" << std::endl;
        }
    }

    /**
     * @brief 发送速度和转向命令
     * @param speed 速度 (m/s), 0 = 停车
     * @param steering 转向角 (rad), 正 = 左转, 负 = 右转
     *
     * 协议帧格式 (11 bytes):
     *   [0]  0x7B          帧头
     *   [1]  0x00          保留
     *   [2]  0x00          保留
     *   [3-4] speed*1000   速度 (int16, 大端)
     *   [5-6] 0x0000       Y速度 (差分底盘用, 阿克曼设为0)
     *   [7-8] steer*1000/2 转向角 (int16, 大端, 除以2是协议约定)
     *   [9]  checksum      XOR校验
     *   [10] 0x7D          帧尾
     */
    void send_command(double speed, double steering) {
        uint8_t tx[SEND_DATA_SIZE];
        short transition;

        tx[0] = FRAME_HEADER;
        tx[1] = 0;
        tx[2] = 0;

        // 速度
        transition = static_cast<short>(speed * 1000);
        tx[3] = static_cast<uint8_t>(transition >> 8);
        tx[4] = static_cast<uint8_t>(transition);

        // Y速度 (阿克曼底盘不用)
        tx[5] = 0;
        tx[6] = 0;

        // 转向角 (协议约定除以2)
        transition = static_cast<short>(steering * 1000 / 2);
        tx[7] = static_cast<uint8_t>(transition >> 8);
        tx[8] = static_cast<uint8_t>(transition);

        // XOR 校验
        uint8_t checksum = 0;
        for (int k = 0; k < 9; k++) {
            checksum ^= tx[k];
        }
        tx[9] = checksum;
        tx[10] = FRAME_TAIL;

        try {
            serial_.write(tx, sizeof(tx));
        } catch (serial::IOException& e) {
            std::cerr << "串口写入失败: " << e.what() << std::endl;
        }
    }

private:
    std::string port_;
    int baud_;
    serial::Serial serial_;
};

// ========== 测试模式 ==========

void mode_interactive(SteeringTester& tester) {
    std::cout << "\n===== 交互模式 =====" << std::endl;
    std::cout << "命令:" << std::endl;
    std::cout << "  l <角度>  左转 (rad), 例: l 0.5" << std::endl;
    std::cout << "  r <角度>  右转 (rad), 例: r 0.5" << std::endl;
    std::cout << "  c         回中 (转向归零)" << std::endl;
    std::cout << "  s <速度>  前进速度 (m/s), 例: s 0.3" << std::endl;
    std::cout << "  p         停车 (速度归零)" << std::endl;
    std::cout << "  q         退出" << std::endl;
    std::cout << "====================\n" << std::endl;

    double current_speed = 0.0;
    double current_steer = 0.0;

    while (g_running) {
        std::cout << "[speed=" << current_speed << " steer=" << current_steer << "] > ";
        std::string line;
        if (!std::getline(std::cin, line)) break;
        if (line.empty()) continue;

        char cmd = line[0];
        double val = 0.0;
        if (line.size() > 1) {
            try { val = std::stod(line.substr(1)); } catch (...) { val = 0.0; }
        }

        switch (cmd) {
            case 'l': case 'L':
                current_steer = std::abs(val);
                tester.send_command(current_speed, current_steer);
                std::cout << "  左转 " << current_steer << " rad (" 
                         << current_steer * 180.0 / M_PI << "°)" << std::endl;
                break;
            case 'r': case 'R':
                current_steer = -std::abs(val);
                tester.send_command(current_speed, current_steer);
                std::cout << "  右转 " << current_steer << " rad ("
                         << current_steer * 180.0 / M_PI << "°)" << std::endl;
                break;
            case 'c': case 'C':
                current_steer = 0.0;
                tester.send_command(current_speed, 0.0);
                std::cout << "  回中" << std::endl;
                break;
            case 's': case 'S':
                current_speed = val;
                tester.send_command(current_speed, current_steer);
                std::cout << "  速度 " << current_speed << " m/s" << std::endl;
                break;
            case 'p': case 'P':
                current_speed = 0.0;
                tester.send_command(0.0, current_steer);
                std::cout << "  停车" << std::endl;
                break;
            case 'q': case 'Q':
                g_running = false;
                break;
            default:
                std::cout << "  未知命令: " << cmd << std::endl;
                break;
        }
    }

    // 安全退出
    tester.send_command(0.0, 0.0);
}

void mode_sweep(SteeringTester& tester) {
    std::cout << "\n===== 扫描模式 =====" << std::endl;
    std::cout << "舵机将从中位 → 左极限 → 中位 → 右极限 → 中位 循环" << std::endl;
    std::cout << "Ctrl+C 退出\n" << std::endl;

    // 扫描参数（可根据实际舵机范围调整）
    const double max_angle = 0.6;   // 最大转向角 (rad, ~34°)
    const double step = 0.05;       // 每步增量 (rad)
    const int hold_ms = 200;        // 每步保持时间 (ms)

    int cycle = 0;
    while (g_running) {
        cycle++;
        std::cout << "--- 第 " << cycle << " 轮 ---" << std::endl;

        // 中 → 左
        std::cout << "中位 → 左极限..." << std::endl;
        for (double a = 0; a <= max_angle && g_running; a += step) {
            tester.send_command(0.0, a);
            std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
            std::cout << "  " << a << " rad (" << a * 180.0 / M_PI << "°)\r" << std::flush;
        }
        std::cout << std::endl;

        // 左 → 中
        std::cout << "左极限 → 中位..." << std::endl;
        for (double a = max_angle; a >= 0 && g_running; a -= step) {
            tester.send_command(0.0, a);
            std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
            std::cout << "  " << a << " rad (" << a * 180.0 / M_PI << "°)\r" << std::flush;
        }
        std::cout << std::endl;

        // 中 → 右
        std::cout << "中位 → 右极限..." << std::endl;
        for (double a = 0; a >= -max_angle && g_running; a -= step) {
            tester.send_command(0.0, a);
            std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
            std::cout << "  " << a << " rad (" << a * 180.0 / M_PI << "°)\r" << std::flush;
        }
        std::cout << std::endl;

        // 右 → 中
        std::cout << "右极限 → 中位..." << std::endl;
        for (double a = -max_angle; a <= 0 && g_running; a += step) {
            tester.send_command(0.0, a);
            std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
            std::cout << "  " << a << " rad (" << a * 180.0 / M_PI << "°)\r" << std::flush;
        }
        std::cout << std::endl;
    }

    tester.send_command(0.0, 0.0);
    std::cout << "\n扫描结束，舵机已回中" << std::endl;
}

void mode_fixed_angle(SteeringTester& tester, double angle) {
    std::cout << "\n===== 固定角度模式 =====" << std::endl;
    std::cout << "设置转向角: " << angle << " rad (" << angle * 180.0 / M_PI << "°)" << std::endl;
    std::cout << "Ctrl+C 退出\n" << std::endl;

    tester.send_command(0.0, angle);

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    tester.send_command(0.0, 0.0);
    std::cout << "已回中" << std::endl;
}

// ========== 帮助 ==========

void print_usage(const char* prog) {
    std::cout << "用法: " << prog << " [选项]\n" << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  (无参数)           交互模式（手动输入命令）" << std::endl;
    std::cout << "  --sweep            自动扫描模式（左→中→右→中循环）" << std::endl;
    std::cout << "  --angle <rad>      固定转向角模式（弧度, 正=左转）" << std::endl;
    std::cout << "  --port <设备>      串口设备 (默认: " << SERIAL_PORT << ")" << std::endl;
    std::cout << "  --baud <波特率>    波特率 (默认: " << BAUD_RATE << ")" << std::endl;
    std::cout << "  --help             显示此帮助" << std::endl;
    std::cout << "\n示例:" << std::endl;
    std::cout << "  " << prog << "                        # 交互模式" << std::endl;
    std::cout << "  " << prog << " --sweep                # 自动扫描" << std::endl;
    std::cout << "  " << prog << " --angle 0.3            # 左转 0.3 rad" << std::endl;
    std::cout << "  " << prog << " --angle -0.4           # 右转 0.4 rad" << std::endl;
    std::cout << "  " << prog << " --port /dev/ttyUSB0    # 指定串口" << std::endl;
}

// ========== Main ==========

int main(int argc, char* argv[]) {
    signal(SIGINT, sig_handler);

    std::string port = SERIAL_PORT;
    int baud = BAUD_RATE;
    std::string mode = "interactive";
    double angle = 0.0;

    // 解析命令行参数
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--sweep") {
            mode = "sweep";
        } else if (arg == "--angle" && i + 1 < argc) {
            mode = "fixed";
            angle = std::stod(argv[++i]);
        } else if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            baud = std::stoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "未知参数: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  转向舵机测试程序" << std::endl;
    std::cout << "  串口: " << port << " @ " << baud << " bps" << std::endl;
    std::cout << "  模式: " << mode << std::endl;
    std::cout << "========================================" << std::endl;

    SteeringTester tester(port, baud);
    if (!tester.open()) {
        return 1;
    }

    if (mode == "sweep") {
        mode_sweep(tester);
    } else if (mode == "fixed") {
        mode_fixed_angle(tester, angle);
    } else {
        mode_interactive(tester);
    }

    tester.close();
    return 0;
}
