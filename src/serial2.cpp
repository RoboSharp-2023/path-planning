#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <chrono>

using namespace std::chrono_literals;

class UARTNode : public rclcpp::Node {
public:
	UARTNode() : Node("uart_node") {
		this->tim = create_wall_timer(200ms, std::bind(&UARTNode::timer_callback, this));
		this->recv_tim = create_wall_timer(100ms, std::bind(&UARTNode::recv_timer_callback, this));

		// シリアルポートのデバイスファイルを設定
		auto param_serial_name = rcl_interfaces::msg::ParameterDescriptor{};
		param_serial_name.description = "";
		this->declare_parameter("serial_name", "", param_serial_name);

		auto device = get_parameter("serial_name").as_string();

		RCLCPP_INFO(this->get_logger(), "%s", device.c_str());

		// シリアルポートをオープン
		fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
		if (fd_ < 0) {
			RCLCPP_ERROR(this->get_logger(), "シリアルポートを開けませんでした");
			return;
		}

		// シリアル通信の設定
		struct termios serial;
		tcgetattr(fd_, &serial);
		cfsetospeed(&serial, B115200);  // ボーレートを9600bpsに設定
		serial.c_cflag &= ~CSIZE;
		serial.c_cflag |= CS8;      // データビットを8bitに設定
		serial.c_cflag &= ~PARENB; // パリティビットを無効に設定
		serial.c_cflag &= ~CSTOPB; // ストップビットを1bitに設定
		tcsetattr(fd_, TCSANOW, &serial);

	}

	~UARTNode()	{
		// シリアルポートをクローズ
		if (fd_ >= 0)
		{
			close(fd_);
		}
	}

	void send(const std::string &data) {
		RCLCPP_INFO(this->get_logger(), "send");
		write(fd_, data.c_str(), data.length());
	}

	void timer_callback() {
		send("a");
	}

	void recv_timer_callback() {
		char buff[256];

		RCLCPP_INFO(this->get_logger(), "read head");
		int n = read(fd_, buff, 256);
		RCLCPP_INFO(this->get_logger(), "read end");
		if(n > 0) {
			std::string recv_data(buff, n);
			RCLCPP_INFO(this->get_logger(), "recv %s", recv_data.c_str());
		}
	}

private:


	int fd_;                       // シリアルポートのファイルディスクリプタ
	std::thread receive_thread_;   // 受信
	rclcpp::TimerBase::SharedPtr tim;
	rclcpp::TimerBase::SharedPtr recv_tim;
};

int main(int argc, char *argv[]) {
	// ROS2ノードの初期化
	rclcpp::init(argc, argv);
	auto node = std::make_shared<UARTNode>();
	// データの送信例

	// ROS2スピン
	rclcpp::spin(node);

	// ROS2ノードのシャットダウン
	rclcpp::shutdown();

	return 0;
}