#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <termios.h>
#include <sys/epoll.h>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

using namespace std;

const string UART_DEVICE_NAME("/dev/ttyACM0");
const auto UART_BAUDRATE = B115200;

class uart_node: public rclcpp::Node {
	int uart_fd;
	int orig_modem_status;

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

	void open_uart() {
		uart_fd = open(UART_DEVICE_NAME.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
		if(uart_fd < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to open uart device");
			exit(1);
		}

		struct termios term;
		tcgetattr(uart_fd, &term);
		cfsetospeed(&term, UART_BAUDRATE);
		cfsetispeed(&term, UART_BAUDRATE);
		term.c_cflag &= ~PARENB; // パリティビットなし
		term.c_cflag &= ~CSTOPB; // ストップビット1
		term.c_cflag &= ~CSIZE;
		term.c_cflag |= CS8; // データビット8
		term.c_cflag &= ~CRTSCTS; // ハードウェアフロー制御なし
		term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		term.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
		term.c_oflag &= ~OPOST;
		tcsetattr(uart_fd, TCSANOW, &term);

		tcsetattr(uart_fd, TCSANOW, &term);

		int flags = fcntl(uart_fd, F_GETFL, 0);
		fcntl(uart_fd, F_SETFL, flags | O_NONBLOCK);

		int request_iqr_request = ioctl(uart_fd, TIOCMGET, &orig_modem_status);
		if (request_iqr_request < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to get modem status");
			exit(1);
		}
		
		int set_irq_request = ioctl(uart_fd, TIOCMIWAIT, TIOCM_CTS | TIOCM_DSR | TIOCM_RI);
		if (set_irq_request < 0) {
			RCLCPP_ERROR(this->get_logger(), "set_iqr_request_val:%d", set_irq_request);
			RCLCPP_ERROR(this->get_logger(), "Failed to set modem status");
			exit(1);
		}

			int epoll_fd = epoll_create1(0);
		if (epoll_fd < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to create epoll fd");
			exit(1);
		}

		struct epoll_event event;
		memset(&event, 0, sizeof(event));
		event.data.fd = uart_fd;
		event.events = EPOLLIN;
		if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, uart_fd, &event) < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to add uart_fd to epoll");
			exit(1);
		}

		struct epoll_event events[10];
		while (rclcpp::ok()) {
			int num_events = epoll_wait(epoll_fd, events, 10, -1);
			if (num_events < 0) {
				RCLCPP_ERROR(this->get_logger(), "Failed to wait for UART event");
				exit(1);
			}

			for (int i = 0; i < num_events; i++) {
				if (events[i].events & EPOLLPRI) {
					constexpr int buff_size = 256;
					char read_data[buff_size];
					int byte_read = read(uart_fd, read_data, buff_size);
					if (byte_read < 0) {
						RCLCPP_ERROR(this->get_logger(), "Failed to read from UART");
						exit(1);
					}

					string msg_str;
					for(int i = 0; i < byte_read; i++) {
						msg_str.push_back(read_data[i]);
					}

					RCLCPP_INFO(this->get_logger(), "msg: %s", msg_str.c_str());
					
					std_msgs::msg::String message;
					message.data = msg_str;
					pub->publish(message);
				}
			}
		}
		RCLCPP_INFO(this->get_logger(), "UART device opened successfully!");
	}
public:
	uart_node() :Node("uart_node") {
		pub = create_publisher<std_msgs::msg::String>("uart", 10);
		open_uart();
	}
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<uart_node>());
	rclcpp::shutdown();
	return 0;
}
