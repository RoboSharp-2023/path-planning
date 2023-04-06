#include <memory>
#include <array>
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <SerialStream.h>

using namespace LibSerial;
using namespace std::chrono_literals;

class serial_node: public rclcpp::Node{
    SerialStream serial;
    rclcpp::TimerBase::SharedPtr time_interrupt;

public:
    serial_node(): Node("serial_connect"){
        this->time_interrupt = create_wall_timer(500ms, std::bind(&serial_node::timer_callback, this));

        auto param_serial_name = rcl_interfaces::msg::ParameterDescriptor{};
        param_serial_name.description = "";
        this->declare_parameter("serial_name", "", param_serial_name);

        std::string port_name = get_parameter("serial_name").as_string();

        serial.Open(port_name);

        serial.SetBaudRate(BaudRate::BAUD_115200);
        serial.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial.SetFlowControl(FlowControl::FLOW_CONTROL_DEFAULT);
        serial.SetParity(Parity::PARITY_NONE);
        serial.SetStopBits(StopBits::STOP_BITS_1);
    }

    ~serial_node(){
        serial.Close();
    }

    void timer_callback() {
        serial << "test" << std::endl;
    }
};
