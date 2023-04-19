#include <memory>
#include <array>
#include <vector>
#include <utility>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "include/RRT_star.hpp"
#include "path_planning/msg/path_data.hpp"

using namespace std;
using namespace std::chrono_literals;

class map_server : public rclcpp::Node {
	rclcpp::Publisher<path_planning::msg::PathData>::SharedPtr p;
	rclcpp::TimerBase::SharedPtr t;

	shared_ptr<node_t> goal(shared_ptr<node_t> root) {
		if (root == nullptr) {
			return nullptr;
		}

		if (root->is_goal) {
			return root;
		}		

		for (auto& child: root->children) {
			auto result = goal(child);
			if (result != nullptr) {
				return result;
			}
		}

		return nullptr;
	}

public:
	map_server() : Node("map_server") {
		this->t = create_wall_timer(500ms, bind(&map_server::timer_callback, this));
		this->p = create_publisher<path_planning::msg::PathData>("path_", 10);
	}

	void timer_callback() {
		RCLCPP_INFO(get_logger(), "timer");
		field_map m;
		m.map_height = 100.0;
		m.map_width = 100.0;
		m.wall.clear();

	  //RRT_star(point_2D _start, point_2D _goal, field_map _map, double _connect_radius, double _step_size, int _max_iterations, double _goal_threshold)	
		RRT_star r(point_2D(0.0, 0.0), point_2D(100.0, 100.0), m, 1.0, 1.0, 1000, 1.0);
		RRT_star r2(point_2D(0.0, 0.0), point_2D(100.0, 100.0), m,30.0, 20.0, 10000, 1.0);
		auto v = r2();
		
		if (v == nullptr) {
			RCLCPP_INFO(this->get_logger(), "v is nullptr");
		}
		vector<double> x, y;
		shared_ptr<node_t> goal_node;

		try
		{
			goal_node = goal(v);
		}
		catch(const std::exception& e)
		{
			RCLCPP_INFO(this->get_logger(), "in goal function");
			std::cerr << e.what() << '\n';
		}
		

		RCLCPP_INFO(this->get_logger(), "GOAL function is ok");

		if (goal_node == nullptr)  
			RCLCPP_INFO(this->get_logger(), "goal is nullptr");

		for (auto a = goal_node; (a != nullptr)? a->parent != nullptr: false; a = a->parent) {
			x.push_back(a->point.x);
			y.push_back(a->point.y);
		}

		x.push_back(v->point.x);
		y.push_back(v->point.y);

		RCLCPP_INFO(this->get_logger(), "size x: %d, y: %d", x.size(), y.size());

		std::reverse(x.begin(), x.end());
		std::reverse(y.begin(), y.end());
		path_planning::msg::PathData msg;
		msg.set__x(x);
		msg.set__y(y);
		p->publish(msg);
	}
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<map_server>());
	rclcpp::shutdown();
	return 0;
}