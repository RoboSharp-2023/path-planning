#include <memory>
#include <array>
#include <vector>
#include <utility>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "eigen3/Eigen/Dense"


using namespace std;

struct state{
	/// omega: angular velocity, alpha: angular acceleration
	double x, y, theta, v_x, v_y, omega, a_x, a_y, alpha;
	double delta_t;
	double sum_time = 0;
};

struct pos_t {
	double x, y, theta;
};

void straight_path_planning(const pos_t start, const pos_t end, vector<state>& trajectory){
	//set delta_t , max velocity and max acceleration
	constexpr double delta_t = 0.01;
	constexpr double max_v = 3.0;
	constexpr double max_a = 0.3 * 9.81;

	//calc distance and angle
	const double distance = hypot(start.x - end.x, start.y - end.y);
	const double angle    = atan2(end.y - start.y, end.x - start.x);

	//calc second and distance while accel
	constexpr double accel_sec = max_v / max_a;
	constexpr double distance_while_accel  = 0.5 * max_a * accel_sec * accel_sec;

	//calc max spd and sec running area
	const double distance_max_spd = distance - 2.0 * distance_while_accel;
	const double max_spd_sec = distance / max_v;

	//calc vector's length
	const double sum_time = max_spd_sec + 2.0 * accel_sec;
	const int vector_length = floor(sum_time/delta_t) + 1;

	//set vector's property
	trajectory.clear();
	trajectory.resize(vector_length);

	//set start state
	state current_state;
	current_state.x = start.x;
	current_state.y = start.y;
	current_state.theta = start.theta;
	current_state.delta_t = delta_t;
	current_state.v_x = 0;
	current_state.v_y = 0;
	current_state.a_x = 0;
	current_state.a_y = 0;
	current_state.omega = 0;
	current_state.alpha = 0;

	double progress_time = 0;
	for(double sec = 0; sec <= accel_sec; sec += delta_t){
		
	}
}



