#include "RRT_star.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "eigen3/Eigen/Dense"



class PID {
	const double dt;
	const double kp, ki, kd;
	double error, error_sum, error_diff, pre_error;

public:
	PID(double _dt, double _kp, double _ki, double _kd): 
		dt(_dt), kp(_kp), ki(_ki), kd(_kd), error(0.0), error_sum(0.0), error_diff(0.0), pre_error(0.0) {}

	double operator() (double error_){
		pre_error = error;
		error = error_;
		error_sum += (pre_error + error) / 2.0 * dt;
		error_diff = (error - pre_error) / dt;

		return kp * error + ki * error_sum + kd * error_diff;
	}
};


class velocity_profiling {};

int main(int argc, char*argv[]) {
	field_map m;
	m.map_width = 1000;
	m.map_height = 1000;
	RRT_star r(point_2D(0, 0), point_2D(10.0, 10.1), m, 0.5, 1, 1000, 0.1);
	r();
}
