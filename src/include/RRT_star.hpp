#include <vector>
#include <algorithm>
#include <queue>
#include <cmath>
#include <random>
#include <memory>

template <typename FloatingPoint>
static inline bool FloatingEqual (FloatingPoint left, FloatingPoint right, FloatingPoint error){
	return error > std::fabs(left-right);
}

struct point_2D {
	double x, y;

	point_2D(): x(0.0), y(0.0) {}
	point_2D(double _x, double _y): x(_x), y(_y) {}
};

point_2D operator + (const point_2D& first, const point_2D second) {
	return point_2D(first.x + second.x, first.y + second.y);
}
point_2D operator - (const point_2D& first, const point_2D second) {
	return point_2D(first.x - second.x, first.y - second.y);
}
point_2D operator * (const double ratio, point_2D point) {
	return point_2D(point.x * ratio, point.y * ratio);
}


struct robot_state {
	point_2D position;
	double angle;

	robot_state(): position(point_2D(0.0, 0.0)), angle(0.0) {}
	robot_state(double x, double y, double _angle): position(point_2D(x, y)), angle(_angle) {}
	robot_state(point_2D _pos, double _angle): position(_pos), angle(_angle) {}
};

struct node {
	point_2D point;
	std::shared_ptr<node> parent;
	double cost;
	std::vector<std::shared_ptr<node>> children;

	node(point_2D _point, std::shared_ptr<node> _parent, double _cost) : point(_point), parent(_parent), cost(_cost) {}
};

struct field_map{
	double map_width;
	double map_height;

	std::vector<std::pair<point_2D, point_2D>> wall;
};

class RRT_star {
	point_2D start;
	point_2D goal;
	double connect_radius;
	double step_size;
	int max_iterations;
	double goal_threshold;
	std::vector<std::shared_ptr<node>> nodes;
	field_map f_map;

	

	void Plan();// Done
	void ConnectToNearNode(std::shared_ptr<node> new_node);
	void RewriteNearNodes(std::shared_ptr<node> new_node);
	bool isCollision(point_2D nearest_point, point_2D new_point);
	double CalcDistance(const point_2D& point1, const point_2D& point2); //Done
	point_2D GenerateRandomPoint();// Done
	point_2D GenerateNewPoint(point_2D nearest_point, point_2D random_point);// Done
	std::shared_ptr<node> FindNearestNode(point_2D point); // Done

public:
	RRT_star(point_2D _start, point_2D _goal, field_map _map, double _connect_radius, double _step_size, int _max_iterations, double _goal_threshold);
//impl
private:
	double CalcDistance(const point_2D& point1, const point_2D& point2){
		return std::hypot(point1.x - point2.x, point1.y - point2.y);
	}
};
