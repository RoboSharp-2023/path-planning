#include "../include/RRT_star.hpp"

using namespace std;

RRT_star::RRT_star(point_2D _start, point_2D _goal, field_map _map, double _connect_radius, double _step_size, int _max_iterations, double _goal_threshold)
					: start(_start), goal(_goal),f_map(_map), connect_radius(_connect_radius), step_size(_step_size), max_iterations(_max_iterations), goal_threshold(_goal_threshold) {
	auto root = make_shared<node>(start, nullptr, 0.0);
	nodes.clear();
	nodes.push_back(root);
}

void RRT_star::Plan(){
	for (int i = 0; i < max_iterations; i++) {
		point_2D rand_point = GenerateRandomPoint();
		auto nearest_node = FindNearestNode(rand_point);
		point_2D new_point = GenerateNewPoint(nearest_node->point, rand_point);

		if(!isCollision(nearest_node->point, new_point)) {
			auto new_node = make_shared<node>(new_point, nearest_node, nearest_node->cost + CalcDistance(new_point, nearest_node->point));

			nodes.push_back(new_node);
			ConnectToNearNode(new_node);
			RewriteNearNodes(new_node);
			if(CalcDistance(new_point, goal) < goal_threshold) {
				break;
			}
		}
	}

	//end
}

point_2D RRT_star::GenerateRandomPoint() {
	random_device rd;
	mt19937 gen(rd());

	uniform_real_distribution<> distX(0.0, f_map.map_width);
	uniform_real_distribution<> distY(0.0, f_map.map_height);

	return point_2D(distX(gen), distY(gen));
}

shared_ptr<node> RRT_star::FindNearestNode(point_2D point) {
	shared_ptr<node> nearest_node = nullptr;
	double min_distance = numeric_limits<double>::max();

	for (const auto& node: nodes) {
		double distance = CalcDistance(point, node->point);
		if (distance < min_distance) {
			min_distance = distance;
			nearest_node = node;
		}
	}

	return nearest_node;
}

point_2D RRT_star::GenerateNewPoint(point_2D nearest_point, point_2D random_point) {
	const auto delta_2D_point = random_point - nearest_point;
	const double distance = std::hypot(delta_2D_point.x, delta_2D_point.y);

	const double ratio = step_size / distance;

	return nearest_point + ratio * delta_2D_point;
}

bool RRT_star::isCollision(point_2D nearest_point ,point_2D new_point) {
	//衝突判定を行う
	//衝突したらtrue, しなかったらfalse
	return false;
}

void RRT_star::ConnectToNearNode(shared_ptr<node> new_node) {
	for (const auto& node: nodes) {
		const double distance = CalcDistance(node->point, new_node->point);

		if (distance < connect_radius) {
			node->children.push_back(new_node);
			new_node->parent = node;
			new_node->cost = node->cost + distance;
		}
	}
}

void RRT_star::RewriteNearNodes(shared_ptr<node> new_node) {
	for (const auto& node: nodes) {
		double distance = CalcDistance(new_node->point, node->point);
		double cost = new_node->cost + distance;
		if (cost < node->cost && !isCollision(node->point, new_node->point)) {
			node->parent->children.erase(std::remove(node->parent->children.begin(), node->parent->children.end(), node));
			node->parent = new_node;
			node->cost = cost;
			new_node->children.push_back(new_node);
		}
	}
}

vector<shared_ptr<node>> RRT_star::operator ()() {
	Plan();
	return nodes;
}
