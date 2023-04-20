#include "../include/RRT_star.hpp"

using namespace std;

RRT_star::RRT_star(point_2D _start, point_2D _goal, field_map _map, double _connect_radius, double _step_size, int _max_iterations, double _goal_threshold)
					: start(_start), goal(_goal),f_map(_map), connect_radius(_connect_radius), step_size(_step_size), max_iterations(_max_iterations), goal_threshold(_goal_threshold) {
	auto root = make_shared<node_t>(start, nullptr, 0.0);
	nodes.clear();
	nodes.push_back(root);
}

void RRT_star::Plan(){
	for (int i = 0; i < max_iterations; i++) {
		// ランダムなポイントをサンプリング
		point_2D rand_point = GenerateRandomPoint();

		// 最も近いノードを検索
		auto nearest_node = FindNearestNode(rand_point);

		// 最も近いノードから指定の距離を表す点new_pointを計算
		point_2D new_point = GenerateNewPoint(nearest_node->point, rand_point);

		// 衝突判定
		if(!isCollision(nearest_node->point, new_point)) {
			//new_pointのノードを作成
			auto new_node = make_shared<node_t>(new_point, nearest_node, nearest_node->cost + CalcDistance(new_point, nearest_node->point));

			// ノードリストに追加
			nodes.push_back(new_node);

			// 新しいノードの近傍にあるノードを検索して最も少ないコストで計算できる点を計算する
			ConnectToNearNode(new_node);

			// ノードを最も小さいコストのところに再接続する
			RewriteNearNodes(new_node);

			//ゴールに到達したら処理を中断する
			if(CalcDistance(new_point, goal) < goal_threshold) {
				new_node->is_goal = true;
				break;
			}
		}
	}

	//end
}

random_device rd;
mt19937 gen(rd());
// ランダムなポイントをサンプリング
point_2D RRT_star::GenerateRandomPoint() {
	uniform_real_distribution<> distX(0.0, f_map.map_width);
	uniform_real_distribution<> distY(0.0, f_map.map_height);

	return point_2D(distX(gen), distY(gen));
}

// 最も近いノードを検索
shared_ptr<node_t> RRT_star::FindNearestNode(point_2D point) {
	shared_ptr<node_t> nearest_node = nullptr;
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

// 最も近いノードから指定の距離を表す点new_pointを計算
point_2D RRT_star::GenerateNewPoint(point_2D nearest_point, point_2D random_point) {
	const auto delta_2D_point = random_point - nearest_point;
	const double distance = std::hypot(delta_2D_point.x, delta_2D_point.y);

	const double ratio = step_size / distance;

	return nearest_point + (ratio * delta_2D_point);
}

// 衝突判定をおこなう
bool RRT_star::isCollision(point_2D nearest_point ,point_2D new_point) {
	//衝突判定を行う
	//衝突したらtrue, しなかったらfalse
	
	
	return false;
}

// 新しいノードの近傍にあるノードを検索して最も少ないコストで計算できる点を計算する
void RRT_star::ConnectToNearNode(shared_ptr<node_t>& new_node) {
	auto least_cost_node = make_shared<node_t>(point_2D(0, 0), nullptr, numeric_limits<double>::max());
	for (const auto& node: nodes) {
		const double distance = CalcDistance(node->point, new_node->point);

		if (distance < least_cost_node->cost) {
			node->children.push_back(new_node);
			new_node->parent = node;
			new_node->cost = node->cost + distance;
			least_cost_node = node;
		}
	}
}

// ノードを最も小さいコストのところに再接続する
void RRT_star::RewriteNearNodes(shared_ptr<node_t> new_node) {
	for (const auto& node: nodes) {
		double distance = CalcDistance(new_node->point, node->point);
		double cost = new_node->cost + distance;
		if (cost < node->cost && !isCollision(node->point, new_node->point)) {
			node->parent->children.clear();
			node->parent = new_node;
			node->cost = cost;
			node->children.push_back(new_node);
		}
	}
}

shared_ptr<node_t> RRT_star::operator ()() {
	Plan();
	return nodes[0];
}
