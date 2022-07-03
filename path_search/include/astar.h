#ifndef ASTAR_H
#define ASTAR_H
#pragma once

#include "consts.h"
#include "Fiesta.h"
#include "ESDFMap.h"
#include "Eigen/Eigen"
#include <vector>

using namespace std;

template<typename T>
struct matrix_hash1 : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class Node{
public:
    Eigen::Vector3d pos;
    Eigen::Vector3i vox;
    double gscore, fscore;
    Node* parent;

    Node(){
        parent = NULL;
        gscore = 0;
        fscore = 0;
    }
    ~Node(){};
};

auto cmp = [](Node* x1, Node* x2) { return x1->fscore > x2->fscore; };
typedef unordered_map<Eigen::Vector3i, Node*, matrix_hash1<Eigen::Vector3i>> NodeMap;
typedef priority_queue<Node*, vector<Node*>, decltype(cmp)> NodePriorityQueue;


class astar{
public:
    astar(fiesta::Fiesta<FIESTA_DEPTH_TYPE, geometry_msgs::TransformStamped::ConstPtr> *f, double min_obstacle_distance, double m_clear_radius);
    vector<Eigen::Vector3d> find_path(Eigen::Vector3d here, Eigen::Vector3d goal, bool &status);

private:
    fiesta::Fiesta<FIESTA_DEPTH_TYPE, geometry_msgs::TransformStamped::ConstPtr> *m_fiesta_pointer;
    fiesta::ESDFMap *m_map;
    double m_min_obstacle_distance;
    int m_max_iters;
    double m_clear_radius;
    double m_map_resolution;
    double get_heuristic(Eigen::Vector3d parent, Eigen::Vector3d child);
    double get_eucledian_heuristic(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double get_manhattan_heuristic(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double get_diag_heuristic(Eigen::Vector3d x1, Eigen::Vector3d x2);
    void update_node(Eigen::Vector3i vox, NodeMap &map, NodePriorityQueue &queue, Node* parent, double gscore, double fscore);
    vector<Eigen::Vector3i> get_neighbors(Eigen::Vector3i vox, Eigen::Vector3i start);

};

#endif