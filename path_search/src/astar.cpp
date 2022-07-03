#include "astar.h"
#include <unordered_map>
#include "eigen_hash.h"
#include "planner_class.h"
//#include "timer.hpp"

using namespace std;

astar::astar(fiesta::Fiesta<FIESTA_DEPTH_TYPE, geometry_msgs::TransformStamped::ConstPtr> *f, double min_obstacle_distance, double clear_radius){
    this->m_min_obstacle_distance = min_obstacle_distance;
    this->m_fiesta_pointer = f;
    this->m_max_iters = 60000;
    this->m_clear_radius = clear_radius;
    this->m_map = f->GetESDFMapPointer();
    this->m_map_resolution = this->m_map->GetResolution();
}


vector<Eigen::Vector3d> astar::find_path(Eigen::Vector3d here, Eigen::Vector3d goal, bool &status){
    //tick();
    Eigen::Vector3i startIndex, endIndex;
    this->m_map->Pos2Vox(here, startIndex);
    this->m_map->Pos2Vox(goal, endIndex);
    bool path_found = false;
    vector<Eigen::Vector3d> path;
    int iters = 0;

    // Don't touch the ESDF in A* search
    // if(this->m_map->GetDistance(startIndex) < this->m_min_obstacle_distance){
    //     // it is impossible for start to be occupied. Clear m_clear_radius around this
    //     for(double i = -this->m_clear_radius; i <= this->m_clear_radius; i+= this->m_map->GetResolution()){
    //         for(double j = -this->m_clear_radius; j <= this->m_clear_radius; j+= this->m_map->GetResolution()){
    //             for(double k = -this->m_clear_radius; k <= this->m_clear_radius; k+= this->m_map->GetResolution()){
    //                 Eigen::Vector3d temp;
    //                 Eigen::Vector3i tempIdx;
    //                 temp << i, j, k;
    //                 temp += here;
    //                 this->m_map->Pos2Vox(temp, tempIdx);
    //                 this->m_map->SetOccupancy(tempIdx, 0);
    //             }
    //         }
    //     }

    //     this->m_map->UpdateESDF();
    // }

    if(this->m_map->GetDistance(endIndex) < this->m_min_obstacle_distance){
        //ROS_INFO("Failed to find path!, Took %lu microseconds", tock());
        status = false;
        return path;
    }

    NodeMap node_lookup;
    NodePriorityQueue open_set(cmp); 

    Node *curr = new Node();
    curr->gscore = 0;
    curr->fscore = this->get_heuristic(here, goal);
    curr->pos = here;
    curr->vox = startIndex;
    curr->parent = NULL;

    node_lookup[curr->vox] = curr;

    open_set.push(curr);

    while(!open_set.empty()){
        iters++;
        if(iters > this->m_max_iters) break;
        curr = open_set.top();

        // Check if we are at the goal
        if(curr->vox(0) == endIndex(0) && curr->vox(1) == endIndex(1) && curr->vox(2) == endIndex(2)){
            path_found = true;
            break;
        }

        open_set.pop();
        vector<Eigen::Vector3i> neighbors = this->get_neighbors(curr->vox, startIndex);
        for(auto neighbor : neighbors){
            Eigen::Vector3d neighbor_pos;
            this->m_map->Vox2Pos(neighbor, neighbor_pos);
            double estimated_gscore = curr->gscore + (curr->pos - neighbor_pos).norm();
            if(node_lookup.find(neighbor) == node_lookup.end()){
                update_node(neighbor, node_lookup, open_set, curr, estimated_gscore, estimated_gscore + this->get_heuristic(neighbor_pos, goal));
                open_set.push(node_lookup[neighbor]);
                continue;
            }

            // We shoul've hit the previous if statement if the neighbor didn't exist
            Node *n_node = node_lookup[neighbor];
            if(n_node->gscore > estimated_gscore){
                update_node(neighbor, node_lookup, open_set, curr, estimated_gscore, estimated_gscore + this->get_heuristic(neighbor_pos, goal));
                open_set.push(node_lookup[neighbor]);
            }
        }
    }

    if(path_found){
        Node *end = node_lookup[endIndex];
        while(end->parent != NULL){
            path.push_back(end->pos);
            end = end->parent;
        }
        // Add the current position to the path
        path.push_back(here);
    }

    // Reverse the path
    reverse(path.begin(), path.end());

    // Memory leak all day!
    /*for(auto n : node_lookup){
        delete &n;
    }*/

    //ROS_INFO("A* Done, planning took %lu microseconds with %i iters", tock(), iters);
    
    status = true;

    return path;

}

vector<Eigen::Vector3i> astar::get_neighbors(Eigen::Vector3i vox, Eigen::Vector3i start){
    // Use 26 connectivity
    Eigen::Vector3i curr;
    vector<Eigen::Vector3i> out;
    for(int i = -1; i <= 1; ++i){
        for(int j = -1; j <= 1; ++j){
            for(int k = -1; k <= 1; ++k){
                if(i == 0 && j == 0 && k == 0) continue;
                curr = vox;
                curr(0) += i;
                curr(1) += j;
                curr(2) += k;

                // Only add to neighbor list if both collision free and more than clear radius away
                if ((this->m_map->GetDistance(curr) < this->m_min_obstacle_distance) &&
                    ((double) (start - vox).norm() * this->m_map_resolution) > this->m_clear_radius) continue;
                if(!this->m_map->VoxInRange(curr)) continue;

                out.push_back(curr);
            }
        }
    }
    return out;
}

void astar::update_node(Eigen::Vector3i vox, NodeMap &map, NodePriorityQueue &queue, Node* parent, double gscore, double fscore){
    Node* n;

    if(map.find(vox) == map.end()){
        //printf("Making new node\n");
        Eigen::Vector3d here;
        this->m_map->Vox2Pos(vox, here);
        // Vox doesn't exist, we should make it
        n = new Node();
        n->pos = here;
        n->vox = vox;

        map[vox] = n;
    }else{
        //printf("Expanding existing node!\n");
    }

    n = map[vox];

    n->parent = parent;
    n->gscore = gscore;
    n->fscore = fscore;
    
    //printf("Pushing new node at %x to queue\n", (unsigned long) n);
    queue.push(n);
}

inline double astar::get_heuristic(Eigen::Vector3d x1, Eigen::Vector3d x2){
    return this->get_manhattan_heuristic(x1, x2);
}

inline double astar::get_eucledian_heuristic(Eigen::Vector3d x1, Eigen::Vector3d x2){
    return (x2-x1).norm();
}

inline double astar::get_manhattan_heuristic(Eigen::Vector3d x1, Eigen::Vector3d x2){
    auto dist = x2 - x1;
    return (fabs(dist(0)) + fabs(dist(1)) + fabs(dist(2)));
}

inline double astar::get_diag_heuristic(Eigen::Vector3d x1, Eigen::Vector3d x2){

}