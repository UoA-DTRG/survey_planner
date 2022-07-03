#ifndef __BSPLINE_OPT_H
#define __BSPLINE_OPT_H
#pragma once

#include "ros/ros.h"
#include <Eigen/Eigen>
#include "bspline.h"
#include "ESDFMap.h"
#include <vector>

using namespace std;

enum OPTIMIZATION_STATUS{
    OPT_OK,
    OPT_ERROR_BSPLINE_TOOSHORT,
    OPT_ERROR_NO_CONVERGE,
    OPT_ERROR_FAILED,
    OPT_ERROR_NULLPTR,
    OPT_ERROR_TARGET_VEL_ZERO
};

struct bspline_opt_params{
    // Collision
    double collision_weight;
    double collision_min_distance;

    // Acceleration
    Eigen::Vector3d acceleration_weight;
    Eigen::Vector3d max_acc_weight;
    Eigen::Vector3d max_acc;

    // Velocity
    Eigen::Vector3d velocity_weight;
    Eigen::Vector3d velocity_target;
    Eigen::Vector3d max_vel_weight;
    Eigen::Vector3d max_vel;

    // Position
    Eigen::Vector3d start_end_position_weight;
    double start_end_position_tolerance;

    // Time costs
    double time_cost;

    // FOV costs
    bool vfov_enabled;
    double vfov;
    double vfov_cost;
    bool hfov_enabled;
    double hfov;
    double hfov_cost;
};

class bspline_opt{
public:
    bspline_opt(bspline* spline, fiesta::ESDFMap* map, struct bspline_opt_params params);
    bspline_opt(vector<bspline*> splines, fiesta::ESDFMap* map, struct bspline_opt_params params);
    void set_params(struct bspline_opt_params p){ this->m_params = p; }
    void trajectory_optimize(vector<Eigen::Vector3d>* control_points, double* interval, int order);
    bool trajectory_optimize(vector<vector<Eigen::Vector3d>>* control_points, vector<double> &intervals, int order);
    void set_termination_velocity_states(Eigen::Vector3d start_vel, Eigen::Vector3d end_vel);
    void set_target_velocity_vector(Eigen::Vector3d target_vel);
    OPTIMIZATION_STATUS optimize();

private:
    struct bspline_opt_params m_params;
    Eigen::Vector3d m_start_vel;
    Eigen::Vector3d m_end_vel;
    bspline* m_bspline;
    vector<bspline*> m_bspline_multi;
    fiesta::ESDFMap* m_map;
};

#endif