#ifndef _OPTIMIZATION_PARAMETERS_H
#define _OPTIMIZATION_PARAMETERS_H
#pragma once

#include "ros/ros.h"
#include "bspline_opt.h"
#include "Eigen/Eigen"

inline void load_optimization_parameters(bspline_opt_params* params, ros::NodeHandle n, ros::NodeHandle pn){
    pn.param("opt/collision_weight", params->collision_weight, 20.0);
    pn.param("opt/collision_min_distance", params->collision_min_distance, 1.5);
    pn.param("opt/acceleration_weight_x", params->acceleration_weight.data()[0], 10.0);
    pn.param("opt/acceleration_weight_y", params->acceleration_weight.data()[1], 10.0);
    pn.param("opt/acceleration_weight_z", params->acceleration_weight.data()[2], 10.0);
    pn.param("opt/max_acc_weight_x", params->max_acc_weight.data()[0], 50.0);
    pn.param("opt/max_acc_weight_y", params->max_acc_weight.data()[1], 50.0);
    pn.param("opt/max_acc_weight_z", params->max_acc_weight.data()[2], 50.0);
    pn.param("opt/max_acc_x", params->max_acc.data()[0], 15.0);
    pn.param("opt/max_acc_y", params->max_acc.data()[1], 15.0);
    pn.param("opt/max_acc_z", params->max_acc.data()[2], 15.0);
    pn.param("opt/start_end_position_weight_x", params->start_end_position_weight.data()[0], 10.0);
    pn.param("opt/start_end_position_weight_y", params->start_end_position_weight.data()[1], 10.0);
    pn.param("opt/start_end_position_weight_z", params->start_end_position_weight.data()[2], 10.0);
    pn.param("opt/start_end_position_tolerance", params->start_end_position_tolerance, 0.2);
    pn.param("opt/velocity_weight_x", params->velocity_weight.data()[0], 3.0);
    pn.param("opt/velocity_weight_y", params->velocity_weight.data()[1], 3.0);
    pn.param("opt/velocity_weight_z", params->velocity_weight.data()[2], 3.0);
    pn.param("opt/max_vel_weight_x", params->max_vel_weight.data()[0], 15.0);
    pn.param("opt/max_vel_weight_y", params->max_vel_weight.data()[1], 15.0);
    pn.param("opt/max_vel_weight_z", params->max_vel_weight.data()[2], 15.0);
    pn.param("opt/max_vel_x", params->max_vel.data()[0], 5.0);
    pn.param("opt/max_vel_y", params->max_vel.data()[1], 5.0);
    pn.param("opt/max_vel_z", params->max_vel.data()[2], 2.0);
    pn.param("opt/time_cost", params->time_cost, 10.0);
    pn.param("opt/vfov_enabled", params->vfov_enabled, false);
    pn.param("opt/vfov_cost", params->vfov_cost, 100.0);
    pn.param("opt/vfov", params->vfov, 1.047);
    pn.param("opt/hfov_enabled", params->hfov_enabled, false);
    pn.param("opt/hfov_cost", params->hfov_cost, 100.0);
    pn.param("opt/hfov", params->hfov, 1.047);
}

#endif