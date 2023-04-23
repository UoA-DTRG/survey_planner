#ifndef PLANNER_CLASS_H
#define PLANNER_CLASS_H
#pragma once

#include <ros/ros.h>
#include "consts.h"
#include "Fiesta.h"
#include "ESDFMap.h"
#include "astar.h"
#include "bspline_opt.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/MarkerArray.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "std_msgs/Int32.h"
#include "tf/transform_listener.h"
#include "uav_messages/Heartbeat.h"
#include "uav_messages/TrajectoryTarget.h"
#include "uav_messages/TrajectoryTargetTest.h"
#include "uav_messages/EsdfQuery.h"
#include "flags.h"
#include <string>

using namespace std;

struct planner_class_params{
    // Frames
    string odom_frame;
    string fcu_frame;
    
    // Subscribe/publish topics
    string goal_topic;
    string trajectory_topic;
    string trajectory_index_topic;

    // Parameters
    double min_distance_to_obstacle;
    double robot_radius;
    double clear_radius;
    double voxel_resolution;

    double target_speed;

    double mpc_sample_time;

    bool use_soft_abort;
};

enum MISSION_STATE{
    STATE_IDLE,
    STATE_EXECUTE_MISSION
};

class planner_class{
public:
    planner_class(fiesta::Fiesta<FIESTA_DEPTH_TYPE, geometry_msgs::TransformStamped::ConstPtr> *f, tf::TransformListener *l);

    // FSM tick. This has to be public so that our base node can setup the timer correctly
    void fsm_tick(const ros::TimerEvent& event);
    void replan_tick(const ros::TimerEvent& event);
    
private:
    // Parameters
    struct planner_class_params m_params;

    // FSM state
    MISSION_STATE state;

    // Initialize node parameters
    void init();
    void init_parameters(ros::NodeHandle n, ros::NodeHandle pn);
    void init_subscribers(ros::NodeHandle n, ros::NodeHandle pn);
    void init_publishers(ros::NodeHandle n, ros::NodeHandle pn);

    // Trajectory planning functions
    bool optimize_bspline(bspline* spline, Eigen::Vector3d start_vel, Eigen::Vector3d end_vel, Eigen::Vector3d target_vel);
    bool check_collision(double& time_to_collision, int& collision_ind);
    void find_trajectory(bspline* spline, Eigen::Vector3d here, Eigen::Vector3d goal, Eigen::Vector3d start_vel, Eigen::Vector3d end_vel, Eigen::Vector3d target_vel);
    void get_current_position(tf::StampedTransform &tf);

    // Subscribe/publish callbacks
    void handle_new_goal(geometry_msgs::Pose p);
    void handle_new_goal_from(Eigen::Vector3d start, geometry_msgs::Pose p);
    void handle_new_nav_goal(geometry_msgs::PoseStamped ps);
    Eigen::Vector3d abort_path_following();
    void visualize_path(bspline* fitted_spline, bspline* optimized_spline);
    void publish_trajectory();
    void publish_trajectory_splice(int splice_index);
    void handle_new_trajectory_index(const std_msgs::Int32 curr);
    void handle_stop(std_msgs::Int32 unused);
    void handle_abort_finished(std_msgs::Int32 unused);

    // Trajectory plan service
    bool handle_new_trajectory_request(uav_messages::TrajectoryTarget::Request &req, uav_messages::TrajectoryTarget::Response &res);
    bool handle_new_trajectory_test_request(uav_messages::TrajectoryTargetTest::Request &req, uav_messages::TrajectoryTargetTest::Response &res);
    bool handle_esdf_lookup(uav_messages::EsdfQuery::Request &req, uav_messages::EsdfQuery::Response &res);
    bool handle_new_waypoints();
    bool handle_new_waypoints(Eigen::Vector3d start);
    ros::ServiceServer m_trajectory_service;
    ros::ServiceServer m_trajectory_test_service;
    ros::ServiceServer m_esdf_lookup_service;

    // Visualization stuff
    visualization_msgs::MarkerArray m_vis_array;

    // Subscribers
    ros::Subscriber m_goal_subscriber;
    ros::Subscriber m_nav_goal_subscriber;
    ros::Subscriber m_trajectory_index_subscriber;
    ros::Subscriber m_stop_subscriber;
    ros::Subscriber m_abort_finished_subscriber;
    
    // Publisher
    ros::Publisher m_vis_publisher;
    ros::Publisher m_trajectory_publisher;

    // Tf stuff
    tf::TransformListener *m_tf_listener;

    // Internal data pointers. We can change between different planners (when) they exist by changing the type of m_planner
    fiesta::Fiesta<FIESTA_DEPTH_TYPE, geometry_msgs::TransformStamped::ConstPtr> *m_fiesta_pointer;
    fiesta::ESDFMap* m_esdf_pointer;
    astar *m_planner;

    // Optimization stuff
    bspline_opt_params m_opt_params;
    bspline* m_opt_bspline;
    double calc_path_length(bspline* spline, double start, double end, double step);
    bool m_replan_mutex;
    bool m_block_replan; // Flag to block replan while waiting for an abort to finish

    // Safety stuff
    bool handle_heartbeat(uav_messages::Heartbeat::Request &req, uav_messages::Heartbeat::Response &res);
    ros::ServiceServer m_heartbeat_service;

    // Trajectory stuff
    trajectory_msgs::MultiDOFJointTrajectory m_trajectory;
    trajectory_msgs::MultiDOFJointTrajectory m_trajectory_splice;   // Secondary message we use to not break our index reference
    Eigen::Vector3d m_position_goal;
    int m_current_trajectory_index;
    bool m_current_traj_already_aborted;

    // Multi-waypoint stuff
    bool generate_multiwaypoint_trajectory(vector<bspline*> &splines);
    bool generate_multiwaypoint_path(vector<bspline*> &splines, vector<bspline*> &vis_spline);
    bool generate_multiwaypoint_path(Eigen::Vector3d here, vector<bspline*> &splines, vector<bspline*> &vis_spline, int start_ind);
    void visualize_path(vector<bspline*> fitted_spline);
    void publish_trajectory(vector<bspline*> splines);
    void publish_trajectory_splice(vector<bspline*> splines, int splice_index);
    double calc_path_length(double start, double end, double step);
    double calc_path_length(vector<bspline*> spline, double start, double end, double step);
    vector<Eigen::Vector3d> m_waypoints;
    vector<int> m_waypoint_indexes;
    Eigen::Vector3d m_start_vel;
    Eigen::Vector3d m_end_vel;
    Eigen::Vector3d m_target_vel;
    
};

#endif
