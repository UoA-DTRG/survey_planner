#include "planner_class.h"
#include "Fiesta.h"
#include "ESDFMap.h"
#include "helpers.hpp"
#include "bspline.h"
#include "bspline_opt.h"
#include "optimization_parameters.hpp"
#include "timer.hpp"

using namespace std;

planner_class::planner_class(fiesta::Fiesta<FIESTA_DEPTH_TYPE, geometry_msgs::TransformStamped::ConstPtr> *f, tf::TransformListener *l){
    m_tf_listener = l;
    init();

    m_fiesta_pointer = f;
    m_planner = new astar(f, m_params.min_distance_to_obstacle, m_params.clear_radius);
    m_esdf_pointer = f->GetESDFMapPointer();
}

void planner_class::init(){
    ros::NodeHandle n("");
    ros::NodeHandle pn("~");

    state = STATE_IDLE;
    m_replan_mutex = false;
    m_block_replan = false;

    init_parameters(n, pn);
    init_subscribers(n, pn);
    init_publishers(n, pn);
}

void planner_class::init_parameters(ros::NodeHandle n, ros::NodeHandle pn){
    // Frames
    string odom_frame;
    string fcu_frame;
    
    // Subscribe/publish topics
    string goal_topic;

    // Parameters
    double min_distance_to_obstacle;

    pn.param("odom_frame", m_params.odom_frame, string("odom_frame"));
    pn.param("fcu_frame", m_params.fcu_frame, string("fcu_link"));

    pn.param("goal_topic", m_params.goal_topic, string("planner/goal"));
    pn.param("trajectory_topic", m_params.trajectory_topic, string("planner/trajectory_out"));
    pn.param("trajectory_index_topic", m_params.trajectory_index_topic, string("path/current_index"));

    pn.param("min_distance_to_obstacle", m_params.min_distance_to_obstacle, 0.5);
    pn.param("robot_radius", m_params.robot_radius, 0.4);
    pn.param("resolution", m_params.voxel_resolution, 0.2);
    pn.param("target_speed", m_params.target_speed, 1.0);
    pn.param("clear_radius", m_params.clear_radius, 0.2);

    pn.param("mpc_sample_time", m_params.mpc_sample_time, 0.1);

    pn.param("use_soft_abort", m_params.use_soft_abort, true);

    load_optimization_parameters(&m_opt_params, n, pn);
}

bool planner_class::handle_heartbeat(uav_messages::Heartbeat::Request &req, uav_messages::Heartbeat::Response &res){
    res.heartbeat = req.heartbeat;
    return true;
}

bool planner_class::handle_new_trajectory_request(uav_messages::TrajectoryTarget::Request &req, uav_messages::TrajectoryTarget::Response &res){
    // For now just use the first point
    /*geometry_msgs::Pose p;
    p.position = req.waypoints[0];

    handle_new_goal(p);*/

    m_waypoints.clear();

    for(auto p : req.waypoints){
        Eigen::Vector3d point;
        helpers::msg_to_eigen_vector(point, p);
        m_waypoints.push_back(point);
    }

    ROS_INFO("Got waypoints!");
    for(auto w : m_waypoints){
        ROS_INFO("%f, %f, %f", w(0), w(1), w(2));
    }

    helpers::msg_to_eigen_vector(m_start_vel, req.start_velocity);
    helpers::msg_to_eigen_vector(m_end_vel, req.end_velocity);
    helpers::msg_to_eigen_vector(m_target_vel, req.target_velocity);

    bool status = handle_new_waypoints();
    if(!status) return false;

    res.success = true;
    res.error = uav_messages::TrajectoryTarget::Request::OK;

    return true;
}

bool planner_class::generate_multiwaypoint_path(Eigen::Vector3d here, vector<bspline*> &splines, vector<bspline*> &vis_spline, int start_ind = 0){
    ROS_INFO("Starting path search with %lu waypoints", m_waypoints.size());

    int num_splines = 0;

    for(int i = start_ind; i < m_waypoints.size(); ++i){
        bool status;
        Eigen::Vector3d start;
	    if(i > start_ind){
            start = m_waypoints[i - 1];
        } else {
            start = here;
        }

        // If the waypoint is too close then cull the waypoint
        if((m_waypoints[i] - start).norm() < 1.0){
            m_waypoints.erase(m_waypoints.begin() + i);
            i--;
            continue;
        }

        auto path = m_planner->find_path(start, m_waypoints[i], status);

        ROS_INFO("Working on waypoint %i, status %i", i, status);

        if(status == false){
            // m_waypoints[i+1] is occupied, therefore we delete it and try again;
            m_waypoints.erase(m_waypoints.begin() + i);
            i--;
        }else{
            // Fit spline. There's probably a nicer way of deep copying here but I'm lazy
            auto fitted_spline = bspline_util::fit_bspline(&path, 0.1);
            auto v_spline = bspline_util::fit_bspline(&path, 0.1);

            splines.push_back(fitted_spline);
            vis_spline.push_back(v_spline);
            
            // Increment spline counter
            num_splines++;
        }
    }

    if(num_splines == 0){
        return false;
    }

    return true;
}

bool planner_class::generate_multiwaypoint_path(vector<bspline*> &splines, vector<bspline*> &vis_spline){
    // Get current position
    tf::StampedTransform odom_to_fcu;
    Eigen::Vector3d here;
    get_current_position(odom_to_fcu);
    helpers::tf_to_eigen_vector(here, odom_to_fcu);

    ROS_INFO("Got current position");

    return generate_multiwaypoint_path(here, splines, vis_spline);
}

bool planner_class::handle_new_trajectory_test_request(uav_messages::TrajectoryTargetTest::Request &req, uav_messages::TrajectoryTargetTest::Response &res){
    tick();

    // Clear waypoints and setup for single waypoint mode
    m_waypoints.clear();
    m_waypoint_indexes.clear();
    Eigen::Vector3d goal, here;
    helpers::msg_to_eigen_vector(here, req.start);
    helpers::msg_to_eigen_vector(goal, req.end);
    m_position_goal = goal;

    // Setup optimization
    bspline* optimized_spline;
    Eigen::Vector3d zero, target;
    // TODO: Figure out what to do with zero and target
    zero << 0.0, 0.0, 0.0;
    target << 1.0, 0.0, 0.0;

    // Find path
    bool status;
    auto path = m_planner->find_path(here, goal, status);

    if(!status){
        return false;
    }

    // Fit bspline to path
    m_opt_bspline = bspline_util::fit_bspline(&path, 0.1);
    auto fitted_spline = bspline_util::fit_bspline(&path, 0.1);

    // No valid trajectory has been generated. Fail gracefully
    if(m_opt_bspline == NULL){
        return false;
    }

    // Optimize bspline
    if(!optimize_bspline(m_opt_bspline, zero, zero, target)){
        for(auto point : path){
            printf("%f, %f, %f\n", point(0), point(1), point(2));
        }
    }

    // Clean up to avoid memory leaking
    //delete fitted_spline;
    //delete m_opt_bspline;

    unsigned long time_taken = tock();

    // Set state to path following
    state = STATE_IDLE;

    publish_trajectory();

    visualize_path(fitted_spline, m_opt_bspline);

    delete fitted_spline;

    res.success = true;
    res.error = uav_messages::TrajectoryTarget::Request::OK;
    res.runtime_us = time_taken;

    auto vel_spline = m_opt_bspline->get_derivative();
    auto acc_spline = vel_spline.get_derivative();

    auto pos = m_opt_bspline->get_discrete_bspline(m_params.mpc_sample_time);
    auto vel = vel_spline.get_discrete_bspline(m_params.mpc_sample_time);
    auto acc = acc_spline.get_discrete_bspline(m_params.mpc_sample_time);

    res.traj.points.clear();
    res.traj.header.frame_id = m_params.odom_frame;
    res.traj.header.seq++;
    res.traj.header.stamp = ros::Time::now();

    // Pad the start of the trajectory with 0.5 seconds of repeating starting points
    // Hopefully this gives the controller a chance to never fall behind
    for(size_t i = 0; i < pos.size() - 1; ++i){
        geometry_msgs::Transform pos_msg;
        geometry_msgs::Twist vel_msg, acc_msg;

        helpers::eigen_vector_to_msg(pos[i], pos_msg.translation);
        helpers::eigen_vector_to_msg(vel[i], vel_msg.linear);
        helpers::eigen_vector_to_msg(acc[i], acc_msg.linear);

        trajectory_msgs::MultiDOFJointTrajectoryPoint p;
        p.time_from_start = ros::Duration((double) i * 0.1);
        p.accelerations.push_back(acc_msg);
        p.velocities.push_back(vel_msg);
        p.transforms.push_back(pos_msg);

        res.traj.points.push_back(p);
    }

    return true;
}

bool planner_class::handle_esdf_lookup(uav_messages::EsdfQuery::Request &req, uav_messages::EsdfQuery::Response &res){
    Eigen::Vector3d pos, grad;
    pos << req.x, req.y, req.z;
    double dist = m_esdf_pointer->GetDistWithGradTrilinear(pos, grad);
    res.distance = dist;

    return true;
}

bool planner_class::handle_new_waypoints(Eigen::Vector3d start){
    // Set m_position_goal to the final waypoint for now to avoid breaking replanning
    m_position_goal = m_waypoints[m_waypoints.size() - 1];

    vector<bspline*> fitted_splines;
    vector<bspline*> trajectory_splines;

    if(!generate_multiwaypoint_path(start, trajectory_splines, fitted_splines)){
        ROS_ERROR("Failed to generate path. Are any of the desired locations clear?");
        abort_path_following();
        return false;
    }

    if(!generate_multiwaypoint_trajectory(trajectory_splines)){
        ROS_ERROR("Failed to optimize trajectory!");
        abort_path_following();
        return false;
    }

    publish_trajectory(trajectory_splines);
    visualize_path(fitted_splines);

    // State shift to execute mission, so collision checking works
    state = STATE_EXECUTE_MISSION;

    return true;
}

bool planner_class::handle_new_waypoints(){
    // Set m_position_goal to the final waypoint for now to avoid breaking replanning
    m_position_goal = m_waypoints[m_waypoints.size() - 1];

    vector<bspline*> fitted_splines;
    vector<bspline*> trajectory_splines;

    if(!generate_multiwaypoint_path(trajectory_splines, fitted_splines)){
        ROS_ERROR("Failed to generate path. Are any of the desired locations clear?");
        abort_path_following();
        return false;
    }

    if(!generate_multiwaypoint_trajectory(trajectory_splines)){
        ROS_ERROR("Failed to optimize trajectory!");
        abort_path_following();
        return false;
    }

    publish_trajectory(trajectory_splines);
    visualize_path(fitted_splines);

    // // Clean up
    // for(auto f : fitted_splines){
    //     delete f;
    // }
    
    // for(auto t : trajectory_splines){
    //     delete t;
    // }

    // State shift to execute mission, so collision checking works
    state = STATE_EXECUTE_MISSION;

    return true;
}

bool planner_class::generate_multiwaypoint_trajectory(vector<bspline*> &splines){
    auto optimizer = std::make_unique<bspline_opt>(splines, m_fiesta_pointer->GetESDFMapPointer(), m_opt_params);

    optimizer->set_target_velocity_vector(m_target_vel);
    optimizer->set_termination_velocity_states(m_start_vel, m_end_vel);
    auto ret = optimizer->optimize();

    if(ret == OPT_OK){
        return true;
    }
    
    ROS_ERROR("Optimizer returned error %i", ret);

    return false;
}

void planner_class::publish_trajectory(vector<bspline*> splines){
    m_trajectory.points.clear();
    m_trajectory.header.frame_id = m_params.odom_frame;
    m_trajectory.header.seq++;
    m_trajectory.header.stamp = ros::Time::now();
    m_waypoint_indexes.clear();

    m_current_trajectory_index = 0;

    auto initial_pos = splines[0]->get_discrete_bspline_point(0);
    Eigen::Vector3d zero;
    zero << 0, 0, 0;

    // Pad the start of the trajectory with 0.5 seconds of repeating starting points
    // Hopefully this gives the controller a chance to never fall behind
    for(size_t i = 0; i < 5; ++i){
        geometry_msgs::Transform pos_msg;
        geometry_msgs::Twist vel_msg, acc_msg;

        helpers::eigen_vector_to_msg(initial_pos, pos_msg.translation);
        helpers::eigen_vector_to_msg(zero, vel_msg.linear);
        helpers::eigen_vector_to_msg(zero, acc_msg.linear);

        trajectory_msgs::MultiDOFJointTrajectoryPoint p;
        p.time_from_start = ros::Duration((double) i * 0.1);
        p.accelerations.push_back(acc_msg);
        p.velocities.push_back(vel_msg);
        p.transforms.push_back(pos_msg);

        m_trajectory.points.push_back(p);
    }

    double offset = 5;

    for(auto spline : splines){
        auto vel_spline = spline->get_derivative();
        auto acc_spline = vel_spline.get_derivative();

        auto pos = spline->get_discrete_bspline(m_params.mpc_sample_time);
        auto vel = vel_spline.get_discrete_bspline(m_params.mpc_sample_time);
        auto acc = acc_spline.get_discrete_bspline(m_params.mpc_sample_time);

        // Publish the trajectory
        for(size_t i = 0; i < pos.size() - 1; ++i){
            geometry_msgs::Transform pos_msg;
            geometry_msgs::Twist vel_msg, acc_msg;

            helpers::eigen_vector_to_msg(pos[i], pos_msg.translation);
            helpers::eigen_vector_to_msg(vel[i], vel_msg.linear);
            helpers::eigen_vector_to_msg(acc[i], acc_msg.linear);

            trajectory_msgs::MultiDOFJointTrajectoryPoint p;
            p.time_from_start = ros::Duration((((double) i) + offset) * m_params.mpc_sample_time);
            p.accelerations.push_back(acc_msg);
            p.velocities.push_back(vel_msg);
            p.transforms.push_back(pos_msg);

            m_trajectory.points.push_back(p);
        }

        offset = offset + (pos.size() - 1);
        m_waypoint_indexes.push_back(offset);
    }

    m_trajectory_publisher.publish(m_trajectory);
}

void planner_class::publish_trajectory_splice(vector<bspline*> splines, int splice_index){
    m_trajectory.header.frame_id = m_params.odom_frame;
    m_trajectory.header.seq++;
    m_trajectory.header.stamp = ros::Time::now();
    // Resize the trajectory to the splice size
    m_trajectory.points.resize(splice_index + 1);

    // TODO: Make this more elegant
    m_trajectory_splice.points.clear();
    m_trajectory_splice.header.frame_id = m_params.odom_frame;
    m_trajectory_splice.header.seq++;
    m_trajectory_splice.header.stamp = ros::Time::now();

    double offset = 0;
    
    for(auto spline : splines){
        auto vel_spline = spline->get_derivative();
        auto acc_spline = vel_spline.get_derivative();

        auto pos = spline->get_discrete_bspline(m_params.mpc_sample_time);
        auto vel = vel_spline.get_discrete_bspline(m_params.mpc_sample_time);
        auto acc = acc_spline.get_discrete_bspline(m_params.mpc_sample_time);

        // Publish the trajectory
        for(size_t i = 1; i < pos.size(); ++i){
            geometry_msgs::Transform pos_msg;
            geometry_msgs::Twist vel_msg, acc_msg;

            helpers::eigen_vector_to_msg(pos[i], pos_msg.translation);
            helpers::eigen_vector_to_msg(vel[i], vel_msg.linear);
            helpers::eigen_vector_to_msg(acc[i], acc_msg.linear);

            trajectory_msgs::MultiDOFJointTrajectoryPoint p;
            p.time_from_start = ros::Duration((((double) i) + offset) * m_params.mpc_sample_time);
            p.accelerations.push_back(acc_msg);
            p.velocities.push_back(vel_msg);
            p.transforms.push_back(pos_msg);

            m_trajectory.points.push_back(p);

            p.time_from_start += ros::Duration(((double)splice_index) * m_params.mpc_sample_time);
            m_trajectory_splice.points.push_back(p);

            /*printf("%f, %f, %f, %f, %f, %f, %f, %f, %f;\n", p.transforms[0].translation.x, p.transforms[0].translation.y, p.transforms[0].translation.z,
                                                            p.velocities[0].linear.x, p.velocities[0].linear.y, p.velocities[0].linear.z,
                                                            p.accelerations[0].linear.x, p.accelerations[0].linear.y, p.accelerations[0].linear.z);*/
        }

        offset += (pos.size() - 1);
    }


    m_trajectory_publisher.publish(m_trajectory_splice);
}

void planner_class::visualize_path(vector<bspline*> fitted_spline){
    m_vis_array.markers.clear();

    visualization_msgs::Marker m;
    m.header.stamp = ros::Time();
    m.header.frame_id = m_params.odom_frame;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1.0;
    m.color.b = 0.0;
    m.color.g = 0.0;
    m.color.r = 1.0;
    m.color.a = 1.0;
    m.scale.x = 0.05;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.id = 0;

    m.points.clear();

    // Visualize bspline
    //vector<Eigen::VectorXd> spline_points = fitted_spline->get_discrete_bspline(0.1);
    m.id++;

    m.color.r = 0.0;
    m.color.g = 1.0;
    m.points.clear();
    for(auto lo : m_trajectory.points){
        geometry_msgs::Point p;
        p.x = lo.transforms[0].translation.x;
        p.y = lo.transforms[0].translation.y;
        p.z = lo.transforms[0].translation.z;

        m.points.push_back(p);

        // Dump trajectory profile
        // const auto pos = lo.transforms[0].translation;
        // const auto vel = lo.velocities[0].linear;
        // const auto acc = lo.accelerations[0].linear;
        // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f;\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z);
    }

    // Push to topic
    m_vis_array.markers.push_back(m);

    for(auto fit : fitted_spline){
        vector<Eigen::VectorXd> spline_points_opt = fit->get_discrete_bspline(0.1);
        m.id++;

        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        m.scale.x = 0.1;
        m.points.clear();
        for(auto loc : spline_points_opt){
            geometry_msgs::Point p;
            p.x = loc(0);
            p.y = loc(1);
            p.z = loc(2);

            m.points.push_back(p);
        }

        // Push to topic
        m_vis_array.markers.push_back(m);
    }

    // Visualise velocity and acceleration too
    /*bspline vel_bspline = fitted_spline->get_derivative();
    bspline acc_bspline = vel_bspline.get_derivative();
    
    m.id++;

    vector<Eigen::VectorXd> vspline = vel_bspline.get_discrete_bspline(0.1);
    vector<Eigen::VectorXd> aspline = acc_bspline.get_discrete_bspline(0.1);

    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 0.05;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points.clear();    
    for(size_t i = 0; i < spline_points_opt.size(); i+=10){
        auto pos = spline_points_opt[i];
        auto vel = vspline[i];
        auto acc = aspline[i];
        //vel *=10;
        //acc *=10;
        vel+=pos;
        acc+=pos;
        m.id++;
        geometry_msgs::Point pos_p, vel_p, acc_p;
        pos_p.x = pos(0);
        pos_p.y = pos(1);
        pos_p.z = pos(2);

        vel_p.x = vel(0);
        vel_p.y = vel(1);
        vel_p.z = vel(2);

        acc_p.x = acc(0);
        acc_p.y = acc(1);
        acc_p.z = acc(2);

        m.color.r = 1.0;
        m.color.g = 0;
        m.color.b = 1.0;
        m.id++;
        m.points.clear();  
        m.points.push_back(pos_p);
        m.points.push_back(vel_p);
        m_vis_array.markers.push_back(m);

        m.color.r = 0;
        m.color.g = 1.0;
        m.color.b = 1.0;
        m.id++;
        m.points.clear();  
        m.points.push_back(pos_p);
        m.points.push_back(acc_p);
        m_vis_array.markers.push_back(m);

    }*/

    m_vis_publisher.publish(m_vis_array);

}

void planner_class::fsm_tick(const ros::TimerEvent& event){
    // We should just wait for the planner to finish executing an abort event before we replan again
    if(m_block_replan){
        ROS_WARN("Replan blocked!");
        return;
    }

    m_replan_mutex = true;
    // This task should be called once every 50ms or so
    switch(state){
        case STATE_IDLE:
            // Do nothing in idle state
            // State transition is handled in function handle_new_goal
            break;
        case STATE_EXECUTE_MISSION:
            // Check if there is a collision
            double time_to_collision;
            int collision_ind;
            if(check_collision(time_to_collision, collision_ind)){
                // Collision detected
                ROS_INFO("Collision found, recomputing trajectory. TTC: %f, IND: %i", time_to_collision, collision_ind);
                constexpr double REPLAN_LIMIT = 2.0;
                if(time_to_collision > REPLAN_LIMIT){
                    // Run alternative code if there is more than 1 point in the waypoint list
                    if(m_waypoints.size() > 1){
                        int splice_ind = m_current_trajectory_index + ((REPLAN_LIMIT + 0.1) / m_params.mpc_sample_time);
                        vector<bspline*> fitted_splines;
                        vector<bspline*> trajectory_splines;

                        // Get boundary conditions for optimization
                        Eigen::Vector3d here, start_vel;
                        helpers::msg_to_eigen_vector(here, m_trajectory.points[splice_ind].transforms[0].translation);
                        helpers::msg_to_eigen_vector(start_vel, m_trajectory.points[splice_ind].velocities[0].linear);

                        // // Cull any already visited waypoints
                        // for(int i = 0; i < m_waypoint_indexes.size(); ++i){
                        //     int ind = m_waypoint_indexes[i];
                        //     if(splice_ind > ind){
                        //         m_waypoint_indexes.erase(m_waypoint_indexes.begin());
                        //         m_waypoints.erase(m_waypoints.begin());
                        //         i--;
                        //     }
                        // }

                        // // Generate new waypoint
                        // if(!generate_multiwaypoint_path(here, trajectory_splines, fitted_splines)){
                        //     ROS_ERROR("Failed to find any viable paths, aborting!");
                        //     abort_path_following();
                        //     return;
                        // }

                        // Cull any already visited waypoints
                        int start = 0;
                        for(int i = 0; i < m_waypoint_indexes.size(); ++i){
                            int ind = m_waypoint_indexes[i];
                            start++;
                            if(splice_ind < ind){
                                break;
                            }
                        }

                        if(start >= m_waypoint_indexes.size()){
                            ROS_WARN("Start greater than wayponit size!");
                            return;
                        }

                        // Generate new waypoint
                        ROS_INFO("Generating Path");
                        if(!generate_multiwaypoint_path(here, trajectory_splines, fitted_splines, start)){
                            ROS_ERROR("Failed to find any viable paths, aborting!");
                            abort_path_following();
                            return;
                        }
                                    
                        // Optimize
                        m_start_vel = start_vel;
                        if(!generate_multiwaypoint_trajectory(trajectory_splines)){
                            ROS_ERROR("Failed to optimize trajectory, aborting!");
                            abort_path_following();
                            return; 
                        }

                        // Publish and visualize trajectory
                        publish_trajectory_splice(trajectory_splines, splice_ind);
                        visualize_path(fitted_splines);
                    }else{                 
                        // We have enough time to guarantee trajectory velocity continuity
                        // Generate a spliced trajectory starting 2 seconds in future
                        // Setup optimization
                        //bspline* optimized_spline;
                        int splice_ind = m_current_trajectory_index + ((REPLAN_LIMIT + 0.1) / m_params.mpc_sample_time);
                        Eigen::Vector3d here, start_vel, zero, target;
                        target << m_params.target_speed, 0.0, 0.0;
                        zero << 0.0, 0.0, 0.0;
                        helpers::msg_to_eigen_vector(here, m_trajectory.points[splice_ind].transforms[0].translation);
                        helpers::msg_to_eigen_vector(start_vel, m_trajectory.points[splice_ind].velocities[0].linear);

                        //find_trajectory(optimized_spline, here, m_position_goal, start_vel, zero, target);

                        // Find path
                        bool status;
                        auto path = m_planner->find_path(here, m_position_goal, status);

                        // Fit bspline to path
                        m_opt_bspline = bspline_util::fit_bspline(&path, 0.1);
                        auto fitted_spline = bspline_util::fit_bspline(&path, 0.1);

                        if(status && m_opt_bspline != NULL){

                            // Optimize bspline
                            optimize_bspline(m_opt_bspline, start_vel, zero, target);

                            publish_trajectory_splice(splice_ind);

                            visualize_path(fitted_spline, m_opt_bspline);
                        }else{
                            ROS_INFO("Failed to replan, aborting");
                            abort_path_following();

                            // Try to replan the trajectory
                            geometry_msgs::Pose goal;
                            helpers::eigen_vector_to_msg(m_position_goal, goal.position);
                            handle_new_goal(goal);
                        }
                    }
                }else{
                    if(m_waypoints.size() > 1){
                        // Generate disruptive multiwaypoint trajectory

                        // Cull already visited waypoints
                        int splice_ind = m_current_trajectory_index;
                        for(int i = 0; i < m_waypoint_indexes.size(); ++i){
                            int ind = m_waypoint_indexes[i];
                            if(splice_ind > ind){
                                m_waypoint_indexes.erase(m_waypoint_indexes.begin());
                                m_waypoints.erase(m_waypoints.begin());
                                i--;
                            }
                        }

                        // Only abort once the waypoints have been correctly handled
                        // Abort current trajectory immediately
                        auto splice_point = abort_path_following();
                        
                        // Set initial velocity state to 0 because we just aborted
                        m_start_vel << 0, 0, 0;
                        handle_new_waypoints(splice_point);
                    }else{
                        // Abort current trajectory immediately
                        auto splice_point = abort_path_following();

                        // Generate disruptive trajectory
                        geometry_msgs::Pose p;
                        helpers::eigen_vector_to_msg(m_position_goal, p.position);
                        handle_new_goal_from(splice_point, p);
                    }
                }
            }
            break;
    }
    m_replan_mutex = false;
}

void planner_class::replan_tick(const ros::TimerEvent& event){
    ros::Time start = ros::Time::now();
    while(m_replan_mutex){
        ros::Time now = ros::Time::now();
        if(now - start > ros::Duration(0.3)) return;
    }

    ROS_INFO("Replan tick!");
    constexpr double REPLAN_LOOKAHEAD_TIME = 5.0;
    if(state == STATE_EXECUTE_MISSION && m_current_trajectory_index < m_trajectory.points.size()){
        double start_time = (double) m_current_trajectory_index * m_params.mpc_sample_time + REPLAN_LOOKAHEAD_TIME;
        double end_time = (double) m_trajectory.points.size() * m_params.mpc_sample_time;//m_opt_bspline->get_knot_last();

        // There's less than 2.1 seconds left in the trajectory, just continue
        if((end_time - start_time) < 2.0) return;

        //double traj_cost = calc_path_length(m_opt_bspline, start_time, end_time, 0.1);
        double traj_cost = calc_path_length(start_time, end_time, 0.1);

        if(m_waypoints.size() > 1){
            // Multi-waypoint mode
	        ROS_INFO("Multiwaypoint Mode");

            int splice_ind = m_current_trajectory_index + (REPLAN_LOOKAHEAD_TIME / m_params.mpc_sample_time);
            vector<bspline*> fitted_splines;
            vector<bspline*> trajectory_splines;

            // Get boundary conditions for optimization
            Eigen::Vector3d here, start_vel;
            helpers::msg_to_eigen_vector(here, m_trajectory.points[splice_ind].transforms[0].translation);
            helpers::msg_to_eigen_vector(start_vel, m_trajectory.points[splice_ind].velocities[0].linear);

            // Cull any already visited waypoints
	        ROS_INFO("Getting splice index");
	        int start = 0;
            for(int i = 0; i < m_waypoint_indexes.size(); ++i){
                int ind = m_waypoint_indexes[i];
                start++;
                if(splice_ind < ind){
                        //m_waypoint_indexes.erase(m_waypoint_indexes.begin());
                        //m_waypoints.erase(m_waypoints.begin());
                        //i--;
                    break;
                }
            }

            if(start >= m_waypoint_indexes.size()){
                ROS_WARN("Start greater than wayponit size!");
                return;
            }

            // Do not abort in the replan tick, just fail
            // Generate new waypoint
	        ROS_INFO("Generating Path");
            if(!generate_multiwaypoint_path(here, trajectory_splines, fitted_splines, start)){
                ROS_ERROR("Failed to find any viable paths, aborting!");
                //abort_path_following();
                return;
            }
            
            // Optimize
            m_start_vel = start_vel;
	        ROS_INFO("Optimizing...");
            if(!generate_multiwaypoint_trajectory(trajectory_splines)){
                ROS_ERROR("Failed to optimize trajectory, aborting!");
                //abort_path_following();
                return;    
            }

            double end = trajectory_splines[trajectory_splines.size() - 1]->get_knot_last();
            double optimized_traj_cost = calc_path_length(trajectory_splines, 0, end, 0.1);

            if(optimized_traj_cost < (traj_cost * 0.9)){
                // Publish and visualize trajectory
                ROS_INFO("Splicing lower cost trajectory, original cost: %f, new cost: %f", traj_cost, optimized_traj_cost);
                publish_trajectory_splice(trajectory_splines, splice_ind);
                visualize_path(fitted_splines);
            }
        }else{
            // Set up new optimization
	        ROS_INFO("Single waypoint!??");
            int splice_ind = m_current_trajectory_index + (REPLAN_LOOKAHEAD_TIME/ m_params.mpc_sample_time);
            Eigen::Vector3d here, start_vel, zero, target;
            target << m_params.target_speed, 0.0, 0.0;
            zero << 0.0, 0.0, 0.0;
            helpers::msg_to_eigen_vector(here, m_trajectory.points[splice_ind].transforms[0].translation);
            helpers::msg_to_eigen_vector(start_vel, m_trajectory.points[splice_ind].velocities[0].linear);

            bool status;
            auto path = m_planner->find_path(here, m_position_goal, status);
            
            // Immediately fail if no path is found
            if(!status){
                ROS_WARN("No path found for single waypoint replan");
                return;
            }

            // Fit bspline to path
            auto opt_spline = bspline_util::fit_bspline(&path, 0.1);
            auto fitted_spline = bspline_util::fit_bspline(&path, 0.1);
            
            // We can't optimize a broken bspline
            if(!status || opt_spline == NULL){
                // Fail if the bspline creation fails
                return;
                // Try to recover by generating from current pose instead
                //tf::StampedTransform uav_loc;
                //get_current_position(uav_loc);
                //helpers::tf_to_eigen_vector(here, uav_loc);

                //auto path = m_planner->find_path(here, m_position_goal, status);

                // Fit bspline to path
                //opt_spline = bspline_util::fit_bspline(&path, 0.1);
                //fitted_spline = bspline_util::fit_bspline(&path, 0.1);

                //if(!status || opt_spline == NULL){
                    // We are actually stuck
                    // TODO: Figure out recovery here
                    //return;
                //}
            }

            optimize_bspline(opt_spline, start_vel, zero, target);

            double optimized_traj_cost = calc_path_length(opt_spline, 0, opt_spline->get_knot_last(), 0.1);

            // Only splice if the new optimized trajectory is more than 10% better
            if(optimized_traj_cost < (traj_cost * 0.9)){
                ROS_INFO("Splicing lower cost trajectory, original cost: %f, new cost: %f", traj_cost, optimized_traj_cost);
                //delete m_opt_bspline;

                *m_opt_bspline = *opt_spline;

                publish_trajectory_splice(splice_ind);

                visualize_path(fitted_spline, m_opt_bspline);
            }

            //delete fitted_spline;
        }
    }
}

double planner_class::calc_path_length(bspline* spline, double start, double end, double step){
    double cost = 0;
    auto point = spline->get_discrete_bspline_point(start);
    for(double t = start + step; t < end; t += step){
        auto point1 = spline->get_discrete_bspline_point(t);
        cost += (point - point1).norm();
        point = point1;
    }

    return cost;
}

// This is megabugged for some reason
double planner_class::calc_path_length(double start, double end, double step){
    double cost = 0;
    Eigen::Vector3d point;
    helpers::msg_to_eigen_vector(point, m_trajectory.points[start / m_params.mpc_sample_time].transforms[0].translation);
    for(int ind = start / m_params.mpc_sample_time; ind < m_trajectory.points.size(); ind++){
        Eigen::Vector3d point1;
        helpers::msg_to_eigen_vector(point1, m_trajectory.points[ind].transforms[0].translation);
        cost += (point - point1).norm();
        point = point1;
    }
    // for(double t = start + step; t < end; t += step){
    //     Eigen::Vector3d point1;
    //     helpers::msg_to_eigen_vector(point, m_trajectory.points[t / m_params.mpc_sample_time].transforms[0].translation);
    //     cost += (point - point1).squaredNorm();
    //     point = point1;
    // }

    return cost;
}

double planner_class::calc_path_length(vector<bspline*> splines, double start, double end, double step){
    double cost = 0;
    for(auto spline : splines){
        auto point = spline->get_discrete_bspline_point(start);
        for(double t = start + step; t < end; t += step){
            auto point1 = spline->get_discrete_bspline_point(t);
            cost += (point - point1).norm();
            point = point1;
        }
    }

    return cost;
}

// NOTE: time_to_collision will be in seconds, and will only be valid if function returns true.
bool planner_class::check_collision(double& time_to_collision, int& collision_ind){
    // Catch invalid trajectories
    if(m_trajectory.points.size() < 2){
        return false;
    }

    constexpr double COLLISION_DEADBAND = 0.2;
    Eigen::Vector3d start;
    if(m_current_trajectory_index < m_trajectory.points.size())
        helpers::msg_to_eigen_vector(start, m_trajectory.points[m_current_trajectory_index].transforms[0].translation);

    for(size_t i = m_current_trajectory_index; i < m_trajectory.points.size(); ++i){
        Eigen::Vector3d here;
        helpers::msg_to_eigen_vector(here, m_trajectory.points[i].transforms[0].translation);

        // Skip if the collision is within the deadband so the UAV can escape bad positions (slowly)
        double dist = (here - start).norm();
        if(dist < COLLISION_DEADBAND){
            //ROS_INFO("Skipping collision check at %i, dist: %f", i, dist);
            continue;
        }

        if(m_esdf_pointer->GetDistance(here) < m_params.clear_radius){
            time_to_collision = ((double)(i - m_current_trajectory_index)) * m_params.mpc_sample_time;
            collision_ind = i;
            return true;
        }
    }
    return false;

}

void planner_class::find_trajectory(bspline* spline, Eigen::Vector3d here, Eigen::Vector3d goal, Eigen::Vector3d start_vel, Eigen::Vector3d end_vel, Eigen::Vector3d target_vel){
    // Use path planner to find a frontend path
    //auto path = m_planner->find_path(here, goal);

    // Fit bspline to path
    //spline = bspline_util::fit_bspline(&path, 0.1);
    //auto fitted_spline = spline;

    // Optimize bspline
    //optimize_bspline(spline, start_vel, end_vel, target_vel);

    //visualize_path(fitted_spline, spline);
}

void planner_class::handle_new_nav_goal(geometry_msgs::PoseStamped p){
    // Abort trajectory to test "replan"
    auto here = abort_path_following();

    auto pose = p.pose;
    pose.position.z = 2.5;

    handle_new_goal_from(here, pose);
}

Eigen::Vector3d planner_class::abort_path_following(){
    // Check if the planner is idle or not
    constexpr int NUM_ABORT_LOOKAHEAD = 2;
    int splice_ind = m_current_trajectory_index + NUM_ABORT_LOOKAHEAD;

    if(splice_ind < m_trajectory.points.size() && m_params.use_soft_abort){
        do{
            // Do a soft stop
            ROS_WARN("Executing Soft Abort!");

            // Get boundary conditions for optimization
            Eigen::Vector3d here, start_vel, zero, target;
            target << m_params.target_speed, 0.0, 0.0;
            zero << 0.0, 0.0, 0.0;
            helpers::msg_to_eigen_vector(here, m_trajectory.points[splice_ind].transforms[0].translation);
            helpers::msg_to_eigen_vector(start_vel, m_trajectory.points[splice_ind].velocities[0].linear);

            // Define the number of bspline points to insert
            constexpr double DECELERATION_RATE = 2; // m/s2
            int size = (int)(start_vel.norm() / DECELERATION_RATE / m_params.mpc_sample_time);

            // Seed n control points based on the deceleration rate
            vector<Eigen::Vector3d> stop_seed;
            auto start = here;
            auto diff = start_vel.normalized();
            for(int i = 0; i < size/2; ++i){
                stop_seed.push_back(here);
                // Increment here slightly
                here += diff * 0.2;
            }
            for(int i = 0; i < size/2; ++i){
                stop_seed.push_back(here);
                // Increment here slightly
                here-= diff * 0.2;
            }
            stop_seed.push_back(start);

            ROS_INFO("Seeded stop traj");
            
            // Fit spline
            m_opt_bspline = bspline_util::fit_bspline(&stop_seed, 0.1);
            auto fitted_spline = bspline_util::fit_bspline(&stop_seed, 0.1);

            // Don't bother compensating for yaw during this optimization
            auto backup = m_opt_params;
            m_opt_params.hfov_enabled = false;
            m_opt_params.vfov_enabled = false;

            // Fallback to a hard stop if we fail the optimization
            bool optstatus = optimize_bspline(m_opt_bspline, start_vel, zero, target);

            // Restore optimization parameters
            m_opt_params = backup;

            if(!optstatus){
                break;
            }

            ROS_INFO("Optimized stop traj");

            ROS_INFO("Assign?");
            // Set flags for noyaw and mandatory
            m_trajectory_splice.joint_names.clear();
            m_trajectory_splice.joint_names.push_back(string("3"));

            ROS_INFO("Pub");
            publish_trajectory_splice(splice_ind);
            ROS_INFO("Viz");
            visualize_path(fitted_spline, m_opt_bspline);

            // Clear flags
            m_trajectory_splice.joint_names.clear();

            // Set replan flag to block until follower finishes executing replan
            m_block_replan = true;

            return start;
        }while(false);
    }
    
    tf::StampedTransform odom_to_fcu;
    get_current_position(odom_to_fcu);

    Eigen::Vector3d here;
    helpers::tf_to_eigen_vector(here, odom_to_fcu);

    // Empty the trajectory
    m_trajectory.points.clear();
    m_trajectory.header.frame_id = m_params.odom_frame;
    m_trajectory.header.seq++;
    m_trajectory.header.stamp = ros::Time::now();

    geometry_msgs::Transform pos;
    geometry_msgs::Twist vel, acc;

    pos.translation.x = odom_to_fcu.getOrigin().x();
    pos.translation.y = odom_to_fcu.getOrigin().y();
    pos.translation.z = odom_to_fcu.getOrigin().z();

    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;

    acc.linear.x = 0;
    acc.linear.y = 0;
    acc.linear.z = 0;

    trajectory_msgs::MultiDOFJointTrajectoryPoint p;
    p.time_from_start = ros::Duration(0.0);
    p.transforms.push_back(pos);
    p.velocities.push_back(vel);
    p.accelerations.push_back(acc);

    m_trajectory.points.push_back(p);

    m_trajectory_publisher.publish(m_trajectory);
    
    return here;
}

void planner_class::get_current_position(tf::StampedTransform &tf){
    try{
        auto now = ros::Time::now();
        m_tf_listener->waitForTransform(m_params.odom_frame, m_params.fcu_frame, now, ros::Duration(0.5));
        m_tf_listener->lookupTransform(m_params.odom_frame, m_params.fcu_frame, now, tf);
    }catch(tf::TransformException ex){
        ROS_ERROR("Failed to get odom to body tf: %s", ex.what());
        return;
    }
}

void planner_class::handle_new_goal(geometry_msgs::Pose p){
    tf::StampedTransform odom_to_fcu;
    get_current_position(odom_to_fcu);
    Eigen::Vector3d here;
    helpers::tf_to_eigen_vector(here, odom_to_fcu);

    handle_new_goal_from(here, p);
}

void planner_class::handle_new_goal_from(Eigen::Vector3d start, geometry_msgs::Pose p){
    // Clear waypoints and setup for single waypoint mode
    m_waypoints.clear();
    m_waypoint_indexes.clear();
    Eigen::Vector3d goal, here;
    here = start;
    helpers::msg_to_eigen_vector(goal, p.position);
    m_position_goal = goal;

    // Setup optimization
    bspline* optimized_spline;
    Eigen::Vector3d zero, target;
    // TODO: Figure out what to do with zero and target
    zero << 0.0, 0.0, 0.0;
    target << m_params.target_speed, 0.0, 0.0;

    // Find path
    bool status;
    auto path = m_planner->find_path(here, goal, status);

    if(!status){
        abort_path_following();
        return;
    }

    // Fit bspline to path
    m_opt_bspline = bspline_util::fit_bspline(&path, 0.1);
    auto fitted_spline = bspline_util::fit_bspline(&path, 0.1);

    // No valid trajectory has been generated. Fail gracefully
    if(m_opt_bspline == NULL){
        abort_path_following();
        return;
    }

    // Optimize bspline
    optimize_bspline(m_opt_bspline, zero, zero, target);

    // Clean up to avoid memory leaking
    //delete fitted_spline;
    //delete m_opt_bspline;

    // Set state to path following
    state = STATE_EXECUTE_MISSION;

    publish_trajectory();

    visualize_path(fitted_spline, m_opt_bspline);

    delete fitted_spline;
}

void planner_class::handle_new_trajectory_index(const std_msgs::Int32 curr){
    m_current_trajectory_index = curr.data;
}

void planner_class::publish_trajectory(){
    auto vel_spline = m_opt_bspline->get_derivative();
    auto acc_spline = vel_spline.get_derivative();

    auto pos = m_opt_bspline->get_discrete_bspline(m_params.mpc_sample_time);
    auto vel = vel_spline.get_discrete_bspline(m_params.mpc_sample_time);
    auto acc = acc_spline.get_discrete_bspline(m_params.mpc_sample_time);

    m_trajectory.points.clear();
    m_trajectory.header.frame_id = m_params.odom_frame;
    m_trajectory.header.seq++;
    m_trajectory.header.stamp = ros::Time::now();

    // Pad the start of the trajectory with 0.5 seconds of repeating starting points
    // Hopefully this gives the controller a chance to never fall behind
    for(size_t i = 0; i < 5; ++i){
        geometry_msgs::Transform pos_msg;
        geometry_msgs::Twist vel_msg, acc_msg;

        helpers::eigen_vector_to_msg(pos[0], pos_msg.translation);
        helpers::eigen_vector_to_msg(vel[0], vel_msg.linear);
        helpers::eigen_vector_to_msg(acc[0], acc_msg.linear);

        trajectory_msgs::MultiDOFJointTrajectoryPoint p;
        p.time_from_start = ros::Duration((double) i * 0.1);
        p.accelerations.push_back(acc_msg);
        p.velocities.push_back(vel_msg);
        p.transforms.push_back(pos_msg);

        m_trajectory.points.push_back(p);
    }

    // Publish the trajectory
    for(size_t i=0; i < pos.size(); ++i){
        geometry_msgs::Transform pos_msg;
        geometry_msgs::Twist vel_msg, acc_msg;

        helpers::eigen_vector_to_msg(pos[i], pos_msg.translation);
        helpers::eigen_vector_to_msg(vel[i], vel_msg.linear);
        helpers::eigen_vector_to_msg(acc[i], acc_msg.linear);

        trajectory_msgs::MultiDOFJointTrajectoryPoint p;
        p.time_from_start = ros::Duration(((double) i) * m_params.mpc_sample_time + 0.5);
        p.accelerations.push_back(acc_msg);
        p.velocities.push_back(vel_msg);
        p.transforms.push_back(pos_msg);

        m_trajectory.points.push_back(p);
    }

    m_trajectory_publisher.publish(m_trajectory);
}

void planner_class::publish_trajectory_splice(int splice_index){
    auto vel_spline = m_opt_bspline->get_derivative();
    auto acc_spline = vel_spline.get_derivative();

    auto pos = m_opt_bspline->get_discrete_bspline(m_params.mpc_sample_time);
    auto vel = vel_spline.get_discrete_bspline(m_params.mpc_sample_time);
    auto acc = acc_spline.get_discrete_bspline(m_params.mpc_sample_time);

    m_trajectory.header.frame_id = m_params.odom_frame;
    m_trajectory.header.seq++;
    m_trajectory.header.stamp = ros::Time::now();
    // Resize the trajectory to the splice size
    m_trajectory.points.resize(splice_index + 1);

    // TODO: Make this more elegant
    m_trajectory_splice.points.clear();
    m_trajectory_splice.header.frame_id = m_params.odom_frame;
    m_trajectory_splice.header.seq++;
    m_trajectory_splice.header.stamp = ros::Time::now();

    // Publish the trajectory
    for(size_t i=0; i < pos.size(); ++i){
        geometry_msgs::Transform pos_msg;
        geometry_msgs::Twist vel_msg, acc_msg;

        helpers::eigen_vector_to_msg(pos[i], pos_msg.translation);
        helpers::eigen_vector_to_msg(vel[i], vel_msg.linear);
        helpers::eigen_vector_to_msg(acc[i], acc_msg.linear);

        trajectory_msgs::MultiDOFJointTrajectoryPoint p;
        p.time_from_start = ros::Duration(((double) i) * m_params.mpc_sample_time);
        p.accelerations.push_back(acc_msg);
        p.velocities.push_back(vel_msg);
        p.transforms.push_back(pos_msg);

        m_trajectory.points.push_back(p);

        p.time_from_start += ros::Duration(((double)splice_index) * m_params.mpc_sample_time);
        m_trajectory_splice.points.push_back(p);

        printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", p.transforms[0].translation.x, p.transforms[0].translation.y, p.transforms[0].translation.z,
                                                        p.velocities[0].linear.x, p.velocities[0].linear.y, p.velocities[0].linear.z,
                                                        p.accelerations[0].linear.x, p.accelerations[0].linear.y, p.accelerations[0].linear.z);
    }


    m_trajectory_publisher.publish(m_trajectory_splice);
}

bool planner_class::optimize_bspline(bspline* spline, Eigen::Vector3d start_vel, Eigen::Vector3d end_vel, Eigen::Vector3d target_vel){
    // Optimizer object should automatically be deleted when this falls out of scope
    
    auto opt = std::make_unique<bspline_opt>(spline, m_fiesta_pointer->GetESDFMapPointer(), m_opt_params);
    //target_vel.normalize();
    opt->set_target_velocity_vector(target_vel);
    opt->set_termination_velocity_states(start_vel, end_vel);
    switch(opt->optimize()){
        case OPT_OK:
            // Do nothing if the optimization succceeds
            return true;
            break;
        case OPT_ERROR_BSPLINE_TOOSHORT:
            ROS_ERROR("Optimization failed: BSpline too short!");
            break;
        case OPT_ERROR_NO_CONVERGE:
            ROS_ERROR("Optimization failed: Optimization did not converge!");
            break;
        case OPT_ERROR_FAILED:
            ROS_ERROR("Optimization failed: Unknown error!");
            break;
        case OPT_ERROR_NULLPTR:
            ROS_ERROR("Optimization failed: BSpline pointer is NULL!");
            break;
        case OPT_ERROR_TARGET_VEL_ZERO:
            ROS_ERROR("Optimization failed: Target speed is zero!");
            break;
    }
    return false;
}

void planner_class::visualize_path(bspline* fitted_spline, bspline* optimized_spline){
    m_vis_array.markers.clear();

    visualization_msgs::Marker m;
    m.header.stamp = ros::Time();
    m.header.frame_id = m_params.odom_frame;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1.0;
    m.color.b = 0.0;
    m.color.g = 0.0;
    m.color.r = 1.0;
    m.color.a = 1.0;
    m.scale.x = 0.05;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.id = 0;

    m.points.clear();

    // Visualize bspline
    //vector<Eigen::VectorXd> spline_points = fitted_spline->get_discrete_bspline(0.1);
    m.id++;

    m.color.r = 0.0;
    m.color.g = 1.0;
    m.points.clear();
    for(auto lo : m_trajectory.points){
        geometry_msgs::Point p;
        p.x = lo.transforms[0].translation.x;
        p.y = lo.transforms[0].translation.y;
        p.z = lo.transforms[0].translation.z;

        m.points.push_back(p);
    }

    // Push to topic
    m_vis_array.markers.push_back(m);

    vector<Eigen::VectorXd> spline_points_opt = fitted_spline->get_discrete_bspline(0.1);
    m.id++;

    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.scale.x = 0.1;
    m.points.clear();
    for(auto loc : spline_points_opt){
        geometry_msgs::Point p;
        p.x = loc(0);
        p.y = loc(1);
        p.z = loc(2);

        m.points.push_back(p);
    }

    // Push to topic
    m_vis_array.markers.push_back(m);

    // Visualise velocity and acceleration too
    /*bspline vel_bspline = fitted_spline->get_derivative();
    bspline acc_bspline = vel_bspline.get_derivative();
    
    m.id++;

    vector<Eigen::VectorXd> vspline = vel_bspline.get_discrete_bspline(0.1);
    vector<Eigen::VectorXd> aspline = acc_bspline.get_discrete_bspline(0.1);

    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 0.05;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points.clear();    
    for(size_t i = 0; i < spline_points_opt.size(); i+=10){
        auto pos = spline_points_opt[i];
        auto vel = vspline[i];
        auto acc = aspline[i];
        //vel *=10;
        //acc *=10;
        vel+=pos;
        acc+=pos;
        m.id++;
        geometry_msgs::Point pos_p, vel_p, acc_p;
        pos_p.x = pos(0);
        pos_p.y = pos(1);
        pos_p.z = pos(2);

        vel_p.x = vel(0);
        vel_p.y = vel(1);
        vel_p.z = vel(2);

        acc_p.x = acc(0);
        acc_p.y = acc(1);
        acc_p.z = acc(2);

        m.color.r = 1.0;
        m.color.g = 0;
        m.color.b = 1.0;
        m.id++;
        m.points.clear();  
        m.points.push_back(pos_p);
        m.points.push_back(vel_p);
        m_vis_array.markers.push_back(m);

        m.color.r = 0;
        m.color.g = 1.0;
        m.color.b = 1.0;
        m.id++;
        m.points.clear();  
        m.points.push_back(pos_p);
        m.points.push_back(acc_p);
        m_vis_array.markers.push_back(m);

    }*/

    m_vis_publisher.publish(m_vis_array);

}

void planner_class::handle_stop(std_msgs::Int32 unused){
    abort_path_following();
}

void planner_class::handle_abort_finished(std_msgs::Int32 unused){
    m_block_replan = false;
}

void planner_class::init_subscribers(ros::NodeHandle n, ros::NodeHandle pn){
    // Init goal subscriber
    m_goal_subscriber = n.subscribe(m_params.goal_topic, 1, &planner_class::handle_new_goal, this);
    m_trajectory_index_subscriber = n.subscribe(m_params.trajectory_index_topic, 5, &planner_class::handle_new_trajectory_index, this);
    m_nav_goal_subscriber = n.subscribe("move_base_simple/goal", 1, &planner_class::handle_new_nav_goal, this);
    m_stop_subscriber = n.subscribe("stop", 1, &planner_class::handle_stop, this);
    m_abort_finished_subscriber = n.subscribe("planner/abort_finished", 1, &planner_class::handle_abort_finished, this);
}

void planner_class::init_publishers(ros::NodeHandle n, ros::NodeHandle pn){
    // Init viz publisher
    m_vis_publisher = n.advertise<visualization_msgs::MarkerArray>("planner/vis", 10);
    m_trajectory_publisher = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(m_params.trajectory_topic, 10);

    m_heartbeat_service = n.advertiseService("traj_follower_heartbeat", &planner_class::handle_heartbeat, this);
    m_trajectory_service = n.advertiseService("planner/new_goal", &planner_class::handle_new_trajectory_request, this);
    m_trajectory_test_service = n.advertiseService("/fast_planner/planner/new_goal_test", &planner_class::handle_new_trajectory_test_request, this);
    m_esdf_lookup_service = n.advertiseService("esdf_lookup", &planner_class::handle_esdf_lookup, this);
}

