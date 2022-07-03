#include <ros/ros.h>
#include "bspline.h"
#include "bspline_opt.h"
#include "ESDFMap.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "bspline_test_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>("/bspline_vis", 5);

    ros::spinOnce();
    
    vector<Eigen::Vector3d> points, points1;
    Eigen::Vector3d p;

    /*p << 4.700000, 4.300000, 2.300000; points.push_back(p);
    p << 4.900000, 4.500000, 2.100000; points.push_back(p);
    p << 5.100000, 4.700000, 2.100000; points.push_back(p);
    p << 5.300000, 4.700000, 2.100000; points.push_back(p);
    p << 5.500000, 4.700000, 2.100000; points.push_back(p);
    p << 5.700000, 4.900000, 2.100000; points.push_back(p);
    p << 5.900000, 5.100000, 2.100000; points.push_back(p);
    p << 6.100000, 5.300000, 2.100000; points.push_back(p);
    p << 6.300000, 5.500000, 2.100000; points.push_back(p);
    p << 6.500000, 5.700000, 2.100000; points.push_back(p);
    p << 6.700000, 5.900000, 2.100000; points.push_back(p);
    p << 6.900000, 6.100000, 2.100000; points.push_back(p);                    
    p << 7.100000, 6.300000, 2.100000; points.push_back(p);                             
    p << 7.300000, 6.500000, 2.100000; points.push_back(p); 
    p << 7.500000, 6.700000, 2.100000; points.push_back(p);
    p << 7.700000, 6.900000, 2.100000; points.push_back(p);                    
    p << 7.900000, 7.100000, 2.100000; points.push_back(p);                                
    p << 8.100000, 7.300000, 2.100000; points.push_back(p); 
    p << 8.300000, 7.500000, 2.100000; points.push_back(p);
    p << 8.500000, 7.700000, 2.100000; points.push_back(p);                    
    p << 8.700000, 7.900000, 2.100000; points.push_back(p);                               
    p << 8.900000, 8.100000, 2.100000; points.push_back(p); 
    p << 9.100000, 8.300000, 2.100000; points.push_back(p);
    p << 9.300000, 8.500000, 2.100000; points.push_back(p);                    
    p << 9.500000, 8.700000, 2.100000; points.push_back(p);                              
    p << 9.500000, 8.900000, 2.100000; points.push_back(p); 
    p << 9.500000, 9.100000, 2.100000; points.push_back(p);
    p << 9.700000, 9.300000, 2.100000; points.push_back(p);                    
    p << 9.900000, 9.500000, 2.100000; points.push_back(p);                                 
    p << 9.900000, 9.700000, 2.100000; points.push_back(p);   
    p << 9.900000, 9.900000, 2.100000; points.push_back(p);
    p << 10.100000, 10.100000, 2.100000;
    points.push_back(p);*/

    p << 0, 0, 2.5; points.push_back(p); points1.push_back(p);
    p << 0.100000, 0.100000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 0.300000, 0.300000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 0.500000, 0.500000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 0.700000, 0.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 0.900000, 0.900000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 1.100000, 1.100000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 1.300000, 1.300000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 1.500000, 1.500000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 1.700000, 1.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 1.900000, 1.900000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 2.100000, 2.100000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 2.300000, 2.300000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 2.500000, 2.500000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 2.700000, 2.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 2.900000, 2.900000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 3.100000, 3.100000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 3.300000, 3.300000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 3.500000, 3.500000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 3.700000, 3.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 3.900000, 3.900000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 4.100000, 4.100000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 4.300000, 4.300000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 4.500000, 4.500000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 4.700000, 4.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 4.900000, 4.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 5.100000, 4.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 5.300000, 4.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 5.500000, 4.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 5.700000, 4.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 5.900000, 4.900000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 6.100000, 5.100000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 6.300000, 5.300000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 6.500000, 5.500000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 6.700000, 5.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 6.900000, 5.900000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 7.100000, 6.100000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 7.300000, 6.300000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 7.500000, 6.500000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 7.700000, 6.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 7.900000, 6.900000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 8.100000, 7.100000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 8.300000, 7.300000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 8.500000, 7.500000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 8.700000, 7.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 8.900000, 7.900000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 9.100000, 8.100000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 9.300000, 8.300000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 9.500000, 8.500000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 9.700000, 8.700000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 9.900000, 8.900000, 2.500000; points.push_back(p); points1.push_back(p);
    p << 10.100000, 9.100000, 2.50000; points.push_back(p); points1.push_back(p);

    // p << 1.146972, 1.390348, 2.762377; points.push_back(p);
    // p << 1.300000, 1.500000, 2.900000; points.push_back(p);
    // p << 1.500000, 1.700000, 3.100000; points.push_back(p);
    // p << 1.700000, 1.900000, 3.300000; points.push_back(p);
    // p << 1.900000, 2.100000, 3.500000; points.push_back(p);
    // p << 2.100000, 2.300000, 3.700000; points.push_back(p);
    // p << 2.300000, 2.500000, 3.700000; points.push_back(p);
    // p << 2.500000, 2.700000, 3.700000; points.push_back(p);
    // p << 2.700000, 2.900000, 3.700000; points.push_back(p);
    // p << 2.900000, 3.100000, 3.700000; points.push_back(p);
    // p << 3.100000, 3.300000, 3.700000; points.push_back(p);
    // p << 3.300000, 3.500000, 3.700000; points.push_back(p);
    // p << 3.500000, 3.700000, 3.700000; points.push_back(p);
    // p << 3.700000, 3.900000, 3.700000; points.push_back(p);
    // p << 3.900000, 4.100000, 3.700000; points.push_back(p);
    // p << 4.100000, 4.300000, 3.700000; points.push_back(p);
    // p << 4.300000, 4.500000, 3.700000; points.push_back(p);
    // p << 4.500000, 4.700000, 3.700000; points.push_back(p);
    // p << 4.700000, 4.900000, 3.700000; points.push_back(p);
    // p << 4.900000, 5.100000, 3.700000; points.push_back(p);
    // p << 5.100000, 5.300000, 3.700000; points.push_back(p);
    // p << 5.300000, 5.500000, 3.700000; points.push_back(p);
    // p << 5.500000, 5.700000, 3.700000; points.push_back(p);
    // p << 5.700000, 5.900000, 3.700000; points.push_back(p);
    // p << 5.900000, 6.100000, 3.700000; points.push_back(p);
    // p << 6.100000, 6.300000, 3.700000; points.push_back(p);
    // p << 6.300000, 6.500000, 3.700000; points.push_back(p);
    // p << 6.500000, 6.700000, 3.700000; points.push_back(p);
    // p << 6.700000, 6.900000, 3.700000; points.push_back(p);
    // p << 6.900000, 7.100000, 3.700000; points.push_back(p);
    // p << 7.100000, 7.300000, 3.700000; points.push_back(p);
    // p << 7.300000, 7.500000, 3.700000; points.push_back(p);
    // p << 7.500000, 7.700000, 3.700000; points.push_back(p);
    // p << 7.700000, 7.900000, 3.700000; points.push_back(p);
    // p << 7.900000, 8.100000, 3.700000; points.push_back(p);
    // p << 8.100000, 8.300000, 3.700000; points.push_back(p);
    // p << 8.300000, 8.500000, 3.700000; points.push_back(p);
    // p << 8.500000, 8.700000, 3.700000; points.push_back(p);
    // p << 8.700000, 8.900000, 3.700000; points.push_back(p);
    // p << 8.900000, 9.100000, 3.700000; points.push_back(p);
    // p << 9.100000, 9.300000, 3.700000; points.push_back(p);
    // p << 9.300000, 9.500000, 3.700000; points.push_back(p);
    // p << 9.500000, 9.700000, 3.700000; points.push_back(p);
    // p << 9.700000, 9.900000, 3.700000; points.push_back(p);
    // p << 9.900000, 10.100000, 3.700000; points.push_back(p);
    // p << 10.100000, 10.300000, 3.700000; points.push_back(p);
    // p << 10.300000, 10.500000, 3.700000; points.push_back(p);
    // p << 10.500000, 10.700000, 3.700000; points.push_back(p);
    // p << 10.700000, 10.900000, 3.700000; points.push_back(p);
    // p << 10.900000, 11.100000, 3.700000; points.push_back(p);
    // p << 11.100000, 11.300000, 3.700000; points.push_back(p);
    // p << 11.300000, 11.500000, 3.700000; points.push_back(p);
    // p << 11.500000, 11.700000, 3.700000; points.push_back(p);
    // p << 11.700000, 11.900000, 3.700000; points.push_back(p);
    // p << 11.900000, 12.100000, 3.700000; points.push_back(p);
    // p << 12.100000, 12.300000, 3.700000; points.push_back(p);
    // p << 12.300000, 12.500000, 3.700000; points.push_back(p);
    // p << 12.500000, 12.700000, 3.700000; points.push_back(p);
    // p << 12.700000, 12.900000, 3.700000; points.push_back(p);
    // p << 12.900000, 13.100000, 3.700000; points.push_back(p);
    // p << 13.100000, 13.300000, 3.700000; points.push_back(p);
    // p << 13.300000, 13.500000, 3.700000; points.push_back(p);
    // p << 13.500000, 13.700000, 3.700000; points.push_back(p);
    // p << 13.700000, 13.900000, 3.700000; points.push_back(p);
    // p << 13.900000, 14.100000, 3.700000; points.push_back(p);
    // p << 14.100000, 14.300000, 3.700000; points.push_back(p);
    // p << 14.300000, 14.500000, 3.700000; points.push_back(p);
    // p << 14.500000, 14.700000, 3.700000; points.push_back(p);
    // p << 14.700000, 14.900000, 3.700000; points.push_back(p);
    // p << 14.900000, 15.100000, 3.700000; points.push_back(p);
    // p << 15.100000, 15.300000, 3.700000; points.push_back(p);
    // p << 15.300000, 15.500000, 3.700000; points.push_back(p);
    // p << 15.500000, 15.700000, 3.700000; points.push_back(p);
    // p << 15.700000, 15.900000, 3.700000; points.push_back(p);
    // p << 15.900000, 16.100000, 3.700000; points.push_back(p);
    // p << 16.100000, 16.300000, 3.700000; points.push_back(p);
    // p << 16.300000, 16.500000, 3.700000; points.push_back(p);
    // p << 16.500000, 16.700000, 3.700000; points.push_back(p);
    // p << 16.700000, 16.900000, 3.700000; points.push_back(p);
    // p << 16.900000, 17.100000, 3.700000; points.push_back(p);
    // p << 17.100000, 17.300000, 3.700000; points.push_back(p);
    // p << 17.300000, 17.500000, 3.700000; points.push_back(p);
    // p << 17.500000, 17.700000, 3.700000; points.push_back(p);
    // p << 17.700000, 17.700000, 3.700000; points.push_back(p);
    // p << 17.900000, 17.900000, 3.700000; points.push_back(p);
    // p << 18.100000, 17.900000, 3.700000; points.push_back(p);
    // p << 18.300000, 17.900000, 3.700000; points.push_back(p);
    // p << 18.500000, 17.900000, 3.700000; points.push_back(p);
    // p << 18.700000, 17.900000, 3.700000; points.push_back(p);
    // p << 18.900000, 17.900000, 3.700000; points.push_back(p);
    // p << 19.100000, 17.900000, 3.700000; points.push_back(p);
    // p << 19.300000, 18.100000, 3.900000; points.push_back(p);



    //Eigen::MatrixXd test_control_points(points.size(), 4);
    //for(size_t i=0; i < points.size(); ++i) test_control_points.row(i) = points[i];
    vector<bspline*> splines;
    bspline* b = bspline_util::fit_bspline(&points, 0.1);
    splines.push_back(b);

    for(size_t i = 0; i < points.size(); ++i){
        points1[i](0) = points1[points.size() - i - 1](0);
        points1[i](1) = points1[points.size() - i - 1](1);
        points1[i](2) += (double) i * 0.1;
    }

    Eigen::Vector3d diff(10.100000, 5.100000, 0.000000);
    // for(size_t i = 0; i < points1.size(); ++i){
    //     points1[i] += diff;
    // }
    //points1.push_back(Eigen::Vector3d(5, -5, 5));
    bspline* b1 = bspline_util::fit_bspline(&points1, 0.1);
    splines.push_back(b1);

    //bspline b(test_control_points, 4, 0.1);
    vector<Eigen::VectorXd> discrete_points;
    for(auto bs : splines){
        auto d = bs->get_discrete_bspline(0.1);
        for(auto dd : d){
            discrete_points.push_back(dd);
        }
    }

    Eigen::Vector3d origin;
    origin << 0, 0, 0;
    fiesta::ESDFMap map(origin, 0.2, 0);
    bspline_opt_params param;
    param.collision_weight = 1.0;
    param.collision_min_distance = 1.0;
    param.acceleration_weight << 5.0, 5.0, 5.0;
    param.max_acc_weight << 50, 50, 50;
    param.max_acc << 100.0, 100.0, 100.0;
    param.start_end_position_weight << 15.0, 15.0, 15.0;
    param.start_end_position_tolerance = 0.2;
    param.velocity_weight << 5.0, 5.0, 5.0;
    param.velocity_target << 1.0, 0.0, 0.0;
    param.max_vel_weight << 5.0, 5.0, 5.0;
    param.max_vel << 5.0, 5.0, 2.0;
    param.time_cost = 0.2;

    pn.param("opt/collision_weight", param.collision_weight, 20.0);
    pn.param("opt/collision_min_distance", param.collision_min_distance, 1.5);
    pn.param("opt/acceleration_weight_x", param.acceleration_weight.data()[0], 10.0);
    pn.param("opt/acceleration_weight_y", param.acceleration_weight.data()[1], 10.0);
    pn.param("opt/acceleration_weight_z", param.acceleration_weight.data()[2], 10.0);
    pn.param("opt/max_acc_weight_x", param.max_acc_weight.data()[0], 50.0);
    pn.param("opt/max_acc_weight_y", param.max_acc_weight.data()[1], 50.0);
    pn.param("opt/max_acc_weight_z", param.max_acc_weight.data()[2], 50.0);
    pn.param("opt/max_acc_x", param.max_acc.data()[0], 15.0);
    pn.param("opt/max_acc_y", param.max_acc.data()[1], 15.0);
    pn.param("opt/max_acc_z", param.max_acc.data()[2], 15.0);
    pn.param("opt/start_end_position_weight_x", param.start_end_position_weight.data()[0], 100.0);
    pn.param("opt/start_end_position_weight_y", param.start_end_position_weight.data()[1], 100.0);
    pn.param("opt/start_end_position_weight_z", param.start_end_position_weight.data()[2], 100.0);
    pn.param("opt/start_end_position_tolerance", param.start_end_position_tolerance, 0.2);
    pn.param("opt/velocity_weight_x", param.velocity_weight.data()[0], 3.0);
    pn.param("opt/velocity_weight_y", param.velocity_weight.data()[1], 3.0);
    pn.param("opt/velocity_weight_z", param.velocity_weight.data()[2], 3.0);
    pn.param("opt/max_vel_weight_x", param.max_vel_weight.data()[0], 15.0);
    pn.param("opt/max_vel_weight_y", param.max_vel_weight.data()[1], 15.0);
    pn.param("opt/max_vel_weight_z", param.max_vel_weight.data()[2], 15.0);
    pn.param("opt/max_vel_x", param.max_vel.data()[0], 5.0);
    pn.param("opt/max_vel_y", param.max_vel.data()[1], 5.0);
    pn.param("opt/max_vel_z", param.max_vel.data()[2], 2.0);
    pn.param("opt/time_cost", param.time_cost, 10.0);
    pn.param("opt/vfov_enabled", param.vfov_enabled, false);
    pn.param("opt/vfov_cost", param.vfov_cost, 100.0);
    pn.param("opt/vfov", param.vfov, 0.536);
    pn.param("opt/hfov_enabled", param.hfov_enabled, false);
    pn.param("opt/hfov_cost", param.hfov_cost, 100.0);
    pn.param("opt/hfov", param.hfov, 0.536);

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;

    m.header.frame_id = "odom_frame";
    m.header.seq = 0;
    m.header.stamp = ros::Time::now();

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
    m.scale.x = 0.05;
    m.scale.x = 0.05;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.id = 0;

    m.points.clear();

    for(auto v : discrete_points){
        geometry_msgs::Point p;
        p.x = v(0);
        p.y = v(1);
        p.z = v(2);
        m.points.push_back(p);
    }

    ma.markers.push_back(m);

    bspline_opt bo(splines, &map, param);
    Eigen::Vector3d start, end;
    start << 0, 0, 0;
    end << 0, 0, 0;
    bo.set_termination_velocity_states(start, end);
    bo.optimize();

    const double resolution = 0.1;
    vector<Eigen::VectorXd> points_opt;
    for(auto bs : splines){
        auto d = bs->get_discrete_bspline(resolution);
        printf("Bspline of size %i pushed\n", d.size());
        for(auto dd : d){
            points_opt.push_back(dd);
        }
    }

    m.id++;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.points.clear();

    for(auto v : points_opt){
        geometry_msgs::Point p;
        p.x = v(0);
        p.y = v(1);
        p.z = v(2);
        m.points.push_back(p);
    }

    ma.markers.push_back(m);

    m.id++;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.points.clear();
    
    for(auto v : points){
        geometry_msgs::Point p;
        p.x = v(0);
        p.y = v(1);
        p.z = v(2);
        m.points.push_back(p);
    }

    //ma.markers.push_back(m);

    m.id++;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.points.clear();
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.scale.x = 0.08;
    m.scale.y = 0.08;
    m.scale.z = 0.08;

    for(auto bs : splines){
        auto opt_control_points = bs->get_control_points();
        printf("Bspline: %i\n", opt_control_points.rows());
        for(size_t i = 0; i < opt_control_points.rows(); i++){
            auto v = opt_control_points.row(i);
            geometry_msgs::Point p;
            p.x = v(0);
            p.y = v(1);
            p.z = v(2);
            m.points.push_back(p);
            printf("(%f, %f, %f), ", v(0), v(1), v(2));
        }
        printf("\n");
    }

    ma.markers.push_back(m);

    m.type = visualization_msgs::Marker::LINE_STRIP;

    // Visualise velocity and acceleration too
    
    
    m.id++;

    vector<Eigen::VectorXd> vspline; 
    vector<Eigen::VectorXd> aspline;

    for(auto bs : splines){
        bspline vel_bspline = bs->get_derivative();
        bspline acc_bspline = vel_bspline.get_derivative();
        auto vv = vel_bspline.get_discrete_bspline(resolution);
        auto aa = acc_bspline.get_discrete_bspline(resolution);

        for(auto vvv : vv){
            vspline.push_back(vvv);
        }

        for(auto aaa : aa){
            aspline.push_back(aaa);
        }
    }

    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 0.05;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points.clear();    
    int count = 0;
    for(size_t i = 0; i < points_opt.size(); i++){
        auto pos = points_opt[i];
        auto vel = vspline[i];
        auto acc = aspline[i];

        // Dump info for analysis
        //printf("%i %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f ;\n", i, pos(0), pos(1), pos(2), vel(0), vel(1), vel(2), vel.norm(), acc(0), acc(1), acc(2), vel.norm());
        //vel *=10;
        //acc *=10;
        bool add = vel.norm() > 2.0;
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

        // m.color.r = 1.0;
        // m.color.g = 0;
        // m.color.b = 1.0;
        // m.id++;
        // m.points.clear();  
        // m.points.push_back(pos_p);
        // m.points.push_back(vel_p);
        // if(count % 10 == 0) ma.markers.push_back(m);

        // m.color.r = 0;
        // m.color.g = 1.0;
        // m.color.b = 1.0;
        // m.id++;
        // m.points.clear();  
        // m.points.push_back(pos_p);
        // m.points.push_back(acc_p);
        // if(count % 10 == 0) ma.markers.push_back(m);

        count++;
    }

    while(ros::ok()){
        vis_pub.publish(ma);
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }


    return 0;
}