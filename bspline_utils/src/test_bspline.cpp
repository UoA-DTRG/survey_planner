#include <ros/ros.h>
#include "bspline.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "bspline_test_node");
    ros::NodeHandle n;

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>("/bspline_vis", 5);

    ros::spinOnce();
    
    vector<Eigen::Vector3d> points;
    Eigen::Vector3d p;
    p << 0, 0, 0;
    points.push_back(p);
    p << 1, 0, 0;
    points.push_back(p);
    p << 1, 1, 0;
    points.push_back(p);
    p << 1, 1, 1;
    points.push_back(p);
    p << 2, 1, 0;
    points.push_back(p);
    p << 2, 0, 2;
    points.push_back(p);
    p << 1, 0, 2;
    points.push_back(p);

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

    Eigen::MatrixXd test_control_points(points.size(), 3);
    for(size_t i=0; i < points.size(); ++i) test_control_points.row(i) = points[i];

    //bspline b(test_control_points, 4, 1);

    //vector<Eigen::VectorXd> spline = b.get_discrete_bspline(0.1);

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

    for(auto v : points){
        geometry_msgs::Point p;
        p.x = v(0);
        p.y = v(1);
        p.z = v(2);
        m.points.push_back(p);
    }

    ma.markers.push_back(m);

    bspline* fit = bspline_util::fit_bspline(&points, 1);
    vector<Eigen::VectorXd> spline = fit->get_discrete_bspline(0.1);

    m.color.g = 1.0;
    m.color.r = 0.0;
    m.id++;
    m.points.clear();

    for(auto v : spline){
        geometry_msgs::Point p;
        p.x = v(0);
        p.y = v(1);
        p.z = v(2);
        m.points.push_back(p);
    }

    ma.markers.push_back(m);

    auto control_points = fit->get_control_points();

    m.id++;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.points.clear();

    for(int i = 0; i < control_points.rows(); ++i){
        Eigen::VectorXd v = control_points.row(i);
        
        geometry_msgs::Point p;
        p.x = v(0);
        p.y = v(1);
        p.z = v(2);
        m.points.push_back(p);
    }

    ma.markers.push_back(m);

    

    bspline bv = fit->get_derivative();
    bspline ba = bv.get_derivative();
    vector<Eigen::VectorXd> vspline = bv.get_discrete_bspline(0.1);
    vector<Eigen::VectorXd> aspline = ba.get_discrete_bspline(0.1);

    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.points.clear();    
    for(size_t i = 0; i < spline.size(); i++){
        auto pos = spline[i];
        auto vel = vspline[i];
        auto acc = aspline[i];
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

        m.color.r = 0;
        m.color.g = 1.0;
        m.color.b = 0;
        m.id++;
        m.points.clear();  
        m.points.push_back(pos_p);
        m.points.push_back(vel_p);
        ma.markers.push_back(m);

        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 1.0;
        m.id++;
        m.points.clear();  
        m.points.push_back(pos_p);
        m.points.push_back(acc_p);
        ma.markers.push_back(m);

    }

    while(ros::ok()){
        vis_pub.publish(ma);
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    return 0;
}