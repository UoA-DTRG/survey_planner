#ifndef __BSPLINE_H
#define __BSPLINE_H
#pragma once

#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen>

using namespace std;


class bspline{
public:
    bspline(Eigen::MatrixXd control_points, int order, double interval);
    vector<Eigen::VectorXd> get_discrete_bspline(double time_resolution);
    Eigen::VectorXd get_discrete_bspline_point(double time);
    void set_knot(Eigen::VectorXd &knots);
    int get_order() { return this->m_deg; };
    Eigen::VectorXd get_knots() { return this->m_knot_vector; };
    double get_interval() { return this->m_interval; }
    void set_interval(double interval);
    Eigen::MatrixXd get_control_points();
    void set_control_points(Eigen::MatrixXd points) { this->m_control_points = points; };
    void update_knot_last();
    double get_knot_last() { return this->m_knot_last; };
    bspline get_derivative();

private:
    // Control points are stored in row major order.
    Eigen::MatrixXd m_control_points;
    Eigen::VectorXd m_knot_vector;
    int m_n, m_deg, m_p;
    double m_knot_offset;   // This is the time offset due to additional knot constraints
    double m_knot_last;     // This is the time of the last knot
    double m_temporal_resolution; // default time resolution to generate discrete points at
    double m_interval;
    Eigen::VectorXd evaluate_de_boor(double time);

};

namespace bspline_util{
    bspline* fit_bspline(vector<Eigen::Vector3d>* fit_points, double interval);
};

#endif