#include "bspline.h"

#define ENABLE_DEBUG_MSG 0
#define DEBUG_PRINT(fmt, ...) do{ if(ENABLE_DEBUG_MSG) printf(fmt, __VA_ARGS__); } while(0)

using namespace std;

// The bspline constructed here is only valid between t=0 to t=m_knot_last.
// This is because of the way bsplines are formed, so any knots outside of this range should not be used.
bspline::bspline(Eigen::MatrixXd control_points, int degree, double interval){
    // Set up B-Spline parameters
    this->m_control_points = control_points;
    this->m_deg = degree;
    this->m_p = control_points.rows();
    this->m_n = this->m_deg + this->m_p + 1;
    this->m_interval = interval;

    DEBUG_PRINT("Creating BSpline with parameters: deg %i, points: %i, n: %i, int: %f\n", this->m_deg, this->m_p, this->m_n, this->m_interval);

    // Set up knot vector. We assume it is uniform at this point, but we can retime the trajectory later
    this->m_knot_vector = Eigen::VectorXd::Zero(this->m_n);

    this->set_interval(interval);

    // Debug print to check the knot vector
    if(ENABLE_DEBUG_MSG){
        DEBUG_PRINT("Knot Vector: ", "");
        for(size_t i = 0; i < this->m_n; ++i) DEBUG_PRINT("%f, ", this->m_knot_vector(i));
        DEBUG_PRINT("\n", "");
    }
}

void bspline::set_knot(Eigen::VectorXd &knots){
    this->m_knot_vector = knots;

    // Update the knot end time whenever we mess with the bspline
    this->update_knot_last();
}

void bspline::update_knot_last(){
    // Set end time of the trajectory to the correct index
    this->m_knot_last = this->m_knot_vector(this->m_n - this->m_deg - 1);
}

void bspline::set_interval(double interval){
    // Reconstruct the knot vector using new interval
    this->m_knot_offset = interval * ((double) this->m_deg);
    double max_val = interval * ((double) (this->m_p - this->m_deg));
    for(int i=0; i < this->m_n; ++i){
        double val = ((double) i) * interval - this->m_knot_offset;
        this->m_knot_vector(i) = (val < 0.0) ? 0.0 : (val > max_val) ? max_val : val;
    }

    this->update_knot_last();
}

vector<Eigen::VectorXd> bspline::get_discrete_bspline(double time_resolution){
    vector<Eigen::VectorXd> ret;    

    for(double time = 0; time <= this->m_knot_last + time_resolution; time += time_resolution){
        ret.push_back(get_discrete_bspline_point(time));
    }
    
    return ret;
}

Eigen::VectorXd bspline::get_discrete_bspline_point(double time){
    return evaluate_de_boor(time);
}

Eigen::VectorXd bspline::evaluate_de_boor(double time){
    // Constrain time so we only look in the valid range of time
    double time_bounded = (time < 0) ? 0 : (time >= this->m_knot_last) ? this->m_knot_last - 0.000001 : time;

    // The following has been written based on python code from wikipedia implementing de boor's algorithm
    // https://en.wikipedia.org/wiki/De_Boor%27s_algorithm#Example_implementation
    // Find the sample number which the current time correlates to
    int k = m_deg;
    for(;k < (this->m_p + 1); k++){
        if(this->m_knot_vector(k + 1) >= time_bounded) {
            DEBUG_PRINT("Found time %f at knot index %i, v %f\n", time_bounded, k, this->m_knot_vector(k+1));
            break;
        }
    }

    // Evaluate de boor's algorithm
    vector<Eigen::VectorXd> d;
    for(int i=0; i<this->m_deg + 1; ++i){
        d.push_back(this->m_control_points.row(i + k - this->m_deg));
    }

    for(int r = 1; r < this->m_deg + 1; ++r){
        for(int j = this->m_deg; j >= r; --j){
            double alpha = (time_bounded - this->m_knot_vector(j + k - this->m_deg)) / (this->m_knot_vector(j + 1 + k - r) - this->m_knot_vector(j + k - this->m_deg));
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
        }
    }

    return d[this->m_deg];    
}

bspline bspline::get_derivative(){
    // Calculate the derivatives of our b spline by manipulating control points. Theory found here:
    // https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
    
    // NOTE: ONLY PRODUCES VALID RESULTS FOR BSPLINES WITH ORDER > 1!! EXPECT SEGFAULTS IF USED ON AN ORDER 1 BSPLINE

    Eigen::MatrixXd control_points(this->m_control_points.rows() - 1, this->m_control_points.cols());

    for(int i=0; i < this->m_control_points.rows() - 1; ++i){
        // Get the current and next control point
        Eigen::VectorXd pi, pip, dp;
        pi = this->m_control_points.row(i);
        pip = this->m_control_points.row(i+1);

        dp = ((this->m_deg) * (pip - pi)) / (this->m_knot_vector(i + this->m_deg + 1) - this->m_knot_vector(i + 1));
        control_points.row(i) = dp;
    }

    bspline b(control_points, this->m_deg - 1, this->m_interval);
    Eigen::VectorXd new_knot(this->m_knot_vector.rows() - 2);
    new_knot = this->m_knot_vector.segment(1, this->m_knot_vector.rows() - 2);
    b.set_knot(new_knot);

    return b;
}

// Thanks fast-planner!
// https://github.com/HKUST-Aerial-Robotics/Fast-Planner
bspline* bspline_util::fit_bspline(vector<Eigen::Vector3d>* fit_points, double interval){
    int K = fit_points->size();

    if(K < 2){
        printf("Cannot create bspline with < 2 points!\n");
        return NULL;
    }

    Eigen::Vector3d prow, vrow, arow;
    // write A
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    for (int i = 0; i < K; ++i) A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    A.block(K, 0, 1, 3)         = (1 / 2.0 / interval) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / interval) * vrow.transpose();

    A.block(K + 2, 0, 1, 3)     = (1 / interval / interval) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / interval / interval) * arow.transpose();

    // write b
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    for (int i = 0; i < K; ++i) {
        bx(i) = (*fit_points)[i](0);
        by(i) = (*fit_points)[i](1);
        bz(i) = (*fit_points)[i](2);
    }

    for (int i = 0; i < 4; ++i) {
        bx(K + i) = 0;
        by(K + i) = 0;
        bz(K + i) = 0;
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    Eigen::MatrixXd control_points = Eigen::MatrixXd::Zero(K + 2, 3);
    control_points.col(0) = px;
    control_points.col(1) = py;
    control_points.col(2) = pz;

    // Rig first and last control point so the provided bspline runs through the provided start/end
    control_points.row(0) = (*fit_points)[0];

    //control_points = control_points.block(1, 0, control_points.rows() - 1, control_points.cols());

    return new bspline(control_points, 3, interval);
}

Eigen::MatrixXd bspline::get_control_points(){
    return this->m_control_points;
}
