#include "bspline_opt.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

using namespace std;

#define PRINT_SOLVER_STATUS 0
#define ENABLE_DEBUG_MSG 0
#define DUMP_CONTROLPOINT_KNOT 0
#define DEBUG_PRINT(fmt, ...) do{ if(ENABLE_DEBUG_MSG) printf(fmt, __VA_ARGS__); } while(0)

// We have to formulate the below as a normal cost function because we cannot use autodiff or numericdiff
// on FIESTA's discrete ESDF representation (i.e. autodiff doesn't work, numeric diff is extremely inefficient). 
// Quick testing (see Numeric diff module below) suggests a 10x increase in runtime using numeric diff
// To get around this, we evaluate the distance and gradients via trilinear interpolation on the ESDF and return
// them to Ceres directly. This is the same as using an optimized analytical derivative
class CollisionCostConstraint : public ceres::CostFunction {
public:
    // Sets up cost function construction
    CollisionCostConstraint(fiesta::ESDFMap* m, double min_dist, double wdist) : map(m), min_distance(min_dist), dist_weight(wdist) {
        set_num_residuals(1);
        auto block_size_ptr = mutable_parameter_block_sizes();
        block_size_ptr->push_back(3);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const{
        // Construct the current position vector. This is slightly faster than creating a new Eigen::Vector3d
        Eigen::Map<const Eigen::Matrix<double, 3, 1>> pos(parameters[0]);

        // We calculate the gradient via trilinear interpolation. Distance information is basically free with this
        Eigen::Vector3d grad;
        double dist = map->GetDistWithGradTrilinear(pos, grad);
        if(dist == -1) dist = min_distance;
        DEBUG_PRINT("Cost function, loc: %f, %f, %f, distance: %f\n", pos(0), pos(1), pos(2), dist);

        // Calculate residual
        residuals[0] = dist_weight * ((dist - min_distance) > 0 ? 0 : ((min_distance - dist) * (min_distance - dist)));

        DEBUG_PRINT("Residual %f\n", residuals[0]);

        // Only write to jacobian if Ceres explicitly requests it.
        if(jacobians != NULL){

            if((dist - min_distance) < 0){
                // Normalize the gradient information in the case where we get a frontier
                // this will fix issues where the frontier boundaries result in extremely large gradients
                // if(grad.norm() > 1e-7){
                //     grad.normalize();
                // }

                // We set this up so that the gradient is in the same structure as the difference, therefore
                // the optimizer does not go the wrong way
                //grad = (dist - min_distance) * grad;
                grad = -grad;

                DEBUG_PRINT("Cost function: grad: %f, %f, %f\n", grad(0), grad(1), grad(2));

                jacobians[0][0] = grad(0);
                jacobians[0][1] = grad(1);
                jacobians[0][2] = grad(2);
            }else{
                // Only generate positive gradient if a collision is present, as we don't want collisions influencing
                // the control points after a zero cost is achieved
                jacobians[0][0] = 0;
                jacobians[0][1] = 0;
                jacobians[0][2] = 0;
            }
        }

        return true;
    }

private:
    fiesta::ESDFMap* map;
    const double min_distance;
    const double dist_weight;
};

struct CollisionCostNumeric{
    typedef ceres::NumericDiffCostFunction<CollisionCostNumeric, ceres::CENTRAL, 1, 3> CollisionCostFunctionNumeric;

    CollisionCostNumeric(fiesta::ESDFMap* m, double min_dist, double wdist) : 
        map(m), 
        min_distance(min_dist), 
        dist_weight(wdist) {}

    bool operator()(const double* const parameters, double* residuals) const{
        Eigen::Map<const Eigen::Matrix<double, 3, 1>> pos(parameters);
        Eigen::Vector3d grad;
        double dist = map->GetDistWithGradTrilinear(pos, grad);
        //double dist = map->GetDistance(pos);

        if(dist == 10000 || dist == -10000) dist = min_distance;

        residuals[0] = dist_weight * ((dist - min_distance) > 0 ? 0 : ((min_distance - dist)));
        return true;
    }

    static CollisionCostFunctionNumeric* Create(fiesta::ESDFMap* m, double min_dist, double wdist){
        return new CollisionCostFunctionNumeric(new CollisionCostNumeric(m, min_dist, wdist));
    }

    fiesta::ESDFMap* map;
    const double min_distance;
    const double dist_weight;
};

// We use Ceres's autodiff to compute acceleration gradients. This is simply the forward difference
// of the acceleration value at point p3. Ceres is allowed to tweak the interval of the bspline to
// produce better quality trajectories
struct AccelerationConstraint{
    typedef ceres::AutoDiffCostFunction<AccelerationConstraint, 6, 3, 3, 3, 1> AccelerationCostFunction;

    AccelerationConstraint(Eigen::Vector3d a_weight, Eigen::Vector3d max_a, Eigen::Vector3d max_a_w, double o, double time_mult_l, double time_mult_h) : 
        accel_weight(a_weight),
        max_acc_weight(max_a_w),
        max_acc(max_a),
        order(o),
        time_mult_l(time_mult_l),
        time_mult_h(time_mult_h) {}

    template<typename T>
    bool operator()(const T* const p2, const T* const p3, const T* const p4, const T* const interval, T* residuals) const{
        // These are technically all Eigen::Vector3ds, but we use Matrix<T> so we can use autodifferentiation
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pim1(p2);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(p3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(p4);
        
        // Compute intervals for accurate velocity tracking. These are computed as a multiplier of the
        // order of the bspline used, and will let us constrain velocities/accelerations when required
        const T interval_l = interval[0] * time_mult_l;
        const T interval_h = interval[0] * time_mult_h;
        const T interval_a = interval[0] * (max(time_mult_l, time_mult_h) - 1);

        DEBUG_PRINT("Interval: %f\n", interval[0]);

        // Compute the velocity and acceleration vectors
        Eigen::Matrix<T, 3, 1> vel1 = order * (pip1 - pi) / interval_h;
        Eigen::Matrix<T, 3, 1> vel2 = order * (pi - pim1) / interval_l;
        Eigen::Matrix<T, 3, 1> accel = (order - 1) * (vel1 - vel2) / interval_a;
        
        // This is technically NOT the true acceleration. It doesn't really matter here if it's correct
        // or not, since the time taken is simply a scaling factor
        //Eigen::Matrix<T, 3, 1> weighted_accel = accel_weight.asDiagonal() * accel;

        residuals[0] = accel_weight(0) * (accel(0));
        residuals[1] = accel_weight(1) * (accel(1));
        residuals[2] = accel_weight(2) * (accel(2));

        // The lower interval is used to determine acceleration limits. This is technically incorrect
        // but there is no easy way of fixing it which works for all cases. Might need to implement something
        residuals[3] = ((accel(0) * accel(0)) > ((max_acc(0)) * (max_acc(0)))) ? max_acc_weight(0) * (accel(0) * accel(0)) : T(0.0);
        residuals[4] = ((accel(1) * accel(1)) > ((max_acc(1)) * (max_acc(1)))) ? max_acc_weight(1) * (accel(1) * accel(1)) : T(0.0);
        residuals[5] = ((accel(2) * accel(2)) > ((max_acc(2)) * (max_acc(2)))) ? max_acc_weight(2) * (accel(2) * accel(2)) : T(0.0);

        return true;        
    }

    // Helper function to create a residual block to pass into Ceres
    static AccelerationCostFunction* Create(Eigen::Vector3d a_weight, Eigen::Vector3d max_a, Eigen::Vector3d max_a_w, double order, double time_mult_l, double time_mult_h) {
        return new AccelerationCostFunction(new AccelerationConstraint(a_weight, max_a, max_a_w, order, time_mult_l, time_mult_h));
    }

    const double order;                         // Order of the BSpline
    const double time_mult_l;                   // Interval multiplier for the p-1 to p time
    const double time_mult_h;                   // Interval multiplier for the p to p+1 time
    const Eigen::Vector3d max_acc_weight;       // Acceleration feasibility weighting for
    const Eigen::Vector3d max_acc;              // Maximum acceptable acceleration
    const Eigen::Vector3d accel_weight;         // Acceleration weight
};

// We use Ceres's autodiff to compute acceleration gradients. This is simply the forward difference
// of the acceleration value at point p3. Ceres is allowed to tweak the interval of the bspline to
// produce better quality trajectories
struct SpliceAccelerationConstraint{
    typedef ceres::AutoDiffCostFunction<SpliceAccelerationConstraint, 6, 3, 3, 3, 1, 1> SpliceAccelerationCostFunction;

    SpliceAccelerationConstraint(Eigen::Vector3d a_weight, Eigen::Vector3d max_a, Eigen::Vector3d max_a_w, double o, double time_mult_l, double time_mult_h) : 
        accel_weight(a_weight),
        max_acc_weight(max_a_w),
        max_acc(max_a),
        order(o),
        time_mult_l(time_mult_l),
        time_mult_h(time_mult_h) {}

    template<typename T>
    bool operator()(const T* const p2, const T* const p3, const T* const p4, const T* const interval1, const T* const interval2, T* residuals) const{
        // These are technically all Eigen::Vector3ds, but we use Matrix<T> so we can use autodifferentiation
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pim1(p2);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(p3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(p4);
        
        // Compute intervals for accurate velocity tracking. These are computed as a multiplier of the
        // order of the bspline used, and will let us constrain velocities/accelerations when required
        const T interval_l = interval1[0] * time_mult_l;
        const T interval_h = interval2[0] * time_mult_h;

        // Slight hack here, interval_a will be computed as an average of the two intervals
        const T interval_a = (0.5 * (interval1[0] + interval2[0])) * (max(time_mult_l, time_mult_h) - 1);

        DEBUG_PRINT("Interval: %f\n", interval1[0]);

        // Compute the velocity and acceleration vectors
        Eigen::Matrix<T, 3, 1> vel1 = order * (pip1 - pi) / interval_h;
        Eigen::Matrix<T, 3, 1> vel2 = order * (pi - pim1) / interval_l;
        Eigen::Matrix<T, 3, 1> accel = (order - 1) * (vel1 - vel2) / interval_a;
        
        // This is technically NOT the true acceleration. It doesn't really matter here if it's correct
        // or not, since the time taken is simply a scaling factor
        //Eigen::Matrix<T, 3, 1> weighted_accel = accel_weight.asDiagonal() * accel;

        residuals[0] = accel_weight(0) * (accel(0));
        residuals[1] = accel_weight(1) * (accel(1));
        residuals[2] = accel_weight(2) * (accel(2));

        // The lower interval is used to determine acceleration limits. This is technically incorrect
        // but there is no easy way of fixing it which works for all cases. Might need to implement something
        residuals[3] = ((accel(0) * accel(0)) > ((max_acc(0)) * (max_acc(0)))) ? max_acc_weight(0) * (accel(0) * accel(0)) : T(0.0);
        residuals[4] = ((accel(1) * accel(1)) > ((max_acc(1)) * (max_acc(1)))) ? max_acc_weight(1) * (accel(1) * accel(1)) : T(0.0);
        residuals[5] = ((accel(2) * accel(2)) > ((max_acc(2)) * (max_acc(2)))) ? max_acc_weight(2) * (accel(2) * accel(2)) : T(0.0);

        return true;        
    }

    // Helper function to create a residual block to pass into Ceres
    static SpliceAccelerationCostFunction* Create(Eigen::Vector3d a_weight, Eigen::Vector3d max_a, Eigen::Vector3d max_a_w, double order, double time_mult_l, double time_mult_h) {
        return new SpliceAccelerationCostFunction(new SpliceAccelerationConstraint(a_weight, max_a, max_a_w, order, time_mult_l, time_mult_h));
    }

    const double order;                         // Order of the BSpline
    const double time_mult_l;                   // Interval multiplier for the p-1 to p time
    const double time_mult_h;                   // Interval multiplier for the p to p+1 time
    const Eigen::Vector3d max_acc_weight;       // Acceleration feasibility weighting for
    const Eigen::Vector3d max_acc;              // Maximum acceptable acceleration
    const Eigen::Vector3d accel_weight;         // Acceleration weight
};

// Initial variant of AccelerationConstraint, so the first acceleration is optimized too
struct InitialAccelerationConstraint{
    typedef ceres::AutoDiffCostFunction<InitialAccelerationConstraint, 6, 3, 3, 1> InitialAccelerationCostFunction;

    InitialAccelerationConstraint(Eigen::Vector3d a_weight, Eigen::Vector3d max_a, Eigen::Vector3d max_a_w, Eigen::Vector3d pim1, double o, double time_mult_l, double time_mult_h) : 
        accel_weight(a_weight),
        max_acc_weight(max_a_w),
        max_acc(max_a),
        pim1(pim1),
        order(o),
        time_mult_l(time_mult_l),
        time_mult_h(time_mult_h) {}

    template<typename T>
    bool operator()(const T* const p3, const T* const p4, const T* const interval, T* residuals) const{
        // These are technically all Eigen::Vector3ds, but we use Matrix<T> so we can use autodifferentiation
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(p3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(p4);
        
        // Compute intervals for accurate velocity tracking. These are computed as a multiplier of the
        // order of the bspline used, and will let us constrain velocities/accelerations when required
        const T interval_l = interval[0] * time_mult_l;
        const T interval_h = interval[0] * time_mult_h;
        const T interval_a = interval[0] * (max(time_mult_l, time_mult_h) - 1);

        DEBUG_PRINT("Interval: %f\n", interval[0]);

        // Compute the velocity and acceleration vectors
        Eigen::Matrix<T, 3, 1> vel1 = order * (pip1 - pi) / interval_l;
        Eigen::Matrix<T, 3, 1> vel2 = order * (pi - pim1) / interval_h;
        Eigen::Matrix<T, 3, 1> accel = (order - 1) * (vel1 - vel2) / interval_a;
        
        // This is technically NOT the true acceleration. It doesn't really matter here if it's correct
        // or not, since the time taken is simply a scaling factor
        //Eigen::Matrix<T, 3, 1> weighted_accel = accel_weight.asDiagonal() * accel;

        residuals[0] = accel_weight(0) * (accel(0));
        residuals[1] = accel_weight(1) * (accel(1));
        residuals[2] = accel_weight(2) * (accel(2));

        // The lower interval is used to determine acceleration limits. This is technically incorrect
        // but there is no easy way of fixing it which works for all cases. Might need to implement something
        residuals[3] = ((accel(0) * accel(0)) > ((max_acc(0)) * (max_acc(0)))) ? max_acc_weight(0) * (accel(0) * accel(0)) : T(0.0);
        residuals[4] = ((accel(1) * accel(1)) > ((max_acc(1)) * (max_acc(1)))) ? max_acc_weight(1) * (accel(1) * accel(1)) : T(0.0);
        residuals[5] = ((accel(2) * accel(2)) > ((max_acc(2)) * (max_acc(2)))) ? max_acc_weight(2) * (accel(2) * accel(2)) : T(0.0);

        return true;        
    }

    // Helper function to create a residual block to pass into Ceres
    static InitialAccelerationCostFunction* Create(Eigen::Vector3d a_weight, Eigen::Vector3d max_a, Eigen::Vector3d max_a_w, Eigen::Vector3d pim1, double order, double time_mult_l, double time_mult_h) {
        return new InitialAccelerationCostFunction(new InitialAccelerationConstraint(a_weight, max_a, max_a_w, pim1, order, time_mult_l, time_mult_h));
    }

    const double order;                         // Order of the BSpline
    const double time_mult_l;                   // Interval multiplier for the p-1 to p time
    const double time_mult_h;                   // Interval multiplier for the p to p+1 time
    const Eigen::Vector3d max_acc_weight;       // Acceleration feasibility weighting for
    const Eigen::Vector3d max_acc;              // Maximum acceptable acceleration
    const Eigen::Vector3d accel_weight;         // Acceleration weight
    const Eigen::Vector3d pim1;
};

// Speed constraint
struct MaxSpeedConstraint{
    typedef ceres::AutoDiffCostFunction<MaxSpeedConstraint, 3, 3, 3, 1> MaxSpeedCostFunction;

    MaxSpeedConstraint(double w, Eigen::Vector3d m, Eigen::Vector3d m_w, double order, double time_mult) : 
        speed_weight(w), 
        max_vel_weight(m_w),
        max_vel(m),
        order(order),
        time_mult(time_mult) {}

    template <typename T>
    bool operator()(const T* const p1, const T* const p2, const T* const t, T* residuals) const {
        // Get both P1 and P2 in Eigen form for easy processing
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(p1);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(p2);
        
        T time = t[0] * time_mult;
        auto vel = order * (pip1 - pi);

        // Max velocity constraints. These are true velocities instead of an estimate
        residuals[0] = ((vel(0) * vel(0)) > ((max_vel(0) * time) * (max_vel(0) * time))) ? max_vel_weight(0) * (vel(0) * vel(0)) : T(0.0);
        residuals[1] = ((vel(1) * vel(1)) > ((max_vel(1) * time) * (max_vel(1) * time))) ? max_vel_weight(1) * (vel(1) * vel(1)) : T(0.0);
        residuals[2] = ((vel(2) * vel(2)) > ((max_vel(2) * time) * (max_vel(2) * time))) ? max_vel_weight(2) * (vel(2) * vel(2)) : T(0.0);

        return true;
    }

    static MaxSpeedCostFunction* Create(double w, Eigen::Vector3d m, Eigen::Vector3d m_w, double order, double time_mult){
        return new MaxSpeedCostFunction(new MaxSpeedConstraint(w, m, m_w, order, time_mult));
    }

    const double order;                         // Order of the BSpline
    const double time_mult;                     // Multiple of the interval to produce the time interval
    const double speed_weight;                  // Speed weight
    const Eigen::Vector3d max_vel_weight;       // Maximum velocity weight
    const Eigen::Vector3d max_vel;              // Maximum permissible velocity
};

// Speed constraint
struct SpeedConstraint{
    typedef ceres::AutoDiffCostFunction<SpeedConstraint, 1, 3, 3, 1> SpeedCostFunction;

    SpeedConstraint(double w, double t, Eigen::Vector3d m, Eigen::Vector3d m_w, double order, double time_mult) : 
        speed_weight(w), 
        target_speed(t), 
        max_vel_weight(m_w),
        max_vel(m),
        order(order),
        time_mult(time_mult) {}

    template <typename T>
    bool operator()(const T* const p1, const T* const p2, const T* const t, T* residuals) const {
        // Get both P1 and P2 in Eigen form for easy processing
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(p1);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(p2);
        
        T time = t[0] * time_mult;
        auto vel = order * (pip1 - pi);
        auto vel_t = target_speed * time;

        // Project vel into target
        auto vel_diff = vel.norm() - vel_t;

        DEBUG_PRINT("Vel err: %f\n", vel_diff);

        // Set residuals to the error in each axis
        residuals[0] = speed_weight * (vel_diff);

        return true;
    }

    static SpeedCostFunction* Create(double w, double t, Eigen::Vector3d m, Eigen::Vector3d m_w, double order, double time_mult){
        return new SpeedCostFunction(new SpeedConstraint(w, t, m, m_w, order, time_mult));
    }

    const double order;                         // Order of the BSpline
    const double time_mult;                     // Multiple of the interval to produce the time interval
    const double speed_weight;                  // Speed weight
    const double target_speed;                  // Target speed
    const Eigen::Vector3d max_vel_weight;       // Maximum velocity weight
    const Eigen::Vector3d max_vel;              // Maximum permissible velocity
};

// The first variant of velocity constraints. Velocities are computed simply by taking the forward difference
// between p1 and p2, and timing is provided via the bspline interval. This constraint actually only constrains
// the projected velocity instead of the entire velocity with the intent of enforcing a more consistent velocity
// profile.
struct VelocityConstraint{
    typedef ceres::AutoDiffCostFunction<VelocityConstraint, 6, 3, 3, 1> VelocityCostFunction;

    VelocityConstraint(Eigen::Vector3d w, Eigen::Vector3d t, Eigen::Vector3d m, Eigen::Vector3d m_w, double order, double time_mult) : 
        vel_weight(w), 
        target_vel(t), 
        max_vel_weight(m_w),
        max_vel(m),
        order(order),
        time_mult(time_mult) {}

    template <typename T>
    bool operator()(const T* const p1, const T* const p2, const T* const t, T* residuals) const {
        // Get both P1 and P2 in Eigen form for easy processing
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(p1);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(p2);
        
        T time = t[0] * time_mult;
        auto vel = order * (pip1 - pi);
        auto vel_t = target_vel * time;

        // Project vel into target
        //auto vel_proj = (((target_vel.dot(vel) / target_vel.dot(target_vel)) * target_vel) - target_vel);
        auto vel_proj = (((vel_t.dot(vel) / vel_t.dot(vel_t)) * vel_t) - vel_t);
        auto vel_weighted = vel_weight.asDiagonal() * vel_proj;

        // Set residuals to the error in each axis
        residuals[0] = vel_weighted(0);
        residuals[1] = vel_weighted(1);
        residuals[2] = vel_weighted(2);

        // Max velocity constraints. These are true velocities instead of an estimate
        residuals[3] = ((vel(0) * vel(0)) > ((max_vel(0) * time) * (max_vel(0) * time))) ? max_vel_weight(0) * (vel(0) * vel(0)) : T(0.0);
        residuals[4] = ((vel(1) * vel(1)) > ((max_vel(1) * time) * (max_vel(1) * time))) ? max_vel_weight(1) * (vel(1) * vel(1)) : T(0.0);
        residuals[5] = ((vel(2) * vel(2)) > ((max_vel(2) * time) * (max_vel(2) * time))) ? max_vel_weight(2) * (vel(2) * vel(2)) : T(0.0);

        return true;
    }

    static VelocityCostFunction* Create(Eigen::Vector3d w, Eigen::Vector3d t, Eigen::Vector3d m, Eigen::Vector3d m_w, double order, double time_mult){
        return new VelocityCostFunction(new VelocityConstraint(w, t, m, m_w, order, time_mult));
    }

    const double order;                         // Order of the BSpline
    const double time_mult;                     // Multiple of the interval to produce the time interval
    const Eigen::Vector3d vel_weight;           // Velocity weight
    const Eigen::Vector3d target_vel;           // Target velocity as a vector, the actual velocity is projected onto this vector
    const Eigen::Vector3d max_vel_weight;       // Maximum velocity weight
    const Eigen::Vector3d max_vel;              // Maximum permissible velocity
};

// Velocity state constraint, the same as above, but without projecting the velocity vector.
// This should be used to constrain the initial and final states to whatever is desired
struct VelocityStateConstraint{
    typedef ceres::AutoDiffCostFunction<VelocityStateConstraint, 3, 3, 3, 1> VelocityStateCostFunction;

    VelocityStateConstraint(Eigen::Vector3d w, Eigen::Vector3d t, Eigen::Vector3d m, Eigen::Vector3d m_w, double order, double time_mult) : 
        vel_weight(w), 
        target_vel(t), 
        max_vel_weight(m_w),
        max_vel(m),
        order(order),
        time_mult(time_mult) {}

    template <typename T>
    bool operator()(const T* const p1, const T* const p2, const T* const t, T* residuals) const {
        // Get both P1 and P2 in Eigen form for easy processing
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(p1);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(p2);
        
        T time = t[0] * time_mult;
        auto vel = order * (pip1 - pi);
        auto vel_t = target_vel * time;

        auto vel_weighted = vel - vel_t;

        // Set residuals to the error in each axis
        // We use the squared error here to make sure that the state constraint is well satisifed
        residuals[0] = vel_weight(0) * (vel_weighted(0) * vel_weighted(0));
        residuals[1] = vel_weight(1) * (vel_weighted(1) * vel_weighted(1));
        residuals[2] = vel_weight(2) * (vel_weighted(2) * vel_weighted(2));

        residuals[3] = ((vel(0) * vel(0)) > ((max_vel(0) * time) * (max_vel(0) * time))) ? max_vel_weight(0) * (vel(0) * vel(0)) : T(0.0);
        residuals[4] = ((vel(1) * vel(1)) > ((max_vel(1) * time) * (max_vel(1) * time))) ? max_vel_weight(1) * (vel(1) * vel(1)) : T(0.0);
        residuals[5] = ((vel(2) * vel(2)) > ((max_vel(2) * time) * (max_vel(2) * time))) ? max_vel_weight(2) * (vel(2) * vel(2)) : T(0.0);

        return true;
    }

    static VelocityStateCostFunction* Create(Eigen::Vector3d w, Eigen::Vector3d t, Eigen::Vector3d m, Eigen::Vector3d m_w, double order, double time_mult){
        // We prescale target vel by bspline_interval to save a divide during runtime evaluation
        return new VelocityStateCostFunction(new VelocityStateConstraint(w, t, m, m_w, order, time_mult));
    }

    const double order;                         // Order of the bspline
    const double time_mult;                     // Interval multiplier to get velocity time
    const Eigen::Vector3d vel_weight;
    const Eigen::Vector3d target_vel;
    const Eigen::Vector3d max_vel_weight;
    const Eigen::Vector3d max_vel;
};

// Variation of VelocityStateConstraint which does not move the first control point. This is to guarantee continunity
// of the path that is generated
struct InitialVelocityStateConstraint{
    typedef ceres::AutoDiffCostFunction<InitialVelocityStateConstraint, 3, 3, 1> InitialVelocityStateCostFunction;

    InitialVelocityStateConstraint(Eigen::Vector3d w, Eigen::Vector3d t, Eigen::Vector3d p1, double order, double time_mult) : 
        vel_weight(w), 
        target_vel(t), 
        order(order),
        p1(p1),
        time_mult(time_mult) {}

    template <typename T>
    bool operator()(const T* const p2, const T* const t, T* residuals) const {
        // Get both P1 and P2 in Eigen form for easy processing
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(p2);
        
        T time = t[0] * time_mult;
        auto vel = order * (pip1 - p1);
        auto vel_t = target_vel * time;

        auto vel_weighted = vel - vel_t;

        // Set residuals to the error in each axis
        // We use the squared error here to make sure that the state constraint is well satisifed
        residuals[0] = vel_weight(0) * (vel_weighted(0) * vel_weighted(0));
        residuals[1] = vel_weight(1) * (vel_weighted(1) * vel_weighted(1));
        residuals[2] = vel_weight(2) * (vel_weighted(2) * vel_weighted(2));

        return true;
    }

    static InitialVelocityStateCostFunction* Create(Eigen::Vector3d w, Eigen::Vector3d t, Eigen::Vector3d p1, double order, double time_mult){
        // We prescale target vel by bspline_interval to save a divide during runtime evaluation
        return new InitialVelocityStateCostFunction(new InitialVelocityStateConstraint(w, t, p1, order, time_mult));
    }

    const double order;                         // Order of the bspline
    const double time_mult;                     // Interval multiplier to get velocity time
    const Eigen::Vector3d vel_weight;
    const Eigen::Vector3d target_vel;
    const Eigen::Vector3d p1;
};

// Variation of VelocityStateConstraint which enforces identical velocities at the boundary of the control points
struct DiffVelocityStateConstraint{
    typedef ceres::AutoDiffCostFunction<DiffVelocityStateConstraint, 3, 3, 3, 3, 1, 1> DiffVelocityStateCostFunction;

    DiffVelocityStateConstraint(Eigen::Vector3d w, double order) : 
        vel_weight(w), 
        order(order) {}

    template <typename T>
    bool operator()(const T* p1, const T* const p2, const T* const p3, const T* const i1, const T* const i2, T* residuals) const {
        // Get both P1 and P2 in Eigen form for easy processing
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pim1(p1);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pi(p2);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(p3);
        
        auto vel2 = i1[0] * order * (pip1 - pi);
        auto vel1 = i2[0] * order * (pi - pim1);

        auto veldiff = vel1 - vel2;

        // Set residuals to the error in each axis
        // We use the squared error here to make sure that the state constraint is well satisifed
        residuals[0] = vel_weight(0) * (veldiff(0) * veldiff(0));
        residuals[1] = vel_weight(1) * (veldiff(1) * veldiff(1));
        residuals[2] = vel_weight(2) * (veldiff(2) * veldiff(2));

        return true;
    }

    static DiffVelocityStateCostFunction* Create(Eigen::Vector3d w, double order){
        // We prescale target vel by bspline_interval to save a divide during runtime evaluation
        return new DiffVelocityStateCostFunction(new DiffVelocityStateConstraint(w, order));
    }

    const double order;                         // Order of the bspline
    const Eigen::Vector3d vel_weight;
};

// Simple constraint which encourages two control points to overlap completely.
// We use this to ensure continuity with multi-waypoint trajectories
struct ControlPointOverlapConstraint{
    typedef ceres::AutoDiffCostFunction<ControlPointOverlapConstraint, 3, 3, 3> ControlPointOverlapCostFunction;

    ControlPointOverlapConstraint(Eigen::Vector3d w) : 
        pos_weight(w) {}

    template <typename T>
    bool operator()(const T* const pm1, const T* const pp1, T* residuals) const {
        // Get both P1 and P2 in Eigen form for easy processing
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pip1(pp1);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pim1(pm1);

        auto diff = pip1 - pim1;

        // Set residuals to the error in each axis
        residuals[0] = pos_weight(0) * (diff(0));
        residuals[1] = pos_weight(1) * (diff(1));
        residuals[2] = pos_weight(2) * (diff(2));

        return true;
    }

    static ControlPointOverlapCostFunction* Create(Eigen::Vector3d w){
        return new ControlPointOverlapCostFunction(new ControlPointOverlapConstraint(w));
    }

    const Eigen::Vector3d pos_weight;
};

// Position cost function, we essentially use this to fix our start/end states (or any waypoints)
// so the optimizer does not attempt to move the start/end positions of our bspline without good 
// reason to do so (either to drastically reduce acceleration or collision costs)
// Without this the optimizer would simply just put all the control points on top of each other, 
// as that would result in the lowest collision and acceleration costs
struct PositionConstraint{
    typedef ceres::AutoDiffCostFunction<PositionConstraint, 3, 3> PositionCostFunction;

    PositionConstraint(Eigen::Vector3d target, Eigen::Vector3d weight, double tol) :
        target_pos(target), pos_weight(weight), pos_tolerance(tol) {}

    template <typename T>
    bool operator()(const T* const point, T* residuals) const {
        
        // We formulate this as the square error, so if these constrained parts move, there better be a damned good reason!
        residuals[0] = (target_pos(0) - point[0]) * pos_weight(0);
        residuals[1] = (target_pos(1) - point[1]) * pos_weight(1);
        residuals[2] = (target_pos(2) - point[2]) * pos_weight(2);

        return true;
    }

    static PositionCostFunction* Create(Eigen::Vector3d target_pos, Eigen::Vector3d pos_weight, double pos_tolerance){
        return new PositionCostFunction(new PositionConstraint(target_pos, pos_weight, pos_tolerance));
    }

    const Eigen::Vector3d target_pos;       // Target position
    const Eigen::Vector3d pos_weight;       // Weighting
    const double pos_tolerance;             // Unused
};

// Time constraint. We simply want to minimize the time taken for the trajectory, so the optimizer
// doesn't just stretch out the time taken to meet acceleration/velocity constraints. I'm not sure
// if this is still useful, but I will leave it in case it is
struct TimeConstraint{
    typedef ceres::AutoDiffCostFunction<TimeConstraint, 1, 1> TimeCostFunction;

    TimeConstraint(double t_c) : 
        time_cost(t_c) {}

    template <typename T>
    bool operator()(const T* const t, T* residuals) const{
        residuals[0] = time_cost * t[0];

        return true;
    }

    static TimeCostFunction* Create(double t_c){
        return new TimeCostFunction(new TimeConstraint(t_c));
    }

    const double time_cost;
};

// Constraint on the maximum vertical deviation so the trajectory is always contained within the vfov of the sensor
struct VFOVConstraint{
    // This has to be numerically differentiated, otherwise the discontinuous region of zero cost can't be implemented.
    typedef ceres::AutoDiffCostFunction<VFOVConstraint, 1, 3, 3> VFOVCostFunction;

    VFOVConstraint(double cost, double vfov) : 
        tan_fov(tan(vfov / 2)),
        cost(cost) {}

    template <typename T>
    bool operator()(const T* const p1, const T* const p2, T* residuals) const{
        Eigen::Map<const Eigen::Matrix<T, 2, 1>> p1_xy(p1);
        Eigen::Map<const Eigen::Matrix<T, 2, 1>> p2_xy(p2);

        T dist = (p2_xy - p1_xy).norm();
        T absh = abs(p2[2] - p1[2]);
        T tantheta = absh / dist;

        residuals[0] = cost * ((tantheta > tan_fov) ? (tantheta - tan_fov) * (tantheta - tan_fov) : T(0));

        return true;
    }

    static VFOVCostFunction* Create(double cost, double vfov){
        return new VFOVCostFunction(new VFOVConstraint(cost, vfov));
    }

    const double tan_fov;
    const double cost;
};

// Constraint on the maximum yaw deviation so the trajectory stays in the FOV of the sensor
struct HFOVConstraint{
    typedef ceres::NumericDiffCostFunction<HFOVConstraint, ceres::CENTRAL, 1, 3, 3, 3> HFOVCostFunction;

    HFOVConstraint(double cost, double hfov) :
        hfov(hfov),
        cos_fov(cos(hfov / 2)),
        tan_fov(tan(hfov / 2)),
        cost(cost) {}

    //template <typename T>
    bool operator()(const double* const p1, const double* const p2, const double* const p3, double* residuals) const{
        Eigen::Map<const Eigen::Matrix<double, 2, 1>> p1_xy(p1);
        Eigen::Map<const Eigen::Matrix<double, 2, 1>> p2_xy(p2);
        Eigen::Map<const Eigen::Matrix<double, 2, 1>> p3_xy(p3);

        Eigen::Matrix<double, 2, 1> diff1 = (p2_xy - p1_xy);
        Eigen::Matrix<double, 2, 1> diff2 = (p3_xy - p2_xy);
        auto proj_a = diff1.dot(diff2) / diff1.dot(diff1) * diff1;
        auto proj_b = diff2 - proj_a;

        double costheta = diff1.dot(diff2) / (diff1.norm() * diff2.norm());
        //T acostheta = acos(costheta);

        //residuals[0] = cost * ((costheta < cos_fov) ? (acos(costheta) - hfov) * (acos(costheta) - hfov) : 0);
        auto tanb = proj_b.norm() * tan_fov;
        

        //double res = (costheta > M_PI_2) ? (2.0 * diff2.norm()) - proj_b.norm() - tanb : proj_b.norm() - tanb;
        double res = costheta - cos_fov;
        residuals[0] = cost * ((costheta > cos_fov) ? res * res : double(0));//((costheta > cos_fov) ? (proj_b.norm()/proj_a.norm()) : T(0)); 

        return true;
    }

    static HFOVCostFunction* Create(double cost, double hfov){
        return new HFOVCostFunction(new HFOVConstraint(cost, hfov));
    }

    const double hfov;
    const double cos_fov;
    const double tan_fov;
    const double cost;
};

// Optimize only a single bspline. This is kept for legacy purposes
bspline_opt::bspline_opt(bspline* spline, fiesta::ESDFMap* map, struct bspline_opt_params params){
    m_bspline_multi.clear();
    m_bspline_multi.push_back(spline);
    m_map = map;
    m_params = params;

    // Assume we want a zero start/end vel unless otherwise loaded
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    set_termination_velocity_states(zero, zero);
}

// Optimize a set of continuous bsplines with joined ends, guaranteeing continunity.
// Note: list of provided bsplines *MUST* be in order and joined end to end, otherwise weird things may happen.
bspline_opt::bspline_opt(vector<bspline*> splines, fiesta::ESDFMap* map, struct bspline_opt_params params){
    m_bspline_multi = splines;
    m_map = map;
    m_params = params;

    // Assume we want a zero start/end vel unless otherwise loaded
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    set_termination_velocity_states(zero, zero);
}

// Actual trajectory optimization code. Should be pretty self explanatory
// This is the single bspline version, which is only kept for legacy purposes
void bspline_opt::trajectory_optimize(vector<Eigen::Vector3d>* control_points, double* interval, int order){
    ceres::Problem problem;

    // Add start and end constraints
    size_t points_end = (*control_points).size() - 1;
    for(size_t i = 0; i < 1; ++i){
        //auto start_state_cost_0 = PositionConstraint::Create((*control_points)[i], m_params.start_end_position_weight, m_params.start_end_position_tolerance);
        //problem.AddResidualBlock(start_state_cost_0, NULL, (*control_points)[i].data());
        auto end_state_cost_0 = PositionConstraint::Create((*control_points)[points_end - i], m_params.start_end_position_weight, m_params.start_end_position_tolerance);
        problem.AddResidualBlock(end_state_cost_0, NULL, (*control_points)[points_end - i].data());
    }

    // Add collision cost
    for(size_t i = 1; i < control_points->size(); ++i){
        auto collision_cost = new CollisionCostConstraint(m_map, m_params.collision_min_distance, m_params.collision_weight);
        problem.AddResidualBlock(collision_cost, NULL, (*control_points)[i].data());
    }

    // Add time cost
    auto time_cost = TimeConstraint::Create(m_params.time_cost);
    problem.AddResidualBlock(time_cost, NULL, interval);

    // Add initial acceleration cost
    auto initial_acceleration_cost = InitialAccelerationConstraint::Create(m_params.acceleration_weight, m_params.max_acc, m_params.max_acc_weight, (*control_points)[0], order, 1, 2);
    problem.AddResidualBlock(initial_acceleration_cost, NULL, (*control_points)[1].data(), (*control_points)[2].data(), interval);

    // Add acceleration cost
    for(size_t i = 2; i < control_points->size() - 1; ++i){
        double int_l, int_h;
        if(i < order){
            int_l = (i < order) ? i : order;
            int_h = (i - 1 < order) ? i + 1 : order;
        }else{
            int_l = (i > (control_points->size() - order)) ? (control_points->size() - i) : order;
            int_h = (i > (control_points->size() - order - 1)) ? (control_points->size() - i - 1) : order;
        }
        DEBUG_PRINT("Adding acc cost size: %lu, i: %lu, order: %i, mul_l: %f, mul_h: %f\n", control_points->size(), i, order, int_l, int_h);
        auto acceleration_cost = AccelerationConstraint::Create(m_params.acceleration_weight, m_params.max_acc, m_params.max_acc_weight, order, int_l, int_h);
        problem.AddResidualBlock(acceleration_cost, NULL, (*control_points)[i-1].data(), (*control_points)[i].data(), (*control_points)[i+1].data(), interval);
    }

    // Add velocity cost
    for(size_t i = 0; i < control_points->size() - 1; ++i){
        double int_mul;
        if(i < order){
            int_mul = i + 1;
        }else{
            int_mul = (i > (control_points->size() - order - 1)) ? (control_points->size() - i - 1) : order;
        }

        DEBUG_PRINT("Adding vel cost size: %lu, i: %lu, order: %i, mul: %f\n", control_points->size(), i, order, int_mul);
        auto velocity_cost = SpeedConstraint::Create(m_params.velocity_weight(0), m_params.velocity_target(0), m_params.max_vel, m_params.max_vel_weight, order, int_mul);
        problem.AddResidualBlock(velocity_cost, NULL, (*control_points)[i].data(), (*control_points)[i+1].data(), interval);
    }
    
    // Constrain initial and final velocities
    // We use a super high velocity weight here to make sure that we overpower any normal acceleration weights
    //auto initial_velocity_cost = VelocityStateConstraint::Create(m_params.velocity_weight * 10000, m_start_vel, m_params.max_vel, m_params.max_vel_weight, order, 1);
    //auto final_velocity_cost = VelocityStateConstraint::Create(m_params.velocity_weight * 10000, m_end_vel, m_params.max_vel, m_params.max_vel_weight, order, 1);
    auto initial_velocity_cost = InitialVelocityStateConstraint::Create(m_params.velocity_weight * 10000, m_start_vel, (*control_points)[0], order, 1);
    auto final_velocity_cost = VelocityStateConstraint::Create(m_params.velocity_weight * 10000, m_end_vel, m_params.max_vel, m_params.max_vel_weight, order, 1);
    problem.AddResidualBlock(initial_velocity_cost, NULL, (*control_points)[1].data(), interval);
    problem.AddResidualBlock(final_velocity_cost, NULL, (*control_points)[points_end - 1].data(), (*control_points)[points_end].data(), interval);


    // SPARSE_NORMAL_CHOLESKY seems to be working wonders for me.
    // Alternatively line search could be used, but appear to take about 20x more steps (with each step being significantly faster)
    // We also raise te function tolerance to 1e-5 to produce faster solves. We don't need the true "minimum", just something that's
    // much better than what we had initially
    ceres::Solver::Options options;
    if(PRINT_SOLVER_STATUS) options.minimizer_progress_to_stdout = true;    // Print solver status if flag is set
    //options.minimizer_type = ceres::LINE_SEARCH;                          // Levenberg-Marquant is significantly faster than LBGFS
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;             // Speedi solver for our sparse problem
    options.max_num_iterations = 1000;                                      // TODO: Figure out what this should really be set to
    options.function_tolerance = 1e-5;                                      // Can probably set this up as a tuning factor (time vs quality).
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;
}

// Actual trajectory optimization code. Should be pretty self explanatory
// This variation optimizes all control points in all provided bsplines at once
bool bspline_opt::trajectory_optimize(vector<vector<Eigen::Vector3d>>* control_points_vec, vector<double> &intervals, int order){
    ceres::Problem problem;

    for(size_t j = 0; j < control_points_vec->size(); ++j){
        double* interval = &intervals[j];
        auto control_points = &(*control_points_vec)[j];
        // Add start and end constraints
        size_t points_end = (*control_points).size() - 1;
        for(size_t i = 0; i < 1; ++i){
            //auto start_state_cost_0 = PositionConstraint::Create((*control_points)[i], m_params.start_end_position_weight, m_params.start_end_position_tolerance);
            //problem.AddResidualBlock(start_state_cost_0, NULL, (*control_points)[i].data());
            auto end_state_cost_0 = PositionConstraint::Create((*control_points)[points_end - i], m_params.start_end_position_weight, m_params.start_end_position_tolerance);
            problem.AddResidualBlock(end_state_cost_0, NULL, (*control_points)[points_end - i].data());
        }

        // Add collision cost
        for(size_t i = 1; i < control_points->size(); ++i){
            //auto collision_cost = CollisionCostNumeric::Create(m_map, m_params.collision_min_distance, m_params.collision_weight);
            auto collision_cost = new CollisionCostConstraint(m_map, m_params.collision_min_distance, m_params.collision_weight);
            
            problem.AddResidualBlock(collision_cost, NULL, (*control_points)[i].data());
        }

        // Add time cost
        auto time_cost = TimeConstraint::Create(m_params.time_cost);
        problem.AddResidualBlock(time_cost, NULL, interval);

        // Add initial acceleration cost only for the first bspline to avoid moving the starting position
        // of the generated trajectory.
        if(j < 1){
            auto initial_acceleration_cost = InitialAccelerationConstraint::Create(m_params.acceleration_weight, m_params.max_acc, m_params.max_acc_weight, (*control_points)[0], order, 1, 2);
            problem.AddResidualBlock(initial_acceleration_cost, NULL, (*control_points)[1].data(), (*control_points)[2].data(), interval);
        }

        // Add acceleration cost. We optimize the first control point too when optimizing every bspline past the first one.
        for(size_t i = (j > 0) ? 1 : 2; i < control_points->size() - 1; ++i){
            double int_l, int_h;
            if(i < order){
                int_l = (i < order) ? i : order;
                int_h = (i - 1 < order) ? i + 1 : order;
            }else{
                int_l = (i > (control_points->size() - order)) ? (control_points->size() - i) : order;
                int_h = (i > (control_points->size() - order - 1)) ? (control_points->size() - i - 1) : order;
            }
            DEBUG_PRINT("Adding acc cost size: %lu, i: %lu, order: %i, mul_l: %f, mul_h: %f\n", control_points->size(), i, order, int_l, int_h);
            auto acceleration_cost = AccelerationConstraint::Create(m_params.acceleration_weight, m_params.max_acc, m_params.max_acc_weight, order, int_l, int_h);
            problem.AddResidualBlock(acceleration_cost, NULL, (*control_points)[i-1].data(), (*control_points)[i].data(), (*control_points)[i+1].data(), interval);
        }

        // Add velocity cost. We also add a velocity constraint at the start if 
        // it's not the first sample.
        for(size_t i = (j > 0) ? 0 : 1; i < control_points->size() - 1; ++i){
            double int_mul;
            if(i < order){
                int_mul = i + 1;
            }else{
                int_mul = (i > (control_points->size() - order - 1)) ? (control_points->size() - i - 1) : order;
            }

            DEBUG_PRINT("Adding vel cost size: %lu, i: %lu, order: %i, mul: %f\n", control_points->size(), i, order, int_mul);
            auto velocity_cost = SpeedConstraint::Create(m_params.velocity_weight(0), m_params.velocity_target(0), m_params.max_vel, m_params.max_vel_weight, order, int_mul);
            problem.AddResidualBlock(velocity_cost, NULL, (*control_points)[i].data(), (*control_points)[i+1].data(), interval);
            auto max_velocity_cost = MaxSpeedConstraint::Create(m_params.velocity_weight(0), m_params.max_vel, m_params.max_vel_weight, order, int_mul);
            problem.AddResidualBlock(max_velocity_cost, NULL, (*control_points)[i].data(), (*control_points)[i+1].data(), interval);
        }

        // Impose VFOV constraints on everything except first point
        if(m_params.vfov_enabled){
            for(size_t i = (j > 0) ? 0 : 1; i < control_points->size() - 1; ++i){
                auto vfov_cost = VFOVConstraint::Create(m_params.vfov_cost, m_params.vfov);
                problem.AddResidualBlock(vfov_cost, NULL, (*control_points)[i].data(), (*control_points)[i+1].data());
            }
        }
        
        if(m_params.hfov_enabled){
            for(size_t i = (j > 0) ? 0 : 1; i < control_points->size() - 4; ++i){
                auto hfov_cost = HFOVConstraint::Create(m_params.hfov_cost, m_params.hfov);
                problem.AddResidualBlock(hfov_cost, NULL, (*control_points)[i].data(), (*control_points)[i+2].data(), (*control_points)[i+4].data());
            }
        }

        // Impose continunity constraints between adjacent bsplines.
        if(j > 0){
            // Get pointer to the last set of bsplines for lazy processing
            vector<Eigen::Vector3d>* last_control_points = &(*control_points_vec)[j - 1];
            auto control_point_position_cost = ControlPointOverlapConstraint::Create(m_params.start_end_position_weight * 100000);
            problem.AddResidualBlock(control_point_position_cost, NULL, (*last_control_points)[last_control_points->size() - 1].data(), (*control_points)[0].data());
            auto splice_cost = SpliceAccelerationConstraint::Create(m_params.acceleration_weight, m_params.max_acc, m_params.max_acc_weight, order, 2, 2);
            problem.AddResidualBlock(splice_cost, NULL, (*last_control_points)[last_control_points->size() - 2].data(), (*last_control_points)[last_control_points->size() - 1].data(), (*control_points)[1].data(), &intervals[j-1], &intervals[j]);
            auto velocity_constraint_cost = DiffVelocityStateConstraint::Create(m_params.velocity_weight * 100000, order);
            problem.AddResidualBlock(velocity_constraint_cost, NULL, (*last_control_points)[last_control_points->size() - 2].data(), (*last_control_points)[last_control_points->size() - 1].data(), (*control_points)[1].data(), &intervals[j-1], &intervals[j]);

            if(m_params.hfov_enabled){
                auto hfov_cost = HFOVConstraint::Create(m_params.hfov_cost, m_params.hfov);
                problem.AddResidualBlock(hfov_cost, NULL, (*last_control_points)[last_control_points->size() - 3].data(), (*last_control_points)[last_control_points->size() - 1].data(), (*control_points)[2].data());
                // auto hfov_cost1 = HFOVConstraint::Create(m_params.hfov_cost, m_params.hfov);
                // problem.AddResidualBlock(hfov_cost1, NULL, (*last_control_points)[last_control_points->size() - 2].data(), (*control_points)[2].data(), (*control_points)[4].data());
            }
        }
    }

    // Constrain initial and final velocities
    // We use a super high velocity weight here to make sure that we overpower any normal acceleration weights
    //auto initial_velocity_cost = VelocityStateConstraint::Create(m_params.velocity_weight * 10000, m_start_vel, m_params.max_vel, m_params.max_vel_weight, order, 1);
    //auto final_velocity_cost = VelocityStateConstraint::Create(m_params.velocity_weight * 10000, m_end_vel, m_params.max_vel, m_params.max_vel_weight, order, 1);
    auto initial_velocity_cost = InitialVelocityStateConstraint::Create(m_params.velocity_weight * 10000, m_start_vel, (*control_points_vec)[0][0], order, 1);
    auto final_velocity_cost = VelocityStateConstraint::Create(m_params.velocity_weight * 10000, m_end_vel, m_params.max_vel, m_params.max_vel_weight, order, 1);
    problem.AddResidualBlock(initial_velocity_cost, NULL, (*control_points_vec)[0][1].data(), &intervals[0]);
    auto last_bspline = &(*control_points_vec)[control_points_vec->size() - 1];
    problem.AddResidualBlock(final_velocity_cost, NULL, (*last_bspline)[last_bspline->size() - 2].data(), (*last_bspline)[last_bspline->size() - 1].data(), &intervals[intervals.size() - 1]);


    // SPARSE_NORMAL_CHOLESKY seems to be working wonders for me.
    // Alternatively line search could be used, but appear to take about 20x more steps (with each step being significantly faster)
    // We also raise the function tolerance to 1e-5 to produce faster solves. We don't need the true "minimum", just something that's
    // much better than what we had initially
    ceres::Solver::Options options;
    if(PRINT_SOLVER_STATUS) options.minimizer_progress_to_stdout = true;    // Print solver status if flag is set
    //options.minimizer_type = ceres::LINE_SEARCH;                          // Levenberg-Marquant is significantly faster than LBGFS
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;             // Speedi solver for our sparse problem
    options.use_explicit_schur_complement = true;
    options.max_num_iterations = 1000;                                      // TODO: Figure out what this should really be set to
    options.function_tolerance = 1e-5;                                      // Can probably set this up as a tuning factor (time vs quality).
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;
    if(summary.termination_type != ceres::TerminationType::CONVERGENCE){
        for(size_t j = 0; j < control_points_vec->size(); ++j){
            printf("Bspline %i\n", j);
            for(size_t i = 0; i < (*control_points_vec)[j].size(); ++i){
                printf("%f, %f, %f\n", (*control_points_vec)[j][i](0), (*control_points_vec)[j][i](1), (*control_points_vec)[j][i](2));
            }
        }
        return false;
    }
    return true;
}

void bspline_opt::set_termination_velocity_states(Eigen::Vector3d start_vel, Eigen::Vector3d end_vel){
    m_start_vel = start_vel;
    m_end_vel = end_vel;
}

void bspline_opt::set_target_velocity_vector(Eigen::Vector3d target_vel){
    m_params.velocity_target = target_vel;
}

OPTIMIZATION_STATUS bspline_opt::optimize(){
    vector<vector<Eigen::Vector3d>> point_vec;
    vector<double> intervals;

    auto ret = OPT_OK;

    for(auto spline : m_bspline_multi){
        // Fail if bspline is nullptr
        if(spline == NULL) return OPT_ERROR_NULLPTR; 
        // Fail if bspline is too short
        auto control_points = spline->get_control_points();
        if(control_points.size() < 4) return OPT_ERROR_BSPLINE_TOOSHORT;
        if(m_params.velocity_target.norm() < 0.1) return OPT_ERROR_TARGET_VEL_ZERO;

        // Get all control points as a C++ vector for lazy processing
        vector<Eigen::Vector3d> points;
        for(size_t i = 0; i < control_points.rows(); ++i){
            points.push_back(control_points.row(i));
        }

        point_vec.push_back(points);
        intervals.push_back(spline->get_interval());
    }

    printf("Got %i B-Splines for optimization\n", point_vec.size());

    // Get interval for velocity/acceleration processing
    int order = m_bspline_multi[0]->get_order();

    // Optimize
    if(!trajectory_optimize(&point_vec, intervals, order)){
        ret = OPT_ERROR_FAILED;
    }

    for(size_t j = 0; j < m_bspline_multi.size(); ++j){
        auto points = point_vec[j];
        auto control_points = m_bspline_multi[j]->get_control_points();
        // Write results back to the original bspline
        for(size_t i = 0; i < points.size(); ++i){
            control_points.row(i) = points[i];
        }

        m_bspline_multi[j]->set_control_points(control_points);
        m_bspline_multi[j]->set_interval(intervals[j]);
    }

    // Debug print
    if(DUMP_CONTROLPOINT_KNOT){
        printf("Interval: %f\n", intervals[0]);

        for(auto points : point_vec){
            for(auto p : points){
                printf("control points: %f, %f, %f\n", p(0), p(1), p(2));
            }
        }
    }

    return ret;
}