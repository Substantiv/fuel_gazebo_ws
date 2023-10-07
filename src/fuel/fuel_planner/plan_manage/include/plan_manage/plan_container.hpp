#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <bspline/non_uniform_bspline.h>
#include <poly_traj/polynomial_traj.h>
#include <active_perception/traj_visibility.h>

using std::vector;
using std::shared_ptr;

namespace fast_planner {
    struct PlanParameters {
        /* planning algorithm parameters */
        double max_vel_{}, max_acc_{}, max_jerk_{};  // physical limits
        double accept_vel_{}, accept_acc_{};

        double max_yawdot_{};
        double local_traj_len_{};  // local replanning trajectory length
        double ctrl_pt_dist{};     // distance between adjacient B-spline control points
        int bspline_degree_{};
        bool min_time_{};

        double clearance_{};
        int dynamic_{};
    };

    struct LocalTrajData {
        /* info of generated traj */
        int traj_id_{};
        double duration_{};
        ros::Time start_time_;
        NonUniformBspline pos_traj_, vel_traj_, acc_traj_;
        NonUniformBspline yaw_traj_, yawdot_traj_, yawdotdot_traj_;
        void culcDerivatives() {
            vel_traj_ = pos_traj_.getDerivative();
            acc_traj_ = vel_traj_.getDerivative();
            yawdot_traj_ = yaw_traj_.getDerivative();
            yawdotdot_traj_ = yawdot_traj_.getDerivative();
        }
    };
    typedef shared_ptr<LocalTrajData> LocalTrajDataPtr;
}  // namespace fast_planner

#endif