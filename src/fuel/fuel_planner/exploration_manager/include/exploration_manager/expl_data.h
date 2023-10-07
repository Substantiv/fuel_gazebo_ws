#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <bspline/Bspline.h>

using std::vector;
using Eigen::Vector3d;

namespace fast_planner {
    struct FSMData {
        // FSM data
        bool have_odom_, static_state_;
        vector<string> state_str_;

        Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
        Eigen::Quaterniond odom_orient_;
        double odom_yaw_;
    };

    struct FSMParam {
        double replan_thresh1_;
        double replan_thresh2_;
        double replan_thresh3_;
        double replan_time_;  // second
    };

    struct ExplorationParam {
        // params
        bool refine_local_;
        int refined_num_;
        double refined_radius_;
        int top_view_num_;
        double max_decay_;
        string tsp_dir_;  // resource dir of tsp solver
        double relax_time_;
    };

}  // namespace fast_planner

#endif