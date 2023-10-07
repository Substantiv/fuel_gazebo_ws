#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <utility>
#include <string>
#include <unordered_map>
#include "plan_env/edt_environment.h"
#include <boost/functional/hash.hpp>
#include <queue>

namespace fast_planner {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<int, 6, 1> Vector6i;

    class PathNode {
    public:
        /* -------------------- */
        Vector6i discretized_state;
        Eigen::Matrix<double, 6, 1> state;
        double g_score{}, f_score{};
        Eigen::Vector3d input;
        double duration{};
        double time{};  // dyn
        shared_ptr<PathNode> parent;

        /* -------------------- */
        PathNode() {
            parent = nullptr;
            input = Eigen::Vector3d::Zero();
        }

        PathNode(Vector6i index_,
                 Vector6d state_,
                 double g_score_,
                 double f_score_,
                 Eigen::Vector3d input_,
                 double duration_,
                 shared_ptr<PathNode> parent_) {
            discretized_state = std::move(index_);
            state = std::move(state_);
            g_score = g_score_;
            f_score = f_score_;
            input = std::move(input_);
            duration = duration_;
            parent = std::move(parent_);
        }

        ~PathNode() = default;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef shared_ptr<PathNode> PathNodePtr;

    typedef Eigen::Matrix<double,3,4> Matrix34;

    class KinodynamicAstar {
    private:
        /* ---------- main data structure ---------- */

        /* ---------- record data ---------- */
        EDTEnvironment::Ptr edt_environment_;

        /* ---------- parameter ---------- */
        /* search */
        double max_tau_{}, init_max_tau_{};
        double max_vel_{}, max_acc_{};
        double w_time_{}, horizon_{}, lambda_heu_{};
        int allocate_num_{}, check_num_{};
        bool optimistic_{};

        /* map */
        double resolution_{}, time_resolution_{};
        Eigen::Vector3d origin_, map_size_3d_;
        double time_origin_{};

        /* helper */
        Vector6i discretizeState(const Vector6d &state);

        static vector<PathNodePtr> retrievePath(PathNodePtr cur_node);

        /* shot trajectory */
        static vector<double> cubic(double a, double b, double c, double d);

        static vector<double> quartic(double a, double b, double c, double d, double e);

        bool computeShotTraj(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2, double time_to_goal,
                             Matrix34 &coef_shot);

        double estimateHeuristic(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, double &optimal_time) const;

    public:
        KinodynamicAstar(ros::NodeHandle &nh, const EDTEnvironment::Ptr &env);

        enum {
            REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4
        };

        /* main API */
        int search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_v, const Eigen::Vector3d &start_a,
                   const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_v, bool init_search,
                   vector<PathNodePtr> &path, bool &is_shot_succ, Matrix34 &coef_shot, double &shot_time);

        static void getSamples(const vector<PathNodePtr> &path, const Eigen::Vector3d &start_v, const Eigen::Vector3d &end_v,
                               bool is_shot_succ, const Matrix34 &coef_shot, double t_shot,
                               double &ts, vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivatives);

        typedef shared_ptr<KinodynamicAstar> Ptr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}  // namespace fast_planner

#endif