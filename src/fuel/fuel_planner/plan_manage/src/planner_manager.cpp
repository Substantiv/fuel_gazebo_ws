// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

#include <memory>
#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <ostream>

namespace fast_planner {
// SECTION interfaces for setup and query

    FastPlannerManager::~FastPlannerManager() {
        std::cout << "des manager" << std::endl;
    }

    FastPlannerManager::FastPlannerManager(ros::NodeHandle &nh) {
        /* read algorithm parameters */

        nh.param("manager/max_vel", pp_.max_vel_, -1.0);
        nh.param("manager/max_acc", pp_.max_acc_, -1.0);
        nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
        nh.param("manager/accept_vel", pp_.accept_vel_, pp_.max_vel_ + 0.5);
        nh.param("manager/accept_acc", pp_.accept_acc_, pp_.max_acc_ + 0.5);
        nh.param("manager/max_yawdot", pp_.max_yawdot_, -1.0);
        nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
        nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
        nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
        nh.param("manager/bspline_degree", pp_.bspline_degree_, 3);
        nh.param("manager/min_time", pp_.min_time_, false);

        bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization,
                use_active_perception;
        nh.param("manager/use_geometric_path", use_geometric_path, false);
        nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
        nh.param("manager/use_topo_path", use_topo_path, false);
        nh.param("manager/use_optimization", use_optimization, false);
        nh.param("manager/use_active_perception", use_active_perception, false);

        local_data_ = make_shared<LocalTrajData>();
        local_data_->traj_id_ = 0;
        sdf_map_.reset(new SDFMap);
        sdf_map_->initMap(nh);
        edt_environment_.reset(new EDTEnvironment);
        edt_environment_->setMap(sdf_map_);

        if (use_geometric_path) {
            astar_path_finder_ = std::make_unique<Astar>(nh, edt_environment_);
        }

        if (use_kinodynamic_path) {
            kino_path_finder_ = std::make_unique<KinodynamicAstar>(nh, edt_environment_);
        }

        if (use_optimization) {
            pos_traj_bspline_optimizer = std::make_unique<BsplineOptimizer>(nh, edt_environment_);
            yaw_traj_bspline_optimizer = std::make_unique<BsplineOptimizer>(nh, edt_environment_);
        }

        if (use_active_perception) {
            visib_util_ = std::make_unique<VisibilityUtil>(nh);
            visib_util_->setEDTEnvironment(edt_environment_);
        }
    }

    bool FastPlannerManager::checkTrajCollision(double &distance) {
        double t_now = (ros::Time::now() - local_data_->start_time_).toSec();

        Eigen::Vector3d cur_pt = local_data_->pos_traj_.evaluateDeBoorT(t_now);
        double radius = 0.0;
        Eigen::Vector3d fut_pt;
        double fut_t = 0.02;

        while (radius < 6.0 && t_now + fut_t < local_data_->duration_) {
            fut_pt = local_data_->pos_traj_.evaluateDeBoorT(t_now + fut_t);
            // double dist = edt_environment_->sdf_map_->getDistance(fut_pt);
            if (sdf_map_->getInflateOccupancy(fut_pt) == 1) {
                distance = radius;
                // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " << dist << std::endl;
                std::cout << "collision at: " << fut_pt.transpose() << std::endl;
                return false;
            }
            radius = (fut_pt - cur_pt).norm();
            fut_t += 0.02;
        }

        return true;
    }

// !SECTION

// SECTION kinodynamic replanning

    bool FastPlannerManager::kinodynamicReplan(const Eigen::Vector3d &start_pt,
                                               const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                               const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel,
                                               const double &time_lb) {
        std::cout << "[Kino replan]: start: " << start_pt.transpose() << ", " << start_vel.transpose()
                  << ", " << start_acc.transpose() << ", goal:" << end_pt.transpose() << ", "
                  << end_vel.transpose() << endl;

        if ((start_pt - end_pt).norm() < 1e-2) {
            cout << "Close goal" << endl;
            return false;
        }

        /******************************
         * Kinodynamic path searching *
         ******************************/
        vector<PathNodePtr> path;
        double shot_time;
        Matrix34 coef_shot;
        bool is_shot_succ;

        int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel,
                                               true, path, is_shot_succ, coef_shot, shot_time);
        if (status == KinodynamicAstar::NO_PATH) {
            ROS_ERROR("search 1 fail");
            status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel,
                                               false, path, is_shot_succ, coef_shot, shot_time);
            if (status == KinodynamicAstar::NO_PATH) {
                cout << "[Kino replan]: Can't find path." << endl;
                return false;
            }
        }

        /*********************************
         * Parameterize path to B-spline *
         *********************************/
        double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
        vector<Eigen::Vector3d> point_set, start_end_derivatives;
        KinodynamicAstar::getSamples(path, start_vel, end_vel, is_shot_succ, coef_shot, shot_time, ts, point_set, start_end_derivatives);
        Eigen::MatrixXd ctrl_pts;
        NonUniformBspline::parameterizeToBspline(
                ts, point_set, start_end_derivatives, pp_.bspline_degree_, ctrl_pts);
        NonUniformBspline init_bspline(ctrl_pts, pp_.bspline_degree_, ts);

        /*********************************
         * B-spline-based optimization   *
         *********************************/
        int cost_function = BsplineOptimizer::NORMAL_PHASE;
        if (pp_.min_time_) cost_function |= BsplineOptimizer::MINTIME;
        vector<Eigen::Vector3d> start, end;
        init_bspline.getBoundaryStates(2, 0, start, end);
        pos_traj_bspline_optimizer->setBoundaryStates(start, end);
        if (time_lb > 0) pos_traj_bspline_optimizer->setTimeLowerBound(time_lb);
        pos_traj_bspline_optimizer->optimize(ctrl_pts, ts, cost_function, 1, 1);
        local_data_->pos_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);

        return true;
    }

    void FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3d> &tour,
                                             const Eigen::Vector3d &cur_vel, const Eigen::Vector3d &cur_acc,
                                             const double &time_lb) {
        if (tour.empty()) ROS_ERROR("Empty path to traj planner");

        // Generate traj through waypoints-based method
        const size_t pt_num = tour.size();
        Eigen::MatrixXd pos(pt_num, 3);
        for (Eigen::Index i = 0; i < pt_num; ++i) pos.row(i) = tour[i];

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd times(pt_num - 1);
        for (Eigen::Index i = 0; i < pt_num - 1; ++i)
            times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

        PolynomialTraj init_traj;
        PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

        // B-spline-based optimization
        vector<Vector3d> points, boundary_deri;
        double duration = init_traj.getTotalTime();
        int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist;
        seg_num = max(8, seg_num);
        double dt = duration / double(seg_num);

        std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt << std::endl;

        for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
            points.push_back(init_traj.evaluate(ts, 0));
        boundary_deri.push_back(init_traj.evaluate(0.0, 1));
        boundary_deri.push_back(init_traj.evaluate(duration, 1));
        boundary_deri.push_back(init_traj.evaluate(0.0, 2));
        boundary_deri.push_back(init_traj.evaluate(duration, 2));

        Eigen::MatrixXd ctrl_pts;
        NonUniformBspline::parameterizeToBspline(
                dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
        NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

        int cost_func = BsplineOptimizer::NORMAL_PHASE;
        if (pp_.min_time_) cost_func |= BsplineOptimizer::MINTIME;

        vector<Vector3d> start, end;
        tmp_traj.getBoundaryStates(2, 0, start, end);
        pos_traj_bspline_optimizer->setBoundaryStates(start, end);
        if (time_lb > 0) pos_traj_bspline_optimizer->setTimeLowerBound(time_lb);
        pos_traj_bspline_optimizer->optimize(ctrl_pts, dt, cost_func, 1, 1);

        local_data_->pos_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);
    }

    void FastPlannerManager::planYawExplore(const Eigen::Vector3d &start_yaw, const double &end_yaw,
                                            const NonUniformBspline &pos_traj, const double duration,
                                            const double &relax_time) {
        const int seg_num = 12;
        double dt_yaw = duration / seg_num;  // time of B-spline segment
        Eigen::Vector3d start_yaw3d = start_yaw;
        std::cout << "dt_yaw: " << dt_yaw << ", start yaw: " << start_yaw3d.transpose()
                  << ", end: " << end_yaw << std::endl;

        while (start_yaw3d[0] < -M_PI) start_yaw3d[0] += 2 * M_PI;
        while (start_yaw3d[0] > M_PI) start_yaw3d[0] -= 2 * M_PI;
        double last_yaw = start_yaw3d[0];

        // Yaw traj control points
        Eigen::MatrixXd yaw(seg_num + 3, 1);
        yaw.setZero();

        // Initial state
        Eigen::Matrix3d states2pts;
        states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw,
                1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
                1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
        yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

        // Add waypoint constraints if looking forward is enabled
        vector<Eigen::Vector3d> waypts;
        vector<int> waypt_idx;
        const double forward_t = 2.0;
        const int relax_num = relax_time / dt_yaw;
        for (int i = 1; i < seg_num - relax_num; ++i) {
            double tc = i * dt_yaw;
            Eigen::Vector3d pc = pos_traj.evaluateDeBoorT(tc);
            double tf = min(duration, tc + forward_t);
            Eigen::Vector3d pf = pos_traj.evaluateDeBoorT(tf);
            Eigen::Vector3d pd = pf - pc;
            Eigen::Vector3d waypt;
            if (pd.norm() > 1e-3) {
                waypt(0) = atan2(pd(1), pd(0));
                waypt(1) = waypt(2) = 0.0;
                roundYaw(last_yaw, waypt(0));
            } else
                waypt = waypts.back();

            last_yaw = waypt(0);
            waypts.push_back(waypt);
            waypt_idx.push_back(i);
        }

        // Final state
        Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
        roundYaw(last_yaw, end_yaw3d(0));
        yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

        // Debug rapid change of yaw
        if (fabs(start_yaw3d[0] - end_yaw3d[0]) >= M_PI) {
            ROS_ERROR("Yaw change rapidly!");
            std::cout << "start yaw: " << start_yaw3d[0] << ", " << end_yaw3d[0] << std::endl;
        }

        // Call B-spline optimization solver
        int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START |
                        BsplineOptimizer::END | BsplineOptimizer::WAYPOINTS;
        vector<Eigen::Vector3d> start = {Eigen::Vector3d(start_yaw3d[0], 0, 0),
                                         Eigen::Vector3d(start_yaw3d[1], 0, 0), Eigen::Vector3d(start_yaw3d[2], 0, 0)};
        vector<Eigen::Vector3d> end = {Eigen::Vector3d(end_yaw3d[0], 0, 0), Eigen::Vector3d(0, 0, 0)};
        yaw_traj_bspline_optimizer->setBoundaryStates(start, end);
        yaw_traj_bspline_optimizer->setWaypoints(waypts, waypt_idx);
        yaw_traj_bspline_optimizer->optimize(yaw, dt_yaw, cost_func, 1, 1);

        // Update traj info
        local_data_->yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
    }

    void FastPlannerManager::roundYaw(const double &last_yaw, double &yaw) {
        while (yaw - last_yaw > M_PI) {
            yaw -= 2 * M_PI;
        }
        while (yaw - last_yaw < -M_PI) {
            yaw += 2 * M_PI;
        }
    }

}  // namespace fast_planner
