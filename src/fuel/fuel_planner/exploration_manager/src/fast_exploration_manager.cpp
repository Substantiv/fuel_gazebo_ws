// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

    FastExplorationManager::~FastExplorationManager() {
        ViewNode::astar_.reset();
        ViewNode::caster_.reset();
        ViewNode::map_.reset();
    }

    FastExplorationManager::FastExplorationManager(ros::NodeHandle &nh) {
        planner_manager_.reset(new FastPlannerManager(nh));
        edt_environment_ = planner_manager_->edt_environment_;
        sdf_map_ = edt_environment_->sdf_map_;
        frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));

        ep_.reset(new ExplorationParam);

        nh.param("exploration/refine_local", ep_->refine_local_, true);
        nh.param("exploration/refined_num", ep_->refined_num_, -1);
        nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
        nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
        nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
        nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
        nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

        nh.param("exploration/vm", ViewNode::vm_, -1.0);
        nh.param("exploration/am", ViewNode::am_, -1.0);
        nh.param("exploration/yd", ViewNode::yd_, -1.0);
        nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
        nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

        ViewNode::astar_.reset(new Astar(nh, edt_environment_));
        ViewNode::map_ = sdf_map_;

        double resolution_ = sdf_map_->getResolution();
        Eigen::Vector3d origin, size;
        sdf_map_->getRegion(origin, size);
        ViewNode::caster_.reset(new RayCaster);
        ViewNode::caster_->setParams(resolution_, origin);

        planner_manager_->astar_path_finder_->lambda_heu_ = 1.0;
        planner_manager_->astar_path_finder_->max_search_time_ = 1.0;

        // Initialize TSP par file
        ofstream par_file(ep_->tsp_dir_ + "/single.par");
        par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
        par_file << "GAIN23 = NO\n";
        par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
        par_file << "RUNS = 1\n";
    }

    int FastExplorationManager::planExplore(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                            const Vector3d &cur_acc, const Vector3d &cur_yaw,
                                            Vector3d &next_pos, double &next_yaw) {
        // Search frontiers and group them into clusters
        frontier_finder_->removeOutDatedFrontiers();
        frontier_finder_->searchAndAddFrontiers();

        // Find viewpoints (x,y,z,cur_yaw) for all frontier clusters and get visible ones' local_traj_data
        if (frontier_finder_->frontiers_.empty()) {
            ROS_WARN("No coverable frontier.");
            return NO_FRONTIER;
        }
        vector<Eigen::Vector3d> mid_points;
        vector<double> mid_points_yaw;
        frontier_finder_->getTopViewpointsInfo(cur_pos, mid_points, mid_points_yaw);

        // Do global and local tour planning and retrieve the next viewpoint
        if (mid_points.size() > 1) {
            // Find the global tour passing through all viewpoints
            // Create TSP and solve by LKH
            // Optimal tour is returned as frontier_sequence of frontier
            vector<int> frontier_sequence;
            findGlobalTour(cur_pos, cur_vel, cur_yaw, frontier_sequence);

            if (ep_->refine_local_) {
                // Do refinement for the next few viewpoints in the global tour
                // Idx of the first K frontier in optimal tour
                int k_num = min(int(frontier_sequence.size()), ep_->refined_num_);
                vector<int> first_k_frontier_ids;
                for (int i = 0; i < k_num; ++i) {
                    Vector3d tmp = mid_points[frontier_sequence[i]];
                    first_k_frontier_ids.push_back(frontier_sequence[i]);
                    if ((tmp - cur_pos).norm() > ep_->refined_radius_ && first_k_frontier_ids.size() >= 2) break;
                }

                vector<vector<Vector3d>> n_points;
                vector<vector<double>> n_yaws;
                frontier_finder_->getNViewPoints(
                        cur_pos, first_k_frontier_ids, ep_->top_view_num_, ep_->max_decay_, n_points, n_yaws);

                vector<Vector3d> refined_points;
                vector<double> refined_yaws;
                refineLocalTour(cur_pos, cur_vel, cur_yaw, n_points, n_yaws, refined_points, refined_yaws);
                next_pos = refined_points[0];
                next_yaw = refined_yaws[0];
            } else {
                // Choose the next viewpoint from global tour
                next_pos = mid_points[frontier_sequence[0]];
                next_yaw = mid_points_yaw[frontier_sequence[0]];
            }
        } else if (mid_points.size() == 1) {
            // Only 1 destination, no need to find global tour through TSP
            if (ep_->refine_local_) {
                // Find the min cost viewpoint for next frontier
                vector<vector<Vector3d>> n_points;
                vector<vector<double>> n_yaws;
                frontier_finder_->getNViewPoints(
                        cur_pos, {0}, ep_->top_view_num_, ep_->max_decay_, n_points, n_yaws);

                double min_cost = 100000;
                int min_cost_id = -1;
                vector<Vector3d> tmp_path;
                for (int i = 0; i < n_points[0].size(); ++i) {
                    auto tmp_cost = ViewNode::computeCost(
                            cur_pos, n_points[0][i], cur_yaw[0], n_yaws[0][i], cur_vel, cur_yaw[1], tmp_path);
                    if (tmp_cost < min_cost) {
                        min_cost = tmp_cost;
                        min_cost_id = i;
                    }
                }
                next_pos = n_points[0][min_cost_id];
                next_yaw = n_yaws[0][min_cost_id];
            } else {
                next_pos = mid_points[0];
                next_yaw = mid_points_yaw[0];
            }
        } else
            ROS_ERROR("Empty destination.");

        return SUCCEED;
    }

    int FastExplorationManager::planMotion(
            const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_acc, const Eigen::Vector3d &cur_yaw,
            const Eigen::Vector3d &next_pos, const double next_yaw) {
        double diff = fabs(next_yaw - cur_yaw[0]);
        double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

        planner_manager_->astar_path_finder_->reset();
        if (planner_manager_->astar_path_finder_->search(cur_pos, next_pos) != Astar::REACH_END) {
            ROS_ERROR("No path to next viewpoint");
            return FAIL;
        }
        auto path_to_next_goal = planner_manager_->astar_path_finder_->getPath();
        shortenPath(path_to_next_goal);

        const double radius_far = 5.0;
        const double radius_close = 1.5;
        const double full_path_len = Astar::pathLength(path_to_next_goal);
        if (full_path_len < radius_close) {
            // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based optimization
            planner_manager_->planExploreTraj(path_to_next_goal, cur_vel, cur_acc, time_lb);
        } else if (full_path_len > radius_far) {
            // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with dead end)
            double len2 = 0.0;
            vector<Eigen::Vector3d> truncated_path = {path_to_next_goal.front()};
            for (size_t i = 1; i < path_to_next_goal.size() && len2 < radius_far; ++i) {
                auto cur_pt = path_to_next_goal[i];
                len2 += (cur_pt - truncated_path.back()).norm();
                truncated_path.push_back(cur_pt);
            }
            planner_manager_->planExploreTraj(truncated_path, cur_vel, cur_acc, time_lb);
        } else {
            // Search kino path to exactly next viewpoint and optimize
            cout << "\n\n\n\n\n\n\n\n0000000000000000000000000\n0000000000000000000000000\n\n\n\n\n\n";
            if (!planner_manager_->kinodynamicReplan(cur_pos, cur_vel, cur_acc, next_pos, Vector3d(0, 0, 0), time_lb))
                return FAIL;
        }

        if (planner_manager_->local_data_->pos_traj_.getTimeSum() < time_lb - 0.1)
            ROS_ERROR("Lower bound not satified!");

        LocalTrajDataPtr local_traj_data = planner_manager_->local_data_;
        local_traj_data->duration_ = local_traj_data->pos_traj_.getTimeSum();
        planner_manager_->planYawExplore(cur_yaw, next_yaw, local_traj_data->pos_traj_,
                                         local_traj_data->duration_, ep_->relax_time_);

        local_traj_data->culcDerivatives();
        local_traj_data->start_time_ = ros::Time::now();
        local_traj_data->traj_id_ += 1;

        return SUCCEED;
    }

    void FastExplorationManager::shortenPath(vector<Vector3d> &path) {
        if (path.empty()) {
            ROS_ERROR("Empty path to shorten");
            return;
        }
        // Shorten the tour, only critical intermediate points are reserved.
        const double dist_thresh = 3.0;
        vector<Vector3d> short_tour = {path.front()};
        for (size_t i = 1; i < path.size() - 1; ++i) {
            if ((path[i] - short_tour.back()).norm() > dist_thresh)
                short_tour.push_back(path[i]);
            else {
                // Add waypoints to shorten path only to avoid collision
                ViewNode::caster_->input(short_tour.back(), path[i + 1]);
                Eigen::Vector3i idx;
                while (ViewNode::caster_->nextId(idx) && ros::ok()) {
                    if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
                        edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
                        short_tour.push_back(path[i]);
                        break;
                    }
                }
            }
        }
        if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

        // Ensure at least three points in the path
        if (short_tour.size() == 2)
            short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
        path = short_tour;
    }

    void FastExplorationManager::findGlobalTour(
            const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw, vector<int> &indices) {
        auto t1 = ros::Time::now();

        // Get cost matrix for current state and clusters
        Eigen::MatrixXd cost_mat;
        frontier_finder_->updateFrontierCostMatrix();
        frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
        const Eigen::Index dimension = cost_mat.rows();

        double mat_time = (ros::Time::now() - t1).toSec();
        t1 = ros::Time::now();

        // Write params and cost matrix to problem file
        ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
        // Problem specification part, follow the format of TSPLIB

        string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                           "\nEDGE_WEIGHT_TYPE : "
                           "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

        prob_file << prob_spec;
        const int scale = 100;
        // Use Asymmetric TSP

        for (Eigen::Index i = 0; i < dimension; ++i) {
            for (Eigen::Index j = 0; j < dimension; ++j) {
                int int_cost = (int) cost_mat(i, j) * scale;
                prob_file << int_cost << " ";
            }
            prob_file << "\n";
        }
        prob_file.close();

        // Call LKH TSP solver
        solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

        // Read optimal tour from the tour section of result file
        ifstream res_file(ep_->tsp_dir_ + "/single.txt");
        string res;
        while (getline(res_file, res)) {
            // Go to tour section
            if (res == "TOUR_SECTION") break;
        }

        // Read path for ATSP formulation
        while (getline(res_file, res)) {
            // Read indices of frontiers in optimal tour
            int id = stoi(res);
            if (id == 1)  // Ignore the current state
                continue;
            if (id == -1) break;
            indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
        }

        double tsp_time = (ros::Time::now() - t1).toSec();
        ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
    }

    void FastExplorationManager::refineLocalTour(
            const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
            const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_yaws,
            vector<Vector3d> &refined_pts, vector<double> &refined_yaws) {
        // Create graph for viewpoints selection
        GraphSearch<ViewNode> g_search;
        vector<ViewNode::Ptr> last_group, cur_group;

        // Add the current state
        ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
        first->vel_ = cur_vel;
        g_search.addNode(first);
        last_group.push_back(first);
        ViewNode::Ptr final_node;

        // Add viewpoints
        for (size_t i = 0; i < n_points.size(); ++i) {
            // Create nodes for viewpoints of one frontier
            for (size_t j = 0; j < n_points[i].size(); ++j) {
                ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
                g_search.addNode(node);
                // Connect a node to nodes in last group
                for (const auto &nd: last_group)
                    g_search.addEdge(nd->id_, node->id_);
                cur_group.push_back(node);

                // Only keep the first viewpoint of the last local frontier
                if (i == n_points.size() - 1) {
                    final_node = node;
                    break;
                }
            }
            // Store nodes for this group for connecting edges
            last_group = cur_group;
            cur_group.clear();
        }

        // Search optimal sequence
        vector<ViewNode::Ptr> path;
        g_search.DijkstraSearch(first->id_, final_node->id_, path);

        // Return searched sequence
        for (size_t i = 1; i < path.size(); ++i) {
            refined_pts.push_back(path[i]->pos_);
            refined_yaws.push_back(path[i]->yaw_);
        }
    }

}  // namespace fast_planner
