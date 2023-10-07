#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
    class EDTEnvironment;

    class SDFMap;

    class FastPlannerManager;

    class FrontierFinder;

    struct ExplorationParam;
    struct ExplorationData;

    enum EXPL_RESULT {
        NO_FRONTIER, FAIL, SUCCEED
    };

    class FastExplorationManager {
    public:
        explicit FastExplorationManager(ros::NodeHandle &nh);

        ~FastExplorationManager();

        int planExplore(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_acc,
                        const Vector3d &cur_yaw, Vector3d &next_pos, double &next_yaw);

        int planMotion(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_acc,const Eigen::Vector3d &cur_yaw,
                       const Eigen::Vector3d &next_pos, const double next_yaw);

        shared_ptr<ExplorationParam> ep_;
        shared_ptr<FastPlannerManager> planner_manager_;
        shared_ptr<FrontierFinder> frontier_finder_;

    private:
        shared_ptr<EDTEnvironment> edt_environment_;
        shared_ptr<SDFMap> sdf_map_;

        // Find optimal tour for coarse viewpoints of all frontiers
        void findGlobalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d& cur_yaw, vector<int> &indices);

        // Refine local tour for next few frontiers, using more diverse viewpoints
        void refineLocalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                             const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_yaws,
                             vector<Vector3d> &refined_pts, vector<double> &refined_yaws);

        void shortenPath(vector<Vector3d> &path);

    public:
        typedef shared_ptr<FastExplorationManager> Ptr;
    };

}  // namespace fast_planner

#endif