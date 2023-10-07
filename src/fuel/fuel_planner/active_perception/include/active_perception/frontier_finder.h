#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <utility>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::pair;

class RayCaster;

namespace fast_planner {
    class EDTEnvironment;

    class PerceptionUtils;

// Viewpoint to cover a frontier cluster
    struct Viewpoint {
        // Position and heading
        Vector3d pos_;
        double yaw_;
        int visible_num_;
    };

// A frontier cluster, the viewpoints to cover it
    struct Frontier {
        vector<Vector3d> cells_;            // Complete voxels belonging to the cluster
        vector<Vector3d> filtered_cells_;   // down-sampled voxels filtered by voxel grid filter
        Vector3d average_;                  // Average position of all voxels
        int id_;                            // Idx of cluster
        vector<Viewpoint> viewpoints_;      // Viewpoints that can cover the cluster
        Vector3d box_min_, box_max_;        // Bounding box of cluster, center & 1/2 side length
        vector<vector<Vector3d>> paths_;    // Path from this cluster to other clusters
        vector<double> costs_;              // cost from this cluster to other clusters
    };

    class FrontierFinder {
    public:
        FrontierFinder(const shared_ptr<EDTEnvironment> &edt, ros::NodeHandle &nh);

        ~FrontierFinder();

        void removeOutDatedFrontiers();

        void searchAndAddFrontiers();

        void getFrontiers(vector<vector<Vector3d>> &clusters);

        void getDormantFrontiers(vector<vector<Vector3d>> &clusters);

        void getFrontierBoxes(vector<pair<Vector3d, Vector3d>> &boxes);

        // Get viewpoint with highest coverage for each frontier
        void getTopViewpointsInfo(const Vector3d &cur_pos, vector<Vector3d> &points, vector<double> &yaws);

        // Get several viewpoints for a subset of frontiers
        void getNViewPoints(const Vector3d &cur_pos, const vector<int> &ids, const int &view_num,
                            const double &max_decay, vector<vector<Vector3d>> &n_points,
                            vector<vector<double>> &n_yaws);

        void updateFrontierCostMatrix();

        void getFullCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                               Eigen::MatrixXd &mat);

        bool isFrontierCovered();

        static void wrapYaw(double &yaw);

        shared_ptr<PerceptionUtils> perception_utils_;
        vector<Frontier> frontiers_, dormant_frontiers_;

    private:
        void splitLargeFrontiers(vector<Frontier> &frontiers);

        bool splitHorizontally(const Frontier &frontier, vector<Frontier> &splits);

        bool isFrontierChanged(const Frontier &ft);

        static bool haveOverlap(const Vector3d &min1, const Vector3d &max1, const Vector3d &min2, const Vector3d &max2);

        void computeFrontierInfo(Frontier &frontier);

        void downSample(const vector<Vector3d> &cluster_in, vector<Vector3d> &cluster_out);

        void sampleViewpoints(Frontier &frontier);

        int countVisibleCells(const Vector3d &pos, const double &yaw, const vector<Vector3d> &cluster);

        bool isNearUnknown(const Vector3d &pos);

        static vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i &voxel);

        static vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i &voxel);

        bool isNeighborUnknown(const Eigen::Vector3i &voxel);

        bool expandFrontier(const Eigen::Vector3i &first, Frontier &frontier);

        // Wrapper of sdf map
        int toAddress(const Eigen::Vector3i &idx);

        bool knownFree(const Eigen::Vector3i &idx);

        // Data
        vector<bool> is_in_frontier_;
        size_t origin_frontiers_num_{};

        // Params
        int cluster_min_{};
        double cluster_size_xy_{}, cluster_size_z_{};
        double candidate_rmax_{}, candidate_rmin_{}, candidate_dphi_{}, min_candidate_dist_{}, min_candidate_clearance_{};
        int down_sample_{};
        double min_view_finish_fraction_{}, resolution_;
        int min_visible_num_{}, candidate_rnum_{};

        // Utils
        shared_ptr<EDTEnvironment> edt_env_;
        unique_ptr<RayCaster> ray_caster_;
    };

}  // namespace fast_planner
#endif