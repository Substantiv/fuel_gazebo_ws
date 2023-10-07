#ifndef _PERCEPTION_UTILS_H_
#define _PERCEPTION_UTILS_H_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <iostream>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
    class PerceptionUtils {
    public:
        explicit PerceptionUtils(ros::NodeHandle &nh);

        ~PerceptionUtils() = default;

        // Set position and yaw
        void setPose(const Vector3d &pos, const double &yaw);

        // Get info of current pose
        void getFOV(vector<Vector3d> &list1, vector<Vector3d> &list2);

        bool insideFOV(const Vector3d &point);

        void getFOVBoundingBox(Vector3d &bmin, Vector3d &bmax);

    private:
        // Data
        Vector3d pos_;                  // Current camera pos
        double yaw_;                    // Current camera yaw
        vector<Vector3d> normals_;      // Camera plane's normals in world frame

        /* Params */
        double left_angle_, right_angle_, top_angle_, max_dist_, vis_dist_;     // Sensing range of camera
        Vector3d n_top_, n_bottom_, n_left_, n_right_;                          // Normal vectors of camera FOV planes in camera frame
        Eigen::Matrix4d T_cb_, T_bc_;                                           // Transform between camera and body
        vector<Vector3d> cam_vertices1_, cam_vertices2_;                        // FOV vertices in body frame
    };

}  // namespace fast_planner
#endif