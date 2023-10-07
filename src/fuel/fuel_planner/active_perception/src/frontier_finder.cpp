#include <active_perception/frontier_finder.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigenvalues>
#include <memory>

namespace fast_planner {
    FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr &edt, ros::NodeHandle &nh) {
        this->edt_env_ = edt;
        int voxel_num = edt->sdf_map_->getVoxelNum();
        is_in_frontier_ = vector<bool>(voxel_num, false);

        nh.param("frontier/cluster_min", cluster_min_, -1);
        nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
        nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
        nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
        nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
        nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
        nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
        nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
        nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
        nh.param("frontier/down_sample", down_sample_, -1);
        nh.param("frontier/min_visib_num", min_visible_num_, -1);
        nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);

        ray_caster_ = std::make_unique<RayCaster>();
        resolution_ = edt_env_->sdf_map_->getResolution();
        Eigen::Vector3d origin, size;
        edt_env_->sdf_map_->getRegion(origin, size);
        ray_caster_->setParams(resolution_, origin);

        perception_utils_.reset(new PerceptionUtils(nh));
    }

    FrontierFinder::~FrontierFinder() = default;

    void FrontierFinder::removeOutDatedFrontiers() {
        Vector3d update_min, update_max;
        edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

        /****************************
         * remove changed frontiers *
         ****************************/
        vector<bool> is_frontier_changed;
        for (const Frontier &frontier: frontiers_) {
            if (haveOverlap(frontier.box_min_, frontier.box_max_, update_min, update_max) &&
                isFrontierChanged(frontier)) {
                is_frontier_changed.push_back(true);
            } else {
                is_frontier_changed.push_back(false);
            }
        }
        vector<Frontier> not_changed_frontiers;
        for (size_t i = 0; i < frontiers_.size(); ++i) {
            if (is_frontier_changed[i]) {
                Vector3i idx;
                for (const Vector3d &cell: frontiers_[i].cells_) {
                    edt_env_->sdf_map_->posToIndex(cell, idx);
                    is_in_frontier_[toAddress(idx)] = false;
                }
            } else {
                not_changed_frontiers.emplace_back(move(frontiers_[i]));
            }
        }
        frontiers_ = move(not_changed_frontiers);

        /********************************************
         * remove changed frontiers costs and paths *
         ********************************************/
        for (Frontier &frontier: frontiers_) {
            vector<double> costs;
            vector<vector<Vector3d>> paths;
            for (size_t i = 0; i < frontier.costs_.size(); ++i) {
                if (!is_frontier_changed[i]) {
                    costs.push_back(frontier.costs_[i]);
                    paths.emplace_back(move(frontier.paths_[i]));
                }
            }
            frontier.costs_ = move(costs);
            frontier.paths_ = move(paths);
        }

        /************************************
         * remove changed dormant frontiers *
         ************************************/
        vector<bool> is_dormant_frontiers_changed;
        for (const Frontier &frontier: dormant_frontiers_) {
            if (haveOverlap(frontier.box_min_, frontier.box_max_, update_min, update_max) &&
                isFrontierChanged(frontier)) {
                is_dormant_frontiers_changed.push_back(true);
            } else {
                is_dormant_frontiers_changed.push_back(false);
            }
        }
        vector<Frontier> not_changed_dormant_frontiers;
        for (int i = 0; i < dormant_frontiers_.size(); ++i) {
            if (is_dormant_frontiers_changed[i]) {
                Vector3i idx;
                for (const Vector3d &cell: dormant_frontiers_[i].cells_) {
                    edt_env_->sdf_map_->posToIndex(cell, idx);
                    is_in_frontier_[toAddress(idx)] = false;
                }
            } else {
                not_changed_dormant_frontiers.emplace_back(move(dormant_frontiers_[i]));
            }
        }
        dormant_frontiers_ = move(not_changed_dormant_frontiers);
    }

    void FrontierFinder::searchAndAddFrontiers() {
        ros::Time t1 = ros::Time::now();

        // Bounding box of updated region
        Vector3d update_min, update_max;
        edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

        // Search new frontier within box slightly inflated from updated box
        Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
        Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
        Vector3d box_min, box_max;
        edt_env_->sdf_map_->getBox(box_min, box_max);
        for (int k = 0; k < 3; ++k) {
            search_min[k] = max(search_min[k], box_min[k]);
            search_max[k] = min(search_max[k], box_max[k]);
        }
        Eigen::Vector3i min_id, max_id;
        edt_env_->sdf_map_->posToIndex(search_min, min_id);
        edt_env_->sdf_map_->posToIndex(search_max, max_id);

        vector<Frontier> tmp_frontiers;
        for (int x = min_id(0); x <= max_id(0); ++x)
            for (int y = min_id(1); y <= max_id(1); ++y)
                for (int z = min_id(2); z <= max_id(2); ++z) {
                    // Scanning the updated region to find seeds of frontiers
                    Eigen::Vector3i cur(x, y, z);
                    if (!is_in_frontier_[toAddress(cur)] && knownFree(cur) && isNeighborUnknown(cur)) {
                        // Expand from the seed cell to find a complete frontier cluster
                        Frontier frontier;
                        if (expandFrontier(cur, frontier)) {
                            tmp_frontiers.push_back(move(frontier));
                        }
                    }
                }
        splitLargeFrontiers(tmp_frontiers);

        ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());

        origin_frontiers_num_ = frontiers_.size();
        int new_num = 0;
        int new_dormant_num = 0;
        // Try to find viewpoints for each cluster and categorize them according to viewpoint number
        for (Frontier &tmp_ftr: tmp_frontiers) {
            // Search viewpoints around frontier
            sampleViewpoints(tmp_ftr);
            if (!tmp_ftr.viewpoints_.empty()) {
                ++new_num;
                sort(tmp_ftr.viewpoints_.begin(), tmp_ftr.viewpoints_.end(),
                     [](const Viewpoint &v1, const Viewpoint &v2) -> bool {
                         return v1.visible_num_ > v2.visible_num_;
                     });
                frontiers_.insert(frontiers_.end(), tmp_ftr);
            } else {
                // Find no viewpoint, move cluster to dormant list
                dormant_frontiers_.push_back(tmp_ftr);
                ++new_dormant_num;
            }
        }
        // Reset indices of frontiers
        int idx = 0;
        for (Frontier &ft: frontiers_) {
            ft.id_ = idx++;
        }
    }

    bool FrontierFinder::expandFrontier(const Eigen::Vector3i &first, Frontier &frontier) {
        // Data for clustering
        queue<Eigen::Vector3i> cell_queue;
        vector<Eigen::Vector3d> expanded;
        Vector3d pos;

        edt_env_->sdf_map_->indexToPos(first, pos);
        expanded.push_back(pos);
        cell_queue.push(first);
        is_in_frontier_[toAddress(first)] = true;

        // Search frontier cluster based on region growing (distance clustering)
        while (!cell_queue.empty()) {
            Vector3i cur_cell = cell_queue.front();
            cell_queue.pop();
            vector<Vector3i> neighbors = allNeighbors(cur_cell);
            for (const Vector3i &next_cell: neighbors) {
                // Qualified cell should be inside bounding box and frontier cell not clustered
                int next_cell_adr = toAddress(next_cell);
                if (is_in_frontier_[next_cell_adr] || !edt_env_->sdf_map_->isInBox(next_cell) ||
                    !(knownFree(next_cell) && isNeighborUnknown(next_cell)))
                    continue;

                edt_env_->sdf_map_->indexToPos(next_cell, pos);
                if (pos[2] < 0.4) continue;  // Remove noise close to ground
                expanded.push_back(pos);
                cell_queue.push(next_cell);
                is_in_frontier_[next_cell_adr] = true;
            }
        }
        if (expanded.size() > cluster_min_) {
            frontier.cells_ = expanded;
            computeFrontierInfo(frontier);
            return true;
        }
        return false;
    }

    void FrontierFinder::splitLargeFrontiers(vector<Frontier> &frontiers) {
        vector<Frontier> split_frontiers;
        for (Frontier &frontier: frontiers) {
            vector<Frontier> splits;
            if (splitHorizontally(frontier, splits)) {
                split_frontiers.insert(split_frontiers.end(), splits.begin(), splits.end());
            } else
                split_frontiers.emplace_back(move(frontier));
        }
        frontiers = move(split_frontiers);
    }

    bool FrontierFinder::splitHorizontally(const Frontier &frontier, vector<Frontier> &splits) {
        // Split a frontier into small piece if it is too large
        auto mean = frontier.average_.head<2>();
        bool need_split = false;
        for (Vector3d cell: frontier.filtered_cells_) {
            if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
                need_split = true;
                break;
            }
        }
        if (!need_split) return false;

        // Compute principal component of Covariance matrix of cells
        Eigen::Matrix2d cov;
        cov.setZero();
        for (Vector3d cell: frontier.filtered_cells_) {
            Eigen::Vector2d diff = cell.head<2>() - mean;
            cov += diff * diff.transpose();
        }
        cov /= double(frontier.filtered_cells_.size());

        // Find eigenvector corresponds to maximal eigenvector
        Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
        auto values = es.eigenvalues().real();
        auto vectors = es.eigenvectors().real();
        Eigen::Index max_idx;
        double max_eigenvalue = -1000000;
        for (Eigen::Index i = 0; i < values.rows(); ++i) {
            if (values[i] > max_eigenvalue) {
                max_idx = i;
                max_eigenvalue = values[i];
            }
        }
        Eigen::Vector2d dominate_eigen_vector = vectors.col(max_idx);

        Frontier ftr1, ftr2;
        for (Vector3d cell: frontier.cells_) {
            if ((cell.head<2>() - mean).dot(dominate_eigen_vector) >= 0)
                ftr1.cells_.push_back(cell);
            else
                ftr2.cells_.push_back(cell);
        }
        computeFrontierInfo(ftr1);
        computeFrontierInfo(ftr2);

        // Recursive call to split frontier that is still too large
        vector<Frontier> splits1;
        if (splitHorizontally(ftr1, splits1)) {
            splits.insert(splits.end(), splits1.begin(), splits1.end());
        } else
            splits.push_back(ftr1);

        vector<Frontier> splits2;
        if (splitHorizontally(ftr2, splits2))
            splits.insert(splits.end(), splits2.begin(), splits2.end());
        else
            splits.push_back(ftr2);

        return true;
    }

    void FrontierFinder::updateFrontierCostMatrix() {
        auto updateCost = [](Frontier &it1, Frontier &it2) -> void {
            // Search path from old cluster's top viewpoint to new cluster'
            Viewpoint &vui = it1.viewpoints_.front();
            Viewpoint &vuj = it2.viewpoints_.front();
            vector<Vector3d> path_ij;
            double cost_ij = ViewNode::computeCost(
                    vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
            // Insert item for both old and new clusters
            it1.costs_.push_back(cost_ij);
            it1.paths_.push_back(path_ij);
            reverse(path_ij.begin(), path_ij.end());
            it2.costs_.push_back(cost_ij);
            it2.paths_.push_back(path_ij);
        };

        // Compute path and cost between old and new clusters
        for (size_t i = 0; i < origin_frontiers_num_; ++i) {
            for (size_t j = origin_frontiers_num_; j < frontiers_.size(); ++j) {
                updateCost(frontiers_[i], frontiers_[j]);
            }
        }
        for (size_t i = origin_frontiers_num_; i < frontiers_.size(); ++i) {
            frontiers_[i].costs_.push_back(0);
            frontiers_[i].paths_.emplace_back();
            for (size_t j = origin_frontiers_num_ + 1; j < frontiers_.size(); ++j) {
                updateCost(frontiers_[i], frontiers_[j]);
            }
        }
    }

    bool FrontierFinder::haveOverlap(const Vector3d &min1, const Vector3d &max1, const Vector3d &min2,
                                     const Vector3d &max2) {
        // Check if two box have overlap part
        Vector3d box_min, box_max;
        for (Eigen::Index i = 0; i < 3; ++i) {
            box_min[i] = max(min1[i], min2[i]);
            box_max[i] = min(max1[i], max2[i]);
            if (box_min[i] > box_max[i] + 1e-3) {
                return false;
            }
        }
        return true;
    }

    bool FrontierFinder::isFrontierChanged(const Frontier &ft) {
        for (const Vector3d &cell: ft.cells_) {
            Eigen::Vector3i idx;
            edt_env_->sdf_map_->posToIndex(cell, idx);
            if (!(knownFree(idx) && isNeighborUnknown(idx))) return true;
        }
        return false;
    }

    void FrontierFinder::computeFrontierInfo(Frontier &ftr) {
        // Compute average position and bounding box of cluster
        ftr.average_.setZero();
        ftr.box_max_ = ftr.cells_.front();
        ftr.box_min_ = ftr.cells_.front();
        for (const Vector3d &cell: ftr.cells_) {
            ftr.average_ += cell;
            for (Eigen::Index i = 0; i < 3; ++i) {
                ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
                ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
            }
        }
        ftr.average_ /= double(ftr.cells_.size());

        // Compute down-sampled cluster
        downSample(ftr.cells_, ftr.filtered_cells_);
    }

    void FrontierFinder::getTopViewpointsInfo(const Vector3d &cur_pos, vector<Eigen::Vector3d> &points,
                                              vector<double> &yaws) {
        points.clear();
        yaws.clear();
        for (const Frontier &frontier: frontiers_) {
            bool no_view = true;
            for (const Viewpoint &view: frontier.viewpoints_) {
                // Retrieve the first viewpoint that is far enough and has the highest coverage
                if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
                points.push_back(view.pos_);
                yaws.push_back(view.yaw_);
                no_view = false;
                break;
            }
            if (no_view) {
                // All viewpoints are very close, just use the first one (with the highest coverage).
                const Viewpoint &view = frontier.viewpoints_.front();
                points.push_back(view.pos_);
                yaws.push_back(view.yaw_);
            }
        }
    }

    void FrontierFinder::getNViewPoints(
            const Vector3d &cur_pos, const vector<int> &ids, const int &view_num, const double &max_decay,
            vector<vector<Eigen::Vector3d>> &n_points, vector<vector<double>> &n_yaws) {
        n_points.clear();
        n_yaws.clear();
        for (const int id: ids) {
            const Frontier &frontier = frontiers_[id];
            vector<Eigen::Vector3d> pts;
            vector<double> ys;
            int visible_thresh = frontier.viewpoints_.front().visible_num_ * max_decay;
            for (const Viewpoint &view: frontier.viewpoints_) {
                if (pts.size() >= view_num || view.visible_num_ <= visible_thresh) break;
                if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
                pts.push_back(view.pos_);
                ys.push_back(view.yaw_);
            }
            if (pts.empty()) {
                // All viewpoints are very close, ignore the distance limit
                for (const Viewpoint &view: frontier.viewpoints_) {
                    if (pts.size() >= view_num || view.visible_num_ <= visible_thresh) break;
                    pts.push_back(view.pos_);
                    ys.push_back(view.yaw_);
                }
            }
            n_points.push_back(pts);
            n_yaws.push_back(ys);
        }
    }

    void FrontierFinder::getFullCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                                           Eigen::MatrixXd &mat) {
        // Use Asymmetric TSP
        Eigen::Index dimension = frontiers_.size();
        mat.resize(dimension + 1, dimension + 1);
        // Fill block for clusters
        for (Eigen::Index i = 0; i < frontiers_.size(); ++i) {
            for (Eigen::Index j = 0; j < frontiers_.size(); ++j) {
                mat(i + 1, j + 1) = frontiers_[i].costs_[j];
            }
        }

        // Fill block from current state to clusters
        mat.leftCols<1>().setZero();
        for (int j = 0; j < frontiers_.size(); ++j) {
            Viewpoint vj = frontiers_[j].viewpoints_.front();
            vector<Vector3d> path;
            mat(0, j + 1) =
                    ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
        }
    }

// Sample viewpoints around frontier's average position, check coverage to the frontier cells
    void FrontierFinder::sampleViewpoints(Frontier &frontier) {
        // Evaluate sample viewpoints on circles, find ones that cover most cells
        for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
             rc <= candidate_rmax_ + 1e-3; rc += dr)
            for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
                const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);

                // Qualified viewpoint is in bounding box and in safe region
                if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
                    edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 ||
                    isNearUnknown(sample_pos)) {
                    continue;
                }

                // Compute average yaw
                const vector<Vector3d> &cells = frontier.filtered_cells_;
                Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
                double avg_yaw = 0.0;
                for (size_t i = 1; i < cells.size(); ++i) {
                    Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
                    double yaw = acos(dir.dot(ref_dir));
                    if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
                    avg_yaw += yaw;
                }
                avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
                wrapYaw(avg_yaw);
                // Compute the fraction of covered and visible cells
                int visible_num = countVisibleCells(sample_pos, avg_yaw, cells);
                if (visible_num > min_visible_num_) {
                    Viewpoint vp = {sample_pos, avg_yaw, visible_num};
                    frontier.viewpoints_.push_back(vp);
                }
            }
    }

    bool FrontierFinder::isFrontierCovered() {
        Vector3d update_min, update_max;
        edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

        auto checkChanges = [&](const vector<Frontier> &frontiers) -> bool {
            for (const Frontier &ftr: frontiers) {
                if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)) continue;
                const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
                int change_num = 0;
                for (const Vector3d &cell: ftr.cells_) {
                    Eigen::Vector3i idx;
                    edt_env_->sdf_map_->posToIndex(cell, idx);
                    if (!(knownFree(idx) && isNeighborUnknown(idx))) { change_num++; }
                    if (change_num >= change_thresh) { return true; }
                }
            }
            return false;
        };

        return (checkChanges(frontiers_) || checkChanges(dormant_frontiers_));
    }

    bool FrontierFinder::isNearUnknown(const Eigen::Vector3d &pos) {
        const int vox_num = floor(min_candidate_clearance_ / resolution_);
        for (int x = -vox_num; x <= vox_num; ++x)
            for (int y = -vox_num; y <= vox_num; ++y)
                for (int z = -1; z <= 1; ++z) {
                    Eigen::Vector3d vox;
                    vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
                    if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN) return true;
                }
        return false;
    }

    int FrontierFinder::countVisibleCells(const Eigen::Vector3d &pos, const double &yaw,
                                          const vector<Eigen::Vector3d> &cluster) {
        perception_utils_->setPose(pos, yaw);
        int visible_num = 0;
        Eigen::Vector3i idx;
        for (const Vector3d &cell: cluster) {
            if (!perception_utils_->insideFOV(cell)) continue;

            // Check if frontier cell is visible (not occluded by obstacles)
            ray_caster_->input(cell, pos);
            bool is_visible = true;
            while (ray_caster_->nextId(idx)) {
                if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 ||
                    edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
                    is_visible = false;
                    break;
                }
            }
            if (is_visible) visible_num += 1;
        }
        return visible_num;
    }

    void FrontierFinder::downSample(
            const vector<Eigen::Vector3d> &cluster_in, vector<Eigen::Vector3d> &cluster_out) {
        // down sampling cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const Vector3d &cell: cluster_in)
            cloud->points.emplace_back(cell[0], cell[1], cell[2]);

        const float leaf_size = (float) edt_env_->sdf_map_->getResolution() * (float) down_sample_;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        sor.filter(*filtered_cloud);

        cluster_out.clear();
        for (const pcl::PointXYZ &pt: filtered_cloud->points)
            cluster_out.emplace_back(pt.x, pt.y, pt.z);
    }

    void FrontierFinder::wrapYaw(double &yaw) {
        while (yaw < -M_PI)
            yaw += 2 * M_PI;
        while (yaw > M_PI)
            yaw -= 2 * M_PI;
    }

    inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i &voxel) {
        vector<Eigen::Vector3i> neighbors(6);
        Eigen::Vector3i tmp;

        tmp = voxel - Eigen::Vector3i(1, 0, 0);
        neighbors[0] = tmp;
        tmp = voxel + Eigen::Vector3i(1, 0, 0);
        neighbors[1] = tmp;
        tmp = voxel - Eigen::Vector3i(0, 1, 0);
        neighbors[2] = tmp;
        tmp = voxel + Eigen::Vector3i(0, 1, 0);
        neighbors[3] = tmp;
        tmp = voxel - Eigen::Vector3i(0, 0, 1);
        neighbors[4] = tmp;
        tmp = voxel + Eigen::Vector3i(0, 0, 1);
        neighbors[5] = tmp;

        return neighbors;
    }

    inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i &voxel) {
        vector<Eigen::Vector3i> neighbors(26);
        Eigen::Vector3i tmp;
        int count = 0;
        for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y)
                for (int z = -1; z <= 1; ++z) {
                    if (x == 0 && y == 0 && z == 0) continue;
                    tmp = voxel + Eigen::Vector3i(x, y, z);
                    neighbors[count++] = tmp;
                }
        return neighbors;
    }

    inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i &voxel) {
        // At least one neighbor is unknown
        vector<Eigen::Vector3i> neighbors = move(sixNeighbors(voxel));
        return std::any_of(neighbors.begin(), neighbors.end(), [&](const Vector3i &nbr) -> bool {
            return edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN;
        });
    }

    inline int FrontierFinder::toAddress(const Eigen::Vector3i &idx) {
        return edt_env_->sdf_map_->toAddress(idx);
    }

    inline bool FrontierFinder::knownFree(const Eigen::Vector3i &idx) {
        return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
    }

}  // namespace fast_planner