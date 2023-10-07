#include <path_searching/kinodynamic_astar.h>
#include <memory>
#include <sstream>
#include <plan_env/sdf_map.h>
#include <unordered_set>
#include <../../plan_manage/include/plan_manage/backward.hpp>
#include <path_searching/matrix_hash.h>

namespace backward {
    backward::SignalHandling sh;
}

using namespace std;
using namespace Eigen;

namespace fast_planner {
    Vector6d stateTransit(const Vector6d &state0, const Eigen::Vector3d &um, const double tau) {
        Eigen::Matrix<double, 6, 6> phi = Eigen::MatrixXd::Identity(6, 6);  // state transit matrix
        for (Eigen::Index i = 0; i < 3; ++i)
            phi(i, i + 3) = tau;

        Vector6d integral;
        integral.head(3) = 0.5 * pow(tau, 2) * um;
        integral.tail(3) = tau * um;

        return phi * state0 + integral;
    }

    KinodynamicAstar::KinodynamicAstar(ros::NodeHandle &nh, const EDTEnvironment::Ptr &env) {
        nh.param("search/max_tau", max_tau_, -1.0);
        nh.param("search/init_max_tau", init_max_tau_, -1.0);
        nh.param("search/max_vel", max_vel_, -1.0);
        nh.param("search/max_acc", max_acc_, -1.0);
        nh.param("search/w_time", w_time_, -1.0);
        nh.param("search/horizon", horizon_, -1.0);
        nh.param("search/resolution_astar", resolution_, -1.0);
        nh.param("search/time_resolution", time_resolution_, -1.0);
        nh.param("search/lambda_heu", lambda_heu_, -1.0);
        nh.param("search/allocate_num", allocate_num_, -1);
        nh.param("search/check_num", check_num_, -1);
        nh.param("search/optimistic", optimistic_, true);

        double vel_margin;
        nh.param("search/vel_margin", vel_margin, 0.0);
        max_vel_ += vel_margin;

        /* ---------- map params ---------- */
        this->edt_environment_ = env;
        edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

        cout << "kino origin_: " << origin_.transpose() << endl;
        cout << "kino map size: " << map_size_3d_.transpose() << endl;
    }

    class NodeComparator {
    public:
        bool operator()(const PathNodePtr &node1, const PathNodePtr &node2) {
            return node1->f_score > node2->f_score;
        }
    };

    int KinodynamicAstar::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_v, const Eigen::Vector3d &start_a,
                                 const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_v,
                                 const bool init_search, vector<PathNodePtr> &path,
                                 bool &is_shot_succ, Matrix34 &coef_shot, double &shot_time) {
        PathNodePtr cur_node = make_shared<PathNode>();
        cur_node->parent = nullptr;
        cur_node->state.head(3) = start_pt;
        cur_node->state.tail(3) = start_v;
        cur_node->discretized_state = discretizeState(cur_node->state);
        cur_node->g_score = 0.0;

        Eigen::VectorXd end_state(6);
        end_state.head(3) = end_pt;
        end_state.tail(3) = end_v;
        Eigen::Vector3i end_index = discretizeState(end_state).head(3);
        double time_to_goal;
        cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
        std::unordered_set<Vector6i, matrix_hash<Vector6i>> visited_ids;
        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> que;
        que.push(cur_node);

        int use_node_num = 1, iter_num = 0;

        double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
        vector<Eigen::Vector3d> inputs;
        vector<double> durations;
        if (init_search) {
            inputs.push_back(start_a);
            for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3; tau += time_res_init * init_max_tau_)
                durations.push_back(tau);
        } else {
            for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
                for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
                    for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res) {
                        inputs.emplace_back(ax, ay, az);
                    }
            for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
                durations.push_back(tau);
        }

        while (!que.empty()) {
            cur_node = que.top();
            que.pop();
            Eigen::Vector3d cur_pos = cur_node->state.head(3);
            Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
            if (visited_ids.count(cur_node->discretized_state)) {
                continue;
            } else {
                visited_ids.insert(cur_node->discretized_state);
            }
            iter_num += 1;

            // Terminate?
            bool reach_horizon = (cur_pos - start_pt).norm() >= horizon_;
            const int tolerance = ceil(1 / resolution_);
            bool near_end = abs(cur_node->discretized_state(0) - end_index(0)) <= tolerance &&
                            abs(cur_node->discretized_state(1) - end_index(1)) <= tolerance &&
                            abs(cur_node->discretized_state(2) - end_index(2)) <= tolerance;
            if (reach_horizon || near_end) {
                path = move(retrievePath(cur_node));
            }
            if (near_end) {
                // Check whether shot traj exist
                estimateHeuristic(cur_node->state, end_state, shot_time);
                is_shot_succ = computeShotTraj(cur_node->state, end_state, shot_time, coef_shot);
                if (is_shot_succ) {
                    std::cout << "reach end" << std::endl;
                    return REACH_END;
                } else if (cur_node->parent) {
                    std::cout << "near end" << std::endl;
                    return NEAR_END;
                } else {
                    std::cout << "no path" << std::endl;
                    return NO_PATH;
                }
            }
            if (reach_horizon) {
                std::cout << "reach horizon" << std::endl;
                return REACH_HORIZON;
            }

            for (const Vector3d &input: inputs) {
                for (double tau: durations) {
                    Vector6d pro_state = stateTransit(cur_state, input, tau);

                    // Check inside map range
                    Eigen::Vector3d pro_pos = pro_state.head(3);
                    if (!edt_environment_->sdf_map_->isInBox(pro_pos)) { continue; }

                    // Check if visited
                    Vector6i pro_discretized_state = discretizeState(pro_state);
                    if (visited_ids.count(pro_discretized_state)) { continue; }

                    // Check maximal velocity
                    Eigen::Vector3d pro_v = pro_state.tail(3);
                    if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_) {
                        continue;
                    }

                    // Check not in the same voxel
                    Vector6i diff = pro_discretized_state - cur_node->discretized_state;
                    if (diff.norm() == 0) { continue; }

                    // Check safety
                    bool is_occ = false;
                    for (int k = 1; k <= check_num_; ++k) {
                        double dt = tau * double(k) / double(check_num_);
                        Vector6d xt = stateTransit(cur_state, input, dt);
                        Eigen::Vector3d pos = xt.head(3);
                        if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 ||
                            !edt_environment_->sdf_map_->isInBox(pos)) {
                            is_occ = true;
                            break;
                        }
                        if (!optimistic_ && edt_environment_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN) {
                            is_occ = true;
                            break;
                        }
                    }
                    if (is_occ) { continue; }

                    double tmp_time_to_goal;
                    double tmp_g_score = (input.squaredNorm() + w_time_) * tau + cur_node->g_score;
                    double tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, tmp_time_to_goal);
                    auto pro_node = std::make_shared<PathNode>(pro_discretized_state, pro_state, tmp_g_score, tmp_f_score, input, tau, cur_node);
                    que.push(pro_node);

                    use_node_num++;
                    if (use_node_num == allocate_num_) {
                        cout << "run out of memory." << endl;
                        return NO_PATH;
                    }
                }
            }
        }

        cout << "open set empty, no path!" << endl;
        cout << "use node num: " << use_node_num << endl;
        cout << "iter num: " << iter_num << endl;
        return NO_PATH;
    }

    vector<PathNodePtr> KinodynamicAstar::retrievePath(PathNodePtr cur_node) {
        vector<PathNodePtr> path;
        while (cur_node) {
            path.push_back(cur_node);
            cur_node = cur_node->parent;
        }
        reverse(path.begin(), path.end());
        for (int i = 0; i < (int) path.size() - 1; ++i) { // 这一步是必要的，不然在采样函数getSamples()中会出错。
            path[i]->duration = path[i + 1]->duration;
        }
        return path;
    }

    double KinodynamicAstar::estimateHeuristic(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2,
                                               double &optimal_time) const {
        const Vector3d dp = x2.head(3) - x1.head(3);
        const Vector3d v0 = x1.segment(3, 3);
        const Vector3d v1 = x2.segment(3, 3);

        double c1 = -36 * dp.dot(dp);
        double c2 = 24 * (v0 + v1).dot(dp);
        double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
        double c4 = 0;
        double c5 = w_time_;

        std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

        double v_max = max_vel_ * 0.5;
        double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
        ts.push_back(t_bar);

        double cost = 100000000;
        optimal_time = t_bar;

        for (const double t: ts) {
            if (t < t_bar) continue;
            double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
            if (c < cost) {
                cost = c;
                optimal_time = t;
            }
        }

        constexpr double tie_breaker = 1.0 / 10000;
        return (1 + tie_breaker) * cost;
    }

    bool KinodynamicAstar::computeShotTraj(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2,
                                           const double time_to_goal,
                                           Matrix34 &coef_shot) {
        /* ---------- get coefficient ---------- */
        const Vector3d p0 = state1.head(3);
        const Vector3d dp = state2.head(3) - p0;
        const Vector3d v0 = state1.segment(3, 3);
        const Vector3d v1 = state2.segment(3, 3);
        const Vector3d dv = v1 - v0;
        coef_shot = Matrix34::Zero();

        Vector3d a = 1.0 / 6.0 * (-12.0 / (time_to_goal * time_to_goal * time_to_goal) * (dp - v0 * time_to_goal) +
                                  6 / (time_to_goal * time_to_goal) * dv);
        Vector3d b = 0.5 * (6.0 / (time_to_goal * time_to_goal) * (dp - v0 * time_to_goal) - 2 / time_to_goal * dv);
        const Vector3d &c = v0;
        const Vector3d &d = p0;

        // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
        // a*t^3 + b*t^2 + v0*t + p0
        coef_shot.col(3) = a, coef_shot.col(2) = b, coef_shot.col(1) = c, coef_shot.col(0) = d;

        Vector3d coord, vel, acc;
        VectorXd poly1d, t, polyv, polya;
        Vector3i index;

        Eigen::MatrixXd Tm(4, 4);
        Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

        /* ---------- forward checking of trajectory ---------- */
        double t_delta = time_to_goal / 10;
        for (double time = t_delta; time <= time_to_goal; time += t_delta) {
            t = VectorXd::Zero(4);
            for (Eigen::Index j = 0; j < 4; j++)
                t(j) = pow(time, j);

            for (int dim = 0; dim < 3; dim++) {
                poly1d = coef_shot.row(dim);
                coord(dim) = poly1d.dot(t);
                vel(dim) = (Tm * poly1d).dot(t);
                acc(dim) = (Tm * Tm * poly1d).dot(t);

                if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_) {
                    cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
                    return false;
                }
            }

            if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) ||
                coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
                coord(2) < origin_(2) || coord(2) >= map_size_3d_(2)) {
                return false;
            }

            if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == SDFMap::OCCUPIED) {
                return false;
            }
        }
        return true;
    }

    vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d) {
        vector<double> dts;

        double a2 = b / a;
        double a1 = c / a;
        double a0 = d / a;

        double Q = (3 * a1 - a2 * a2) / 9;
        double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
        double D = Q * Q * Q + R * R;
        if (D > 0) {
            double S = std::cbrt(R + sqrt(D));
            double T = std::cbrt(R - sqrt(D));
            dts.push_back(-a2 / 3 + (S + T));
            return dts;
        } else if (D == 0) {
            double S = std::cbrt(R);
            dts.push_back(-a2 / 3 + S + S);
            dts.push_back(-a2 / 3 - S);
            return dts;
        } else {
            double theta = acos(R / sqrt(-Q * Q * Q));
            dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
            dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
            dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
            return dts;
        }
    }

    vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e) {
        vector<double> dts;

        double a3 = b / a;
        double a2 = c / a;
        double a1 = d / a;
        double a0 = e / a;

        vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
        double y1 = ys.front();
        double r = a3 * a3 / 4 - a2 + y1;
        if (r < 0) return dts;

        double R = sqrt(r);
        double D, E;
        if (R != 0) {
            D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
            E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        } else {
            D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
            E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
        }

        if (!std::isnan(D)) {
            dts.push_back(-a3 / 4 + R / 2 + D / 2);
            dts.push_back(-a3 / 4 + R / 2 - D / 2);
        }
        if (!std::isnan(E)) {
            dts.push_back(-a3 / 4 - R / 2 + E / 2);
            dts.push_back(-a3 / 4 - R / 2 - E / 2);
        }

        return dts;
    }

    void KinodynamicAstar::getSamples(const vector<PathNodePtr> &path,
                                      const Eigen::Vector3d &start_v, const Eigen::Vector3d &end_v,
                                      const bool is_shot_succ, const Matrix34 &coef_shot, const double t_shot,
                                      double &ts, vector<Eigen::Vector3d> &point_set,
                                      vector<Eigen::Vector3d> &start_end_derivatives) {
        /* ---------- path duration ---------- */
        double sum_t = 0;
        for (const PathNodePtr &node: path) {
            sum_t += node->duration;
        }
        if (is_shot_succ) {
            sum_t += t_shot;
        }

        int seg_num = floor(sum_t / ts);
        seg_num = max(8, seg_num);
        ts = sum_t / seg_num;
        double t_from_pre_node = 0;

        for (const PathNodePtr &node: path) {
            while (t_from_pre_node < node->duration) {
                Vector6d xt = stateTransit(node->state, node->input, t_from_pre_node);
                point_set.emplace_back(xt.head(3));
                t_from_pre_node += ts;
            }
            t_from_pre_node -= node->duration;
        }

        if (is_shot_succ) {
            while (t_from_pre_node < t_shot) {
                Vector3d coord;
                Vector4d poly1d, time;

                for (Eigen::Index j = 0; j < 4; j++)
                    time(j) = pow(t_from_pre_node, j);

                for (int dim = 0; dim < 3; dim++) {
                    poly1d = coef_shot.row(dim);
                    coord(dim) = poly1d.dot(time);
                }

                point_set.push_back(coord);
                t_from_pre_node += ts;
            }
        }

        // Calculate boundary vel and acc
        Eigen::Vector3d end_vel, end_acc;
        if (is_shot_succ) {
            end_vel = end_v;
            for (int dim = 0; dim < 3; ++dim) {
                Vector4d coe = coef_shot.row(dim);
                end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot;
            }
        } else {
            end_vel = path.back()->state.tail(3);
            end_acc = path.back()->input;
        }

        // calculate start acc
        Eigen::Vector3d start_acc;
        if (path.empty()) { // no searched trajectory, calculate by shot trajectory
            start_acc = 2 * coef_shot.col(2);
        } else { // input of searched trajectory
            start_acc = path.front()->input;
        }

        start_end_derivatives.push_back(start_v);
        start_end_derivatives.push_back(end_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(end_acc);
    }

    Vector6i KinodynamicAstar::discretizeState(const Vector6d &state) {
        Vector3i idx = ((state.head(3) - origin_) / resolution_).array().floor().cast<int>();
        Vector6i discretized_state;
        discretized_state.head(3) = idx;
        discretized_state.tail(3) = (state.tail(3) / resolution_).array().floor().cast<int>();
        return discretized_state;
    }


}  // namespace fast_planner
