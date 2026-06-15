// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// #include <algorithm>
// #include <cmath>
// #include <limits>
// #include <numeric>
// #include <random>
// #include <tuple>
// #include <vector>

// #include <Eigen/Eigenvalues>
// #include <sophus/se3.hpp>
// #include <sophus/so3.hpp>

// #include <tbb/blocked_range.h>
// #include <tbb/concurrent_vector.h>
// #include <tbb/global_control.h>
// #include <tbb/parallel_for.h>
// #include <tbb/parallel_reduce.h>
// #include <tbb/task_arena.h>

// #include "Registration.hpp"
// #include "VoxelHashMap.hpp"
// #include "VoxelUtils.hpp"

// namespace Eigen {
// using Matrix6d   = Eigen::Matrix<double, 6, 6>;
// using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
// using Vector6d   = Eigen::Matrix<double, 6, 1>;
// }  // namespace Eigen

// using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
// using LinearSystem    = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

// namespace {

// inline double square(double x) { return x * x; }

// void TransformPoints(const Sophus::SE3d& T, std::vector<Eigen::Vector3d>& points) {
//     std::transform(points.cbegin(), points.cend(), points.begin(),
//                    [&](const auto& point) { return T * point; });
// }

// Correspondences DataAssociation(const std::vector<Eigen::Vector3d>& points,
//                                 const kiss_icp::VoxelHashMap& voxel_map,
//                                 const double max_correspondance_distance) {
//     using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
//     Correspondences correspondences;
//     correspondences.reserve(points.size());

//     tbb::parallel_for(
//         tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
//         [&](const tbb::blocked_range<points_iterator>& r) {
//             std::for_each(r.begin(), r.end(), [&](const auto& point) {
//                 const auto& [closest_neighbor, distance] = voxel_map.GetClosestNeighbor(point);
//                 if (distance < max_correspondance_distance) {
//                     correspondences.emplace_back(point, closest_neighbor);
//                 }
//             });
//         });

//     return correspondences;
// }

// LinearSystem BuildLinearSystem(const Correspondences& correspondences,
//                                const double kernel_scale) {
//     auto compute_jacobian_and_residual = [](const auto& correspondence) {
//         const auto& [source, target] = correspondence;
//         const Eigen::Vector3d residual = source - target;

//         Eigen::Matrix3_6d J_r;
//         J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
//         J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source);

//         return std::make_tuple(J_r, residual);
//     };

//     auto sum_linear_systems = [](LinearSystem a, const LinearSystem& b) {
//         a.first += b.first;
//         a.second += b.second;
//         return a;
//     };

//     auto GM_weight = [&](const double& residual2) {
//         return square(kernel_scale) / square(kernel_scale + residual2);
//     };

//     using correspondence_iterator = Correspondences::const_iterator;
//     const auto& [JTJ, JTr] = tbb::parallel_reduce(
//         tbb::blocked_range<correspondence_iterator>{correspondences.cbegin(),
//                                                     correspondences.cend()},
//         LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero()),
//         [&](const tbb::blocked_range<correspondence_iterator>& r, LinearSystem J) -> LinearSystem {
//             return std::transform_reduce(
//                 r.begin(), r.end(), J, sum_linear_systems,
//                 [&](const auto& correspondence) {
//                     const auto& [J_r, residual] = compute_jacobian_and_residual(correspondence);
//                     const double w = GM_weight(residual.squaredNorm());
//                     return LinearSystem(J_r.transpose() * w * J_r,        // JTJ
//                                         J_r.transpose() * w * residual);  // JTr
//                 });
//         },
//         sum_linear_systems);

//     return {JTJ, JTr};
// }

// }  // namespace

// namespace kiss_icp {

// Registration::Registration(int max_num_iteration,
//                            double convergence_criterion,
//                            int max_num_threads)
//     : max_num_iterations_(max_num_iteration),
//       convergence_criterion_(convergence_criterion),
//       max_num_threads_(max_num_threads > 0 ? max_num_threads
//                                            : tbb::this_task_arena::max_concurrency()) {
//     static const auto tbb_control_settings =
//         tbb::global_control(tbb::global_control::max_allowed_parallelism,
//                             static_cast<size_t>(max_num_threads_));
// }

// Sophus::SE3d Registration::AlignPointsToMap(const std::vector<Eigen::Vector3d>& frame,
//                                            const VoxelHashMap& voxel_map,
//                                            const Sophus::SE3d& initial_guess,
//                                            const double max_distance,
//                                            const double kernel_scale) {
//     if (voxel_map.Empty()) return initial_guess;

//     std::vector<Eigen::Vector3d> source = frame;
//     TransformPoints(initial_guess, source);

//     Sophus::SE3d T_icp = Sophus::SE3d();
//     for (int j = 0; j < max_num_iterations_; ++j) {
//         const auto correspondences = DataAssociation(source, voxel_map, max_distance);
//         if (correspondences.empty()) break;

//         const auto sys = BuildLinearSystem(correspondences, kernel_scale);
//         const Eigen::Vector6d dx = sys.first.ldlt().solve(-sys.second);
//         const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);

//         TransformPoints(estimation, source);
//         T_icp = estimation * T_icp;

//         if (dx.norm() < convergence_criterion_) break;
//     }

//     return T_icp * initial_guess;
// }

// }  // namespace kiss_icp

// PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS
// PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS 
// PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS // PURE P2PLANE KISS 
// #include <algorithm>
// #include <cmath>
// #include <limits>
// #include <numeric>
// #include <random>
// #include <tuple>
// #include <vector>

// #include <Eigen/Eigenvalues>
// #include <sophus/se3.hpp>
// #include <sophus/so3.hpp>

// #include <tbb/blocked_range.h>
// #include <tbb/concurrent_vector.h>
// #include <tbb/global_control.h>
// #include <tbb/parallel_for.h>
// #include <tbb/task_arena.h>

// #include "Registration.hpp"
// #include "VoxelHashMap.hpp"
// #include "VoxelUtils.hpp"

// namespace Eigen {
// using Matrix6d   = Eigen::Matrix<double, 6, 6>;
// using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
// using Vector6d   = Eigen::Matrix<double, 6, 1>;
// }  // namespace Eigen

// using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
// using LinearSystem    = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

// namespace {

// inline double square(double x) { return x * x; }

// void TransformPoints(const Sophus::SE3d& T, std::vector<Eigen::Vector3d>& points) {
//     std::transform(points.cbegin(), points.cend(), points.begin(),
//                    [&](const auto& point) { return T * point; });
// }

// Correspondences DataAssociation(const std::vector<Eigen::Vector3d>& points,
//                                 const kiss_icp::VoxelHashMap& voxel_map,
//                                 const double max_correspondance_distance) {
//     using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
//     Correspondences correspondences;
//     correspondences.reserve(points.size());

//     tbb::parallel_for(
//         tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
//         [&](const tbb::blocked_range<points_iterator>& r) {
//             std::for_each(r.begin(), r.end(), [&](const auto& point) {
//                 const auto& [closest_neighbor, distance] = voxel_map.GetClosestNeighbor(point);
//                 if (distance < max_correspondance_distance) {
//                     correspondences.emplace_back(point, closest_neighbor);
//                 }
//             });
//         });

//     return correspondences;
// }

// // ------------------------------
// // Fixed-k KNN (brute force)
// // ------------------------------
// std::vector<Eigen::Vector3d> BruteForceKNN(const std::vector<Eigen::Vector3d>& pts,
//                                            const Eigen::Vector3d& query,
//                                            int k) {
//     std::vector<std::pair<double, int>> dist_idx;
//     dist_idx.reserve(pts.size());

//     for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
//         const double d2 = (pts[i] - query).squaredNorm();
//         if (d2 < 1e-18) continue;
//         dist_idx.emplace_back(d2, i);
//     }

//     if (static_cast<int>(dist_idx.size()) < k) return {};

//     // keep k smallest (unordered ok)
//     std::nth_element(dist_idx.begin(), dist_idx.begin() + (k - 1), dist_idx.end(),
//                      [](const auto& a, const auto& b) { return a.first < b.first; });

//     std::vector<Eigen::Vector3d> knn;
//     knn.reserve(k);
//     for (int j = 0; j < k; ++j) knn.push_back(pts[dist_idx[j].second]);
//     return knn;
// }

// // ------------------------------
// // PCA normal + surface variation
// // ------------------------------
// bool ComputePCA(const std::vector<Eigen::Vector3d>& neighbors,
//                 Eigen::Vector3d& eigenvalues_out,
//                 Eigen::Vector3d& normal_out,
//                 double& surface_variation_out) {
//     if (neighbors.size() < 3) return false;

//     Eigen::Vector3d mean = Eigen::Vector3d::Zero();
//     for (const auto& p : neighbors) mean += p;
//     mean /= static_cast<double>(neighbors.size());

//     Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
//     for (const auto& p : neighbors) {
//         const Eigen::Vector3d d = p - mean;
//         cov += d * d.transpose();
//     }
//     cov /= static_cast<double>(neighbors.size());

//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
//     if (solver.info() != Eigen::Success) return false;

//     const Eigen::Vector3d evals = solver.eigenvalues();   // ascending
//     const Eigen::Matrix3d evecs = solver.eigenvectors();

//     eigenvalues_out = evals;
//     normal_out = evecs.col(0);  // smallest eigenvalue eigenvector = normal

//     const double sum = evals.sum();
//     surface_variation_out = (sum > 1e-18) ? (evals(0) / sum) : 1.0;
//     return true;
// }

// // ==========================================================
// // PURE Point-to-Plane (fixed-k) Linear System
// // - No P2P term
// // - Discard correspondence if plane is not reliable (optional gate)
// // ==========================================================
// LinearSystem BuildLinearSystemPureP2Plane_FixedK(
//     const Correspondences& corrs,
//     const kiss_icp::VoxelHashMap& voxel_map,
//     const double kernel_scale,
//     const int /*voxel_radius_unused*/,
//     int* num_plane_used,
//     int* num_total_corrs) {

//     Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero();
//     Eigen::Vector6d JTr = Eigen::Vector6d::Zero();

//     int plane_used = 0;
//     const int total_corrs = static_cast<int>(corrs.size());

//     const auto GM_weight_scalar = [&](double r2) {
//         return square(kernel_scale) / square(kernel_scale + r2);
//     };

//     // --- Settings ---
//     const int candidate_voxel_radius = 1;
//     const int k = 10;
//     const int min_candidates = k + 3;

//     // Optional plane quality gate (uncomment if you want)
//     // const double sv_threshold = 0.05;

//     for (const auto& c : corrs) {
//         const Eigen::Vector3d& p = c.first;   // source (transformed)
//         const Eigen::Vector3d& q = c.second;  // target in map

//         const auto candidates = voxel_map.GetNeighborhoodCandidates(q, candidate_voxel_radius);
//         if (static_cast<int>(candidates.size()) < min_candidates) continue;

//         const auto neigh = BruteForceKNN(candidates, q, k);
//         if (neigh.empty()) continue;

//         Eigen::Vector3d evals;
//         Eigen::Vector3d n;
//         double sv = 1.0;
//         if (!ComputePCA(neigh, evals, n, sv)) continue;

//         const double nn = n.norm();
//         if (nn < 1e-12) continue;
//         n /= nn;

//         // consistent normal direction
//         if (n.dot(p - q) > 0.0) n = -n;

//         // reliability gate
//         // if (!(sv < sv_threshold)) continue;

//         const double r_plane = n.dot(p - q);

//         Eigen::Vector6d Jt;
//         Jt.head<3>() = n;
//         Jt.tail<3>() = Sophus::SO3d::hat(p) * n;

//         const double w = GM_weight_scalar(r_plane * r_plane);

//         JTJ.noalias() += w * (Jt * Jt.transpose());
//         JTr.noalias() += w * (Jt * r_plane);

//         ++plane_used;
//     }

//     if (num_plane_used) *num_plane_used = plane_used;
//     if (num_total_corrs) *num_total_corrs = total_corrs;

//     return {JTJ, JTr};
// }

// }  // namespace

// namespace kiss_icp {

// Registration::Registration(int max_num_iteration, double convergence_criterion, int max_num_threads)
//     : max_num_iterations_(max_num_iteration),
//       convergence_criterion_(convergence_criterion),
//       max_num_threads_(max_num_threads > 0 ? max_num_threads
//                                            : tbb::this_task_arena::max_concurrency()) {
//     static const auto tbb_control_settings =
//         tbb::global_control(tbb::global_control::max_allowed_parallelism,
//                             static_cast<size_t>(max_num_threads_));
// }

// Sophus::SE3d Registration::AlignPointsToMap(const std::vector<Eigen::Vector3d>& frame,
//                                            const VoxelHashMap& voxel_map,
//                                            const Sophus::SE3d& initial_guess,
//                                            const double max_distance,
//                                            const double kernel_scale) {
//     if (voxel_map.Empty()) return initial_guess;

//     std::vector<Eigen::Vector3d> source = frame;
//     TransformPoints(initial_guess, source);

//     Sophus::SE3d T_icp = Sophus::SE3d();

//     for (int j = 0; j < max_num_iterations_; ++j) {
//         const auto correspondences = DataAssociation(source, voxel_map, max_distance);
//         if (correspondences.empty()) break;

//         int plane_used = 0;
//         int total_corrs = 0;

//         const auto sys = BuildLinearSystemPureP2Plane_FixedK(
//             correspondences, voxel_map, kernel_scale,
//             /*voxel_radius_unused=*/2,
//             &plane_used, &total_corrs);

//         // If too few plane constraints, system may be ill-conditioned -> stop
//         if (plane_used < 20) break;

//         const Eigen::Vector6d dx = sys.first.ldlt().solve(-sys.second);
//         const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);

//         TransformPoints(estimation, source);
//         T_icp = estimation * T_icp;

//         if (dx.norm() < convergence_criterion_) break;
//     }

//     return T_icp * initial_guess;
// }

// }  // namespace kiss_icp

// 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS 
// 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS 
// 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS // 固定k的HYBRID KISS 
// #include <algorithm>
// #include <cmath>
// #include <limits>
// #include <numeric>
// #include <random>
// #include <tuple>
// #include <vector>

// #include <Eigen/Eigenvalues>
// #include <sophus/se3.hpp>
// #include <sophus/so3.hpp>

// #include <tbb/blocked_range.h>
// #include <tbb/concurrent_vector.h>
// #include <tbb/global_control.h>
// #include <tbb/parallel_for.h>
// #include <tbb/task_arena.h>

// #include <iostream>

// #include "Registration.hpp"
// #include "VoxelHashMap.hpp"
// #include "VoxelUtils.hpp"

// namespace Eigen {
// using Matrix6d   = Eigen::Matrix<double, 6, 6>;
// using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
// using Vector6d   = Eigen::Matrix<double, 6, 1>;
// }  // namespace Eigen

// using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
// using LinearSystem    = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

// namespace {

// inline double square(double x) { return x * x; }
// inline double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }

// void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
//     std::transform(points.cbegin(), points.cend(), points.begin(),
//                    [&](const auto &point) { return T * point; });
// }

// Correspondences DataAssociation(const std::vector<Eigen::Vector3d> &points,
//                                 const kiss_icp::VoxelHashMap &voxel_map,
//                                 const double max_correspondance_distance) {
//     using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
//     Correspondences correspondences;
//     correspondences.reserve(points.size());

//     tbb::parallel_for(
//         tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
//         [&](const tbb::blocked_range<points_iterator> &r) {
//             std::for_each(r.begin(), r.end(), [&](const auto &point) {
//                 const auto &[closest_neighbor, distance] = voxel_map.GetClosestNeighbor(point);
//                 if (distance < max_correspondance_distance) {
//                     correspondences.emplace_back(point, closest_neighbor);
//                 }
//             });
//         });

//     return correspondences;
// }

// // ------------------------------
// // Fixed-k KNN (brute force)
// // ------------------------------
// std::vector<Eigen::Vector3d> BruteForceKNN(const std::vector<Eigen::Vector3d> &pts,
//                                            const Eigen::Vector3d &query,
//                                            int k) {
//     std::vector<std::pair<double, int>> dist_idx;
//     dist_idx.reserve(pts.size());

//     for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
//         const double d2 = (pts[i] - query).squaredNorm();
//         if (d2 < 1e-18) continue;
//         dist_idx.emplace_back(d2, i);
//     }

//     if (static_cast<int>(dist_idx.size()) < k) return {};

//     std::nth_element(dist_idx.begin(), dist_idx.begin() + (k - 1), dist_idx.end(),
//                      [](const auto &a, const auto &b) { return a.first < b.first; });

//     std::vector<Eigen::Vector3d> knn;
//     knn.reserve(k);
//     for (int j = 0; j < k; ++j) knn.push_back(pts[dist_idx[j].second]);
//     return knn;
// }

// // ------------------------------
// // PCA normal + surface variation
// // ------------------------------
// bool ComputePCA(const std::vector<Eigen::Vector3d> &neighbors,
//                 Eigen::Vector3d &eigenvalues_out,
//                 Eigen::Vector3d &normal_out,
//                 double &surface_variation_out) {
//     if (neighbors.size() < 3) return false;

//     Eigen::Vector3d mean = Eigen::Vector3d::Zero();
//     for (const auto &p : neighbors) mean += p;
//     mean /= static_cast<double>(neighbors.size());

//     Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
//     for (const auto &p : neighbors) {
//         const Eigen::Vector3d d = p - mean;
//         cov += d * d.transpose();
//     }
//     cov /= static_cast<double>(neighbors.size());

//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
//     if (solver.info() != Eigen::Success) return false;

//     const Eigen::Vector3d evals = solver.eigenvalues();   // ascending
//     const Eigen::Matrix3d evecs = solver.eigenvectors();

//     eigenvalues_out = evals;
//     normal_out = evecs.col(0); // eigenvector of smallest eigenvalue

//     const double sum = evals.sum();
//     surface_variation_out = (sum > 1e-18) ? (evals(0) / sum) : 0.0;

//     return true;
// }

// // ===== Hybrid fixed-k (GenZ-style) =====
// LinearSystem BuildLinearSystemHybridGenZFromCorrs(
//     const Correspondences &corrs,
//     const kiss_icp::VoxelHashMap &voxel_map,
//     const double kernel_scale,
//     const int /*voxel_radius_unused*/,
//     int *num_plane_used,
//     int *num_total_used) {

//     Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero();
//     Eigen::Vector6d JTr = Eigen::Vector6d::Zero();

//     int plane_used = 0;
//     int used_total = 0;

//     auto GM_weight_scalar = [&](double r2) {
//         return square(kernel_scale) / square(kernel_scale + r2);
//     };

//     // --- Settings ---
//     const int candidate_voxel_radius = 1;
//     const int k = 7;
//     const int min_candidates = k + 3;

//     // plane quality threshold
//     const double sv_threshold = 0.5;

//     for (const auto &c : corrs) {
//         const Eigen::Vector3d &p = c.first;
//         const Eigen::Vector3d &q = c.second;

//         const Eigen::Vector3d r_p2p = p - q;

//         Eigen::Matrix3_6d J_p2p;
//         J_p2p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
//         J_p2p.block<3, 3>(0, 3) = -Sophus::SO3d::hat(p);

//         double alpha = 0.0;
//         bool has_plane = false;

//         Eigen::Vector3d n = Eigen::Vector3d::Zero();
//         double sv = 1.0;

//         // Local plane estimation (fixed k)
//         const auto candidates = voxel_map.GetNeighborhoodCandidates(q, candidate_voxel_radius);
//         if (static_cast<int>(candidates.size()) >= min_candidates) {
//             const auto neigh = BruteForceKNN(candidates, q, k);
//             if (!neigh.empty()) {
//                 Eigen::Vector3d evals;
//                 if (ComputePCA(neigh, evals, n, sv)) {
//                     const double nn = n.norm();
//                     if (nn > 1e-12) {
//                         n /= nn;
//                         if (sv < sv_threshold) {
//                             has_plane = true;
//                             alpha = clamp01(1.0 - sv);
//                         } else {
//                             has_plane = false;
//                             alpha = 0.0;
//                         }
//                     }
//                 }
//             }
//         }

//         // P2Plane
//         if (has_plane && alpha > 0.0) {
//             const double r_plane = n.dot(p - q);

//             Eigen::Vector6d Jt;
//             Jt.head<3>() = n;
//             Jt.tail<3>() = Sophus::SO3d::hat(p) * n;

//             const double w_plane = alpha * GM_weight_scalar(r_plane * r_plane);

//             JTJ.noalias() += w_plane * (Jt * Jt.transpose());
//             JTr.noalias() += w_plane * (Jt * r_plane);

//             ++plane_used;
//         }

//         // P2P
//         const double w_p2p = (1.0 - alpha) * GM_weight_scalar(r_p2p.squaredNorm());
//         if (w_p2p > 0.0) {
//             JTJ.noalias() += w_p2p * (J_p2p.transpose() * J_p2p);
//             JTr.noalias() += w_p2p * (J_p2p.transpose() * r_p2p);
//         }

//         ++used_total;
//     }

//     if (num_plane_used) *num_plane_used = plane_used;
//     if (num_total_used) *num_total_used = used_total;

//     return {JTJ, JTr};
// }

// }  // namespace

// namespace kiss_icp {

// Registration::Registration(int max_num_iteration, double convergence_criterion, int max_num_threads)
//     : max_num_iterations_(max_num_iteration),
//       convergence_criterion_(convergence_criterion),
//       max_num_threads_(max_num_threads > 0 ? max_num_threads
//                                            : tbb::this_task_arena::max_concurrency()) {
//     static const auto tbb_control_settings =
//         tbb::global_control(tbb::global_control::max_allowed_parallelism,
//                             static_cast<size_t>(max_num_threads_));
// }

// Sophus::SE3d Registration::AlignPointsToMap(const std::vector<Eigen::Vector3d> &frame,
//                                            const VoxelHashMap &voxel_map,
//                                            const Sophus::SE3d &initial_guess,
//                                            const double max_distance,
//                                            const double kernel_scale) {
//     if (voxel_map.Empty()) return initial_guess;

//     std::vector<Eigen::Vector3d> source = frame;
//     TransformPoints(initial_guess, source);

//     Sophus::SE3d T_icp = Sophus::SE3d();

//     for (int j = 0; j < max_num_iterations_; ++j) {
//         const auto correspondences = DataAssociation(source, voxel_map, max_distance);
//         if (correspondences.empty()) break;

//         int plane_used = 0;
//         int used_total = 0;

//         const auto sys = BuildLinearSystemHybridGenZFromCorrs(
//             correspondences,
//             voxel_map,
//             kernel_scale,
//             /*voxel_radius_unused=*/2,
//             &plane_used,
//             &used_total);

//         const Eigen::Vector6d dx = sys.first.ldlt().solve(-sys.second);
//         const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);

//         TransformPoints(estimation, source);
//         T_icp = estimation * T_icp;

//         if (dx.norm() < convergence_criterion_) break;
//     }

//     return T_icp * initial_guess;
// }

// }  // namespace kiss_icp

// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//
// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//
// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//// 新增weinmann的code//
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

#include <Eigen/Eigenvalues>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include "Registration.hpp"
#include "VoxelHashMap.hpp"
#include "VoxelUtils.hpp"

namespace Eigen {
using Matrix6d   = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d   = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem    = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

namespace {

inline double square(double x) { return x * x; }
inline double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }

void TransformPoints(const Sophus::SE3d& T, std::vector<Eigen::Vector3d>& points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto& point) { return T * point; });
}

Correspondences DataAssociation(const std::vector<Eigen::Vector3d>& points,
                                const kiss_icp::VoxelHashMap& voxel_map,
                                const double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    Correspondences correspondences;
    correspondences.reserve(points.size());

    tbb::parallel_for(
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        [&](const tbb::blocked_range<points_iterator>& r) {
            std::for_each(r.begin(), r.end(), [&](const auto& point) {
                const auto& [closest_neighbor, distance] = voxel_map.GetClosestNeighbor(point);
                if (distance < max_correspondance_distance) {
                    correspondences.emplace_back(point, closest_neighbor);
                }
            });
        });

    return correspondences;
}

// ==========================================================
// Weinmann optNESS settings
// ==========================================================
static constexpr int kKMin   = 4;
static constexpr int kKMax   = 10;
static constexpr int kDeltaK = 1;

// ==========================================================
// Online covariance accumulator (Welford-style, vector version)
// ==========================================================
struct OnlineCov3 {
    int n = 0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d M2   = Eigen::Matrix3d::Zero();

    inline void add(const Eigen::Vector3d& x) {
        ++n;
        const Eigen::Vector3d delta = x - mean;
        mean += delta / double(n);
        const Eigen::Vector3d delta2 = x - mean;
        M2 += delta * delta2.transpose();
    }

    inline bool covariance(Eigen::Matrix3d& C) const {
        if (n < 2) return false;
        C = M2 / double(n - 1);
        return true;
    }
};

inline bool EigenDecompFromCov(const Eigen::Matrix3d& C,
                               Eigen::Vector3d& evals_out_asc,
                               Eigen::Matrix3d& evecs_out) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
    if (es.info() != Eigen::Success) return false;
    evals_out_asc = es.eigenvalues();  // ascending
    evecs_out = es.eigenvectors();
    return true;
}

inline double EigenEntropyFromEvalsAsc(Eigen::Vector3d evals_asc) {
    const double eps = 1e-8;
    evals_asc = evals_asc.cwiseMax(eps);

    const double s = evals_asc.sum();
    if (s <= 0.0) return std::numeric_limits<double>::infinity();

    const Eigen::Vector3d p = evals_asc / s;
    return -(p(0) * std::log(p(0)) + p(1) * std::log(p(1)) + p(2) * std::log(p(2)));
}

// ==========================================================
// KNN sorted, INCLUDE self at index 0 (query + K neighbors)
// ==========================================================
inline std::vector<Eigen::Vector3d> BruteForceKNN_Sorted_IncludeSelf(
    const std::vector<Eigen::Vector3d>& pts,
    const Eigen::Vector3d& query,
    int Kmax_neighbors) {

    std::vector<std::pair<double, int>> dist_idx;
    dist_idx.reserve(pts.size());

    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        const double d2 = (pts[i] - query).squaredNorm();
        if (d2 < 1e-18) continue;
        dist_idx.emplace_back(d2, i);
    }

    const int K = std::min(Kmax_neighbors, static_cast<int>(dist_idx.size()));
    if (K > 0) {
        std::nth_element(dist_idx.begin(), dist_idx.begin() + (K - 1), dist_idx.end(),
                         [](const auto& a, const auto& b) { return a.first < b.first; });
        dist_idx.resize(K);

        std::sort(dist_idx.begin(), dist_idx.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
    }

    std::vector<Eigen::Vector3d> knn;
    knn.reserve(1 + K);
    knn.push_back(query);
    for (const auto& di : dist_idx) knn.push_back(pts[di.second]);
    return knn;
}

inline bool SelectK_Weinmann_OptNESS(const std::vector<Eigen::Vector3d>& knn_sorted,
                                    int k_min, int k_max, int delta_k,
                                    int& best_k_out,
                                    Eigen::Matrix3d& best_cov_out) {
    if (k_min < 3) k_min = 3;
    if (delta_k <= 0) delta_k = 1;
    if (k_max < k_min) return false;

    // knn_sorted includes self at index 0, so we need size >= (k_max + 1)
    if (static_cast<int>(knn_sorted.size()) < (k_max + 1)) return false;

    OnlineCov3 oc;

    double best_H = std::numeric_limits<double>::infinity();
    int best_k = k_min;
    Eigen::Matrix3d best_C = Eigen::Matrix3d::Zero();

    for (int i = 0; i < (k_max + 1); ++i) {
        oc.add(knn_sorted[i]);

        const int k = i;  // because self at 0 -> k is neighbor size in the MATLAB sense
        if (k < k_min) continue;
        if (((k - k_min) % delta_k) != 0) continue;

        Eigen::Matrix3d C;
        if (!oc.covariance(C)) continue;

        Eigen::Vector3d evals_asc;
        Eigen::Matrix3d evecs;
        if (!EigenDecompFromCov(C, evals_asc, evecs)) continue;

        const double H = EigenEntropyFromEvalsAsc(evals_asc);
        if (H < best_H) {
            best_H = H;
            best_k = k;
            best_C = C;
        }
    }

    if (!std::isfinite(best_H)) return false;
    best_k_out = best_k;
    best_cov_out = best_C;
    return true;
}

inline bool NormalAndSVFromCov(const Eigen::Matrix3d& C,
                               Eigen::Vector3d& normal_out,
                               double& sv_out) {
    Eigen::Vector3d evals_asc;
    Eigen::Matrix3d evecs;
    if (!EigenDecompFromCov(C, evals_asc, evecs)) return false;

    Eigen::Vector3d n = evecs.col(0);
    const double nn = n.norm();
    if (nn < 1e-12) return false;
    n /= nn;

    const double sum = evals_asc.sum();
    if (sum <= 1e-18) return false;

    sv_out = evals_asc(0) / sum;
    normal_out = n;
    return true;
}

// ==========================================================
// Hybrid (GenZ-style) using optNESS k + PCA
// ==========================================================
LinearSystem BuildLinearSystemHybridWeinmannFromCorrs(
    const Correspondences& corrs,
    const kiss_icp::VoxelHashMap& voxel_map,
    const double kernel_scale,
    const int /*voxel_radius_unused*/,
    int* num_plane_used,
    int* num_total_used) {

    Eigen::Matrix6d JTJ = Eigen::Matrix6d::Zero();
    Eigen::Vector6d JTr = Eigen::Vector6d::Zero();

    int plane_used = 0;
    int used_total = 0;

    const auto GM_weight_scalar = [&](double r2) {
        return square(kernel_scale) / square(kernel_scale + r2);
    };

    // Neighborhood candidates
    const int candidate_voxel_radius = 1;
    const int Kmax = 150;

    // your gating (keep as-is)
    const double sv_gate = 0.3;
    const int k_gate = 4;

    for (const auto& c : corrs) {
        const Eigen::Vector3d& p = c.first;
        const Eigen::Vector3d& q = c.second;

        const Eigen::Vector3d r_p2p = p - q;

        Eigen::Matrix3_6d J_p2p;
        J_p2p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_p2p.block<3, 3>(0, 3) = -Sophus::SO3d::hat(p);

        double alpha = 0.0;
        bool has_plane = false;
        Eigen::Vector3d n = Eigen::Vector3d::Zero();
        double sv = 1.0;
        int best_k = -1;

        const auto candidates = voxel_map.GetNeighborhoodCandidates(q, candidate_voxel_radius);
        if (static_cast<int>(candidates.size()) >= (kKMin + 3)) {
            const auto knn_sorted = BruteForceKNN_Sorted_IncludeSelf(candidates, q, Kmax);

            if (static_cast<int>(knn_sorted.size()) >= (kKMax + 1)) {
                Eigen::Matrix3d best_cov;
                if (SelectK_Weinmann_OptNESS(knn_sorted, kKMin, kKMax, kDeltaK, best_k, best_cov)) {
                    if (NormalAndSVFromCov(best_cov, n, sv)) {
                        if (n.dot(p - q) > 0.0) n = -n;

                        const bool enable_plane = (sv < sv_gate) && (best_k >= k_gate);
                        if (enable_plane) {
                            has_plane = true;
                            alpha = clamp01(1.0 - sv);
                        } else {
                            has_plane = false;
                            alpha = 0.0;
                        }
                    }
                }
            }
        }

        // P2Plane term
        if (has_plane && alpha > 0.0) {
            const double r_plane = n.dot(p - q);

            Eigen::Vector6d Jt;
            Jt.head<3>() = n;
            Jt.tail<3>() = Sophus::SO3d::hat(p) * n;

            const double w_plane = alpha * GM_weight_scalar(r_plane * r_plane);
            JTJ.noalias() += w_plane * (Jt * Jt.transpose());
            JTr.noalias() += w_plane * (Jt * r_plane);
            ++plane_used;
        }

        // P2P term
        const double w_p2p = (1.0 - alpha) * GM_weight_scalar(r_p2p.squaredNorm());
        if (w_p2p > 0.0) {
            JTJ.noalias() += w_p2p * (J_p2p.transpose() * J_p2p);
            JTr.noalias() += w_p2p * (J_p2p.transpose() * r_p2p);
        }

        ++used_total;
    }

    if (num_plane_used) *num_plane_used = plane_used;
    if (num_total_used) *num_total_used = used_total;
    return {JTJ, JTr};
}

}  // namespace

namespace kiss_icp {

Registration::Registration(int max_num_iteration, double convergence_criterion, int max_num_threads)
    : max_num_iterations_(max_num_iteration),
      convergence_criterion_(convergence_criterion),
      max_num_threads_(max_num_threads > 0 ? max_num_threads
                                           : tbb::this_task_arena::max_concurrency()) {
    static const auto tbb_control_settings = tbb::global_control(
        tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_num_threads_));
}

Sophus::SE3d Registration::AlignPointsToMap(const std::vector<Eigen::Vector3d>& frame,
                                           const VoxelHashMap& voxel_map,
                                           const Sophus::SE3d& initial_guess,
                                           const double max_distance,
                                           const double kernel_scale) {
    if (voxel_map.Empty()) return initial_guess;

    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    Sophus::SE3d T_icp = Sophus::SE3d();

    for (int j = 0; j < max_num_iterations_; ++j) {
        const auto correspondences = DataAssociation(source, voxel_map, max_distance);
        if (correspondences.empty()) break;

        int plane_used = 0;
        int used_total = 0;

        const auto sys = BuildLinearSystemHybridWeinmannFromCorrs(
            correspondences, voxel_map, kernel_scale,
            /*voxel_radius_unused=*/2,
            &plane_used, &used_total);

        const Eigen::Vector6d dx = sys.first.ldlt().solve(-sys.second);
        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);

        TransformPoints(estimation, source);
        T_icp = estimation * T_icp;

        if (dx.norm() < convergence_criterion_) break;
    }

    return T_icp * initial_guess;
}

}  // namespace kiss_icp