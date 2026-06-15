// MIT License

// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// #include "VoxelHashMap.hpp"

// #include <Eigen/Core>
// #include <algorithm>
// #include <array>
// #include <sophus/se3.hpp>
// #include <vector>

// #include "VoxelUtils.hpp"

// namespace {
// using kiss_icp::Voxel;
// static const std::array<Voxel, 27> voxel_shifts{
//     {Voxel{0, 0, 0},   Voxel{1, 0, 0},   Voxel{-1, 0, 0},  Voxel{0, 1, 0},   Voxel{0, -1, 0},
//      Voxel{0, 0, 1},   Voxel{0, 0, -1},  Voxel{1, 1, 0},   Voxel{1, -1, 0},  Voxel{-1, 1, 0},
//      Voxel{-1, -1, 0}, Voxel{1, 0, 1},   Voxel{1, 0, -1},  Voxel{-1, 0, 1},  Voxel{-1, 0, -1},
//      Voxel{0, 1, 1},   Voxel{0, 1, -1},  Voxel{0, -1, 1},  Voxel{0, -1, -1}, Voxel{1, 1, 1},
//      Voxel{1, 1, -1},  Voxel{1, -1, 1},  Voxel{1, -1, -1}, Voxel{-1, 1, 1},  Voxel{-1, 1, -1},
//      Voxel{-1, -1, 1}, Voxel{-1, -1, -1}}};
// }  // namespace

// namespace kiss_icp {

// std::tuple<Eigen::Vector3d, double> VoxelHashMap::GetClosestNeighbor(
//     const Eigen::Vector3d &query) const {
//     // Convert the point to voxel coordinates
//     const auto &voxel = PointToVoxel(query, voxel_size_);
//     // Find the nearest neighbor
//     Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
//     double closest_distance = std::numeric_limits<double>::max();
//     std::for_each(voxel_shifts.cbegin(), voxel_shifts.cend(), [&](const auto &voxel_shift) {
//         const auto &query_voxel = voxel + voxel_shift;
//         auto search = map_.find(query_voxel);
//         if (search != map_.end()) {
//             const auto &points = search.value();
//             const Eigen::Vector3d &neighbor = *std::min_element(
//                 points.cbegin(), points.cend(), [&](const auto &lhs, const auto &rhs) {
//                     return (lhs - query).norm() < (rhs - query).norm();
//                 });
//             double distance = (neighbor - query).norm();
//             if (distance < closest_distance) {
//                 closest_neighbor = neighbor;
//                 closest_distance = distance;
//             }
//         }
//     });
//     return std::make_tuple(closest_neighbor, closest_distance);
// }

// std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const {
//     std::vector<Eigen::Vector3d> points;
//     points.reserve(map_.size() * static_cast<size_t>(max_points_per_voxel_));
//     std::for_each(map_.cbegin(), map_.cend(), [&](const auto &map_element) {
//         const auto &voxel_points = map_element.second;
//         points.insert(points.end(), voxel_points.cbegin(), voxel_points.cend());
//     });
//     points.shrink_to_fit();
//     return points;
// }

// void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points,
//                           const Eigen::Vector3d &origin) {
//     AddPoints(points);
//     RemovePointsFarFromLocation(origin);
// }

// void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) {
//     std::vector<Eigen::Vector3d> points_transformed(points.size());
//     std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
//                    [&](const auto &point) { return pose * point; });
//     const Eigen::Vector3d &origin = pose.translation();
//     Update(points_transformed, origin);
// }

// void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
//     const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
//     std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
//         const auto voxel = PointToVoxel(point, voxel_size_);
//         auto search = map_.find(voxel);
//         if (search != map_.end()) {
//             auto &voxel_points = search.value();
//             if (voxel_points.size() == max_points_per_voxel_ ||
//                 std::any_of(voxel_points.cbegin(), voxel_points.cend(),
//                             [&](const auto &voxel_point) {
//                                 return (voxel_point - point).norm() < map_resolution;
//                             })) {
//                 return;
//             }
//             voxel_points.emplace_back(point);
//         } else {
//             std::vector<Eigen::Vector3d> voxel_points;
//             voxel_points.reserve(max_points_per_voxel_);
//             voxel_points.emplace_back(point);
//             map_.insert({voxel, std::move(voxel_points)});
//         }
//     });
// }

// void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
//     const auto max_distance2 = max_distance_ * max_distance_;
//     for (auto it = map_.begin(); it != map_.end();) {
//         const auto &[voxel, voxel_points] = *it;
//         const auto &pt = voxel_points.front();
//         if ((pt - origin).squaredNorm() >= (max_distance2)) {
//             it = map_.erase(it);
//         } else {
//             ++it;
//         }
//     }
// }
// }  // namespace kiss_icp


// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//
// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//
// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//// 新增快取的code//

#include "VoxelHashMap.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sophus/se3.hpp>
#include <vector>

#include "VoxelUtils.hpp"


namespace {
using kiss_icp::Voxel;
static const std::array<Voxel, 27> voxel_shifts{
    {Voxel{0, 0, 0},   Voxel{1, 0, 0},   Voxel{-1, 0, 0},  Voxel{0, 1, 0},   Voxel{0, -1, 0},
     Voxel{0, 0, 1},   Voxel{0, 0, -1},  Voxel{1, 1, 0},   Voxel{1, -1, 0},  Voxel{-1, 1, 0},
     Voxel{-1, -1, 0}, Voxel{1, 0, 1},   Voxel{1, 0, -1},  Voxel{-1, 0, 1},  Voxel{-1, 0, -1},
     Voxel{0, 1, 1},   Voxel{0, 1, -1},  Voxel{0, -1, 1},  Voxel{0, -1, -1}, Voxel{1, 1, 1},
     Voxel{1, 1, -1},  Voxel{1, -1, 1},  Voxel{1, -1, -1}, Voxel{-1, 1, 1},  Voxel{-1, 1, -1},
     Voxel{-1, -1, 1}, Voxel{-1, -1, -1}}};
}  // namespace

namespace kiss_icp {

std::tuple<Eigen::Vector3d, double> VoxelHashMap::GetClosestNeighbor(
    const Eigen::Vector3d &query) const {
    // Convert the point to voxel coordinates
    const auto &voxel = PointToVoxel(query, voxel_size_);
    // Find the nearest neighbor
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    double closest_distance = std::numeric_limits<double>::max();
    std::for_each(voxel_shifts.cbegin(), voxel_shifts.cend(), [&](const auto &voxel_shift) {
        const auto &query_voxel = voxel + voxel_shift;
        auto search = map_.find(query_voxel);
        if (search != map_.end()) {
            const auto &points = search.value();
            const Eigen::Vector3d &neighbor = *std::min_element(
                points.cbegin(), points.cend(), [&](const auto &lhs, const auto &rhs) {
                    return (lhs - query).norm() < (rhs - query).norm();
                });
            double distance = (neighbor - query).norm();
            if (distance < closest_distance) {
                closest_neighbor = neighbor;
                closest_distance = distance;
            }
        }
    });
    return std::make_tuple(closest_neighbor, closest_distance);
}

// 改的
std::vector<Eigen::Vector3d> VoxelHashMap::GetNeighborhoodCandidates(
    const Eigen::Vector3d &query, int voxel_radius) const {
    std::vector<Eigen::Vector3d> candidates;

    if (map_.empty()) return candidates;

    // Convert the point to voxel coordinates
    const auto &center = PointToVoxel(query, voxel_size_);

    // For now, support radius=1 efficiently using the same 27 shifts
    if (voxel_radius <= 1) {
        // Rough reserve: 27 voxels * max_points_per_voxel_
        candidates.reserve(27ULL * static_cast<size_t>(max_points_per_voxel_));

        std::for_each(voxel_shifts.cbegin(), voxel_shifts.cend(), [&](const auto &shift) {
            const auto v = center + shift;
            auto it = map_.find(v);
            if (it != map_.end()) {
                const auto &pts = it.value();
                candidates.insert(candidates.end(), pts.cbegin(), pts.cend());
            }
        });
        return candidates;
    }

    // Generic radius > 1 (optional). This is heavier but correct.
    const int r = voxel_radius;
    const size_t side = static_cast<size_t>(2 * r + 1);
    candidates.reserve(side * side * side * static_cast<size_t>(max_points_per_voxel_));

    for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dz = -r; dz <= r; ++dz) {
                const auto v = center + Voxel{dx, dy, dz};
                auto it = map_.find(v);
                if (it != map_.end()) {
                    const auto &pts = it.value();
                    candidates.insert(candidates.end(), pts.cbegin(), pts.cend());
                }
            }
        }
    }
    return candidates;
}
// 改的



// ===== Route A: Voxel-level plane cache =====

std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(map_.size() * static_cast<size_t>(max_points_per_voxel_));
    std::for_each(map_.cbegin(), map_.cend(), [&](const auto &map_element) {
        const auto &voxel_points = map_element.second;
        points.insert(points.end(), voxel_points.cbegin(), voxel_points.cend());
    });
    points.shrink_to_fit();
    return points;
}

void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points,
                          const Eigen::Vector3d &origin) {
    AddPoints(points);
    RemovePointsFarFromLocation(origin);
}

// ===== Route A: Voxel-level plane cache =====

void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) {
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Eigen::Vector3d &origin = pose.translation();
    Update(points_transformed, origin);
}

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        const auto voxel = PointToVoxel(point, voxel_size_);
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_points = search.value();
            if (voxel_points.size() == max_points_per_voxel_ ||
                std::any_of(voxel_points.cbegin(), voxel_points.cend(),
                            [&](const auto &voxel_point) {
                                return (voxel_point - point).norm() < map_resolution;
                            })) {
                return;
            }
            voxel_points.emplace_back(point);
        } else {
            std::vector<Eigen::Vector3d> voxel_points;
            voxel_points.reserve(max_points_per_voxel_);
            voxel_points.emplace_back(point);
            map_.insert({voxel, std::move(voxel_points)});
        }
    });
}

void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    const auto max_distance2 = max_distance_ * max_distance_;
    for (auto it = map_.begin(); it != map_.end();) {
        const auto &[voxel, voxel_points] = *it;
        const auto &pt = voxel_points.front();
        if ((pt - origin).squaredNorm() >= (max_distance2)) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}
}  // namespace kiss_icp
