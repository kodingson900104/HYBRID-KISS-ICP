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
#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

#include "VoxelHashMap.hpp"

namespace kiss_icp {

struct Registration {
    explicit Registration(int max_num_iteration, double convergence_criterion, int max_num_threads);

    Sophus::SE3d AlignPointsToMap(const std::vector<Eigen::Vector3d> &frame,
                                  const VoxelHashMap &voxel_map,
                                  const Sophus::SE3d &initial_guess,
                                  const double max_correspondence_distance,
                                  const double kernel_scale);

    // ----------------------------------------------------------
    // [NEW] Optional: set target frame for index-to-index RMSE
    // - For two-frame synthetic datasets where point i corresponds to point i
    // - If not set (nullptr), index RMSE is disabled and won't be reported
    // ----------------------------------------------------------
    void SetIndexRMSETargetFrame(const std::vector<Eigen::Vector3d>* target_frame);

    int max_num_iterations_;
    double convergence_criterion_;
    int max_num_threads_;

    // [NEW] stored pointer (not owning)
    const std::vector<Eigen::Vector3d>* rmse_target_frame_ = nullptr;
};

}  // namespace kiss_icp

