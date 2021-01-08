// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef ORB_SLAM3_SPARSE_IMG_ALIGN_H_
#define ORB_SLAM3_SPARSE_IMG_ALIGN_H_

#include <nlls_solver.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "Frame.h"
#include "Converter.h"
#include "MapPoint.h"

namespace ORB_SLAM3
{

  /// Optimize the pose of the frame by minimizing the photometric error of feature patches.
  class SparseImgAlign : public ORB_SLAM3::NLLSSolver<6, SE3>
  {
    static const int patch_halfsize_ = 2;
    static const int patch_size_ = 2 * patch_halfsize_;
    static const int patch_area_ = patch_size_ * patch_size_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cv::Mat resimg_;

    SparseImgAlign(
        int n_levels,
        int min_level,
        int n_iter,
        Method method,
        bool display,
        bool verbose);

    size_t run(Frame *ref_frame, Frame *cur_frame, cv::Mat &TCR);

    /// Return fisher information matrix, i.e. the Hessian of the log-likelihood
    /// at the converged state.
    Matrix<double, 6, 6> getFisherInformation();

  protected:
    Frame *ref_frame_; //!< reference frame, has depth for gradient pixels.
    Frame *cur_frame_; //!< only the image is known!
    int level_;        //!< current pyramid level on which the optimization runs.
    bool display_;     //!< display residual image.
    int max_level_;    //!< coarsest pyramid level for the alignment.
    int min_level_;    //!< finest pyramid level for the alignment.

    // cache:
    Matrix<double, 6, Dynamic, ColMajor> jacobian_cache_;
    bool have_ref_patch_cache_;
    cv::Mat ref_patch_cache_;
    std::vector<bool> visible_fts_;

    void precomputeReferencePatches();
    virtual double computeResiduals(const SE3 &model, bool linearize_system, bool compute_weight_scale = false);
    virtual int solve();
    virtual void update(const ModelType &old_model, ModelType &new_model);
    virtual void startIteration();
    virtual void finishIteration();

    // *************************************************************************************
    // 一些固定的雅可比
    // xyz 到 相机坐标 的雅可比，平移在前
    // 这里已经取了负号，不要再取一遍！
    inline Eigen::Matrix<double, 2, 6> JacobXYZ2Cam(const Vector3f &xyz)
    {
      Eigen::Matrix<double, 2, 6> J;
      const float x = xyz[0];
      const float y = xyz[1];
      const float z_inv = 1. / xyz[2];
      const float z_inv_2 = z_inv * z_inv;

      J(0, 0) = -z_inv;               // -1/z
      J(0, 1) = 0.0;                  // 0
      J(0, 2) = x * z_inv_2;          // x/z^2
      J(0, 3) = y * J(0, 2);          // x*y/z^2
      J(0, 4) = -(1.0 + x * J(0, 2)); // -(1.0 + x^2/z^2)
      J(0, 5) = y * z_inv;            // y/z

      J(1, 0) = 0.0;               // 0
      J(1, 1) = -z_inv;            // -1/z
      J(1, 2) = y * z_inv_2;       // y/z^2
      J(1, 3) = 1.0 + y * J(1, 2); // 1.0 + y^2/z^2
      J(1, 4) = -J(0, 3);          // -x*y/z^2
      J(1, 5) = -x * z_inv;        // x/z
      return J;
    }
  };

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_SPARSE_IMG_ALIGN_H_
