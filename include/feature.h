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

#ifndef ORB_SLAM3_FEATURE_H_
#define ORB_SLAM3_FEATURE_H_

#include <KeyFrame.h>

namespace ORB_SLAM3 {

/// A salient image region that is tracked across frames.
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum FeatureType {
    CORNER,
    EDGELET
  };

  FeatureType type = CORNER;     //!< Type can be corner or edgelet.
  KeyFrame* frame;         //!< Pointer to frame in which the feature was detected.
  Eigen::Vector2d px;          //!< Coordinates in pixels on pyramid level 0.
  Eigen::Vector3d f;           //!< Unit-bearing vector of the feature.
  int level;            //!< Image pyramid level where feature was extracted.
  Eigen::Vector2d grad;        //!< Dominant gradient direction for edglets, normalized.
};

} // namespace svo

#endif // ORB_SLAM3_FEATURE_H_
