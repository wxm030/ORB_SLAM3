/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include <sophus/se3.h>

namespace ORB_SLAM3
{
    const int WarpHalfPatchSize = 4;
    const int WarpPatchSize = 8;

    class ORBmatcher
    {
    public:
        ORBmatcher(float nnratio = 0.6, bool checkOri = true);

        // Computes the Hamming distance between two ORB descriptors
        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

        // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
        // Used to track the local map (Tracking)
        // 和local map之间的projection
        /**
         * @brief 通过投影，对Local MapPoint进行跟踪
         *
         * 将Local MapPoint投影到当前帧中, 由此增加当前帧的MapPoints \n
         * 在SearchLocalPoints()中已经将Local MapPoints重投影（isInFrustum()）到当前帧 \n
         * 并标记了这些点是否在当前帧的视野中，即mbTrackInView \n
         * 对这些MapPoints，在其投影点附近根据描述子距离选取匹配，以及最终的方向投票机制进行剔除
         * @param  F           当前帧
         * @param  vpMapPoints Local MapPoints
         * @param  th          阈值，窗口的大小倍数
         * @param checkLevel   是否检测金字塔的层数？(在类DSO提取算法中，只有第0层金字塔的特征点)
         * @return             成功匹配的数量
         * @see SearchLocalPoints() isInFrustum()
         */
        int SearchByProjection(Frame &F, const std::vector<MapPoint *> &vpMapPoints, const float th = 3, const bool bFarPoints = false, const float thFarPoints = 50.0f, bool checkLevel = true);

        // Project MapPoints tracked in last frame into the current frame and search matches.
        // Used to track from previous frame (Tracking)
        // 和上一帧之间的projection
        // @param[th] 窗口的大小
        /**
         * @brief 通过投影，对上一帧的特征点进行跟踪
         *
         * 上一帧中包含了MapPoints，对这些MapPoints进行tracking，由此增加当前帧的MapPoints \n
         * 1. 将上一帧的MapPoints投影到当前帧(根据速度模型可以估计当前帧的Tcw)
         * 2. 在投影点附近根据描述子距离选取匹配，以及最终的方向投票机制进行剔除
         * @param  CurrentFrame 当前帧
         * @param  LastFrame    上一帧
         * @param  th           阈值
         * @param  bMono        是否为单目
         * @param  checkLevel   是否检查层数
         * @return              成功匹配的数量
         * @see SearchByBoW()
         */
        int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono, bool checkLevel = true);

        // Project MapPoints seen in KeyFrame into the Frame and search matches.
        // Used in relocalisation (Tracking)
        int SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const std::set<MapPoint *> &sAlreadyFound, const float th, const int ORBdist);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in loop detection (Loop Closing)
        int SearchByProjection(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints, std::vector<MapPoint *> &vpMatched, int th, float ratioHamming = 1.0);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in Place Recognition (Loop Closing and Merging)
        int SearchByProjection(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints, const std::vector<KeyFrame *> &vpPointsKFs, std::vector<MapPoint *> &vpMatched, std::vector<KeyFrame *> &vpMatchedKF, int th, float ratioHamming = 1.0);

        // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
        // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
        // Used in Relocalisation and Loop Detection
        int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint *> &vpMapPointMatches);
        int SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12);

        // Matching for the Map Initialization (only used in the monocular case)
        int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize = 10);

        // Matching to triangulate new MapPoints. Check Epipolar Constraint.
        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                                   std::vector<pair<size_t, size_t>> &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse = false);

        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                                   vector<pair<size_t, size_t>> &vMatchedPairs, const bool bOnlyStereo, vector<cv::Mat> &vMatchedPoints);

        // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
        // In the stereo and RGB-D case, s12=1
        int SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

        // Project MapPoints into KeyFrame and search for duplicated MapPoints.
        int Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th = 3.0, const bool bRight = false);

        // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
        int Fuse(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

        /************************************/
        // 直接法的匹配
        // 用直接法判断能否从在当前图像上找到某地图点的投影
        // todo 这个函数经常会有误拒的情况，需要进一步检查。
        bool FindDirectProjection(KeyFrame *ref, Frame *curr, MapPoint *mp, Eigen::Vector2d &px_curr, int &search_level);

    public:
        static const int TH_LOW;
        static const int TH_HIGH;
        static const int HISTO_LENGTH;

    protected:
        bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF, const bool b1 = false);
        bool CheckDistEpipolarLine2(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF, const float unc);

        float RadiusByViewingCos(const float &viewCos);

        void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

        //迭代光流匹配
        /**
         * @brief align a pixel with reference image patch
         * @param[in] cur_img The current image
         * @param[in] ref_patch_with_boarder the patch with boarder, used to compute the gradient (or FEJ)
         * @param[in] ref_patch the patch in reference frame, by default is 64x64
         * @param[in] n_iter maximum iterations
         * @param[out] cur_px_estimate the estimated position in current image, must have an initial value
         * @return True if successful
        */
        bool Align2D(const cv::Mat &cur_img, uint8_t *ref_patch_with_border, uint8_t *ref_patch, const int n_iter, Eigen::Vector2d &cur_px_estimate, bool no_simd);

        // 计算affine wrap矩阵
        void GetWarpAffineMatrix(KeyFrame *ref, Frame *curr, const Eigen::Vector2d &px_ref, MapPoint *mp, int level, const Sophus::SE3 &TCR, Eigen::Matrix2d &ACR);

        // perform affine warp
        void WarpAffine(const Eigen::Matrix2d &ACR, const cv::Mat &img_ref, const Eigen::Vector2d &px_ref, const int &level_ref, const KeyFrame *ref, const int &search_level, const int &half_patch_size, uint8_t *patch);

        // 计算最好的金字塔层数
        // 选择一个分辨率，使得warp不要太大
        // ORB每层金字塔默认是1.2倍缩放，所以每缩小一层是1.2*1.2=1.44,取倒数为0.694444444444
        inline int GetBestSearchLevel(const Eigen::Matrix2d &ACR, const int &max_level, const KeyFrame *ref)
        {
            int search_level = 0;
            float D = ACR.determinant();
            while (D > 3.0 && search_level < max_level)
            {
                search_level += 1;
                D *= ref->mvInvLevelSigma2[1];
            }
            return search_level;
        }

        // 双线性插值
        inline uchar GetBilateralInterpUchar(const double &x, const double &y, const cv::Mat &gray)
        {
            const double xx = x - floor(x);
            const double yy = y - floor(y);
            uchar *data = &gray.data[int(y) * gray.step + int(x)];
            return uchar(
                (1 - xx) * (1 - yy) * data[0] +
                xx * (1 - yy) * data[1] +
                (1 - xx) * yy * data[gray.step] +
                xx * yy * data[gray.step + 1]);
        }

        // 匹配局部地图用的 patch, 默认8x8
        uchar _patch[WarpPatchSize * WarpPatchSize];
        // 带边界的，左右各1个像素
        uchar _patch_with_border[(WarpPatchSize + 2) * (WarpPatchSize + 2)];

        float mfNNratio;
        bool mbCheckOrientation;
    };

} // namespace ORB_SLAM3

#endif // ORBMATCHER_H
