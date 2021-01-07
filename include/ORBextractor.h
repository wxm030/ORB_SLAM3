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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>
#include <fast/fast.h>
#include "Frame.h"

namespace ORB_SLAM3
{

    class ExtractorNode
    {
    public:
        ExtractorNode() : bNoMore(false) {}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    /// Temporary container used for corner detection. Features are initialized from these.
    struct Corner
    {
        int x;       //!< x-coordinate of corner in the image.
        int y;       //!< y-coordinate of corner in the image.
        int level;   //!< pyramid level of the corner.
        float score; //!< shi-tomasi score of the corner.
        float angle; //!< for gradient-features: dominant gradient angle.
        int size;    //keypoint diameter
        Corner(int x, int y, float score, int level, float angle, int size) : x(x), y(y), level(level), score(score), angle(angle), size(size)
        {
        }
    };
    typedef std::vector<Corner> Corners;

    class ORBextractor
    {
    public:
        enum
        {
            HARRIS_SCORE = 0,
            FAST_SCORE = 1
        };

        // select the keypoint method, support original ORB-SLAM, Grid FAST in SVO and a dynamic grid FAST (DSO like)
        // 使用哪种方法提取特征点，支持原版ORB,SVO里的网格FAST，以及类DSO的变网格FAST
        // 相对来说，原版 ORB-SLAM 的特征提取重复性较好，但比较慢。后两个快一些，但重复性差一些
        typedef enum
        {
            ORBSLAM_KEYPOINT,
            FAST_KEYPOINT,
            DSO_KEYPOINT
        } KeyPointMethod;

        ORBextractor(int nfeatures, float scaleFactor, int nfeaturePyrlevels, int nImgPyrlevel, int mgridSize,
                     int iniThFAST, int minThFAST);

        ~ORBextractor() {}

        // Compute the ORB features and descriptors on an image.
        // ORB are dispersed on the image using an octree.
        // Mask is ignored in the current implementation.
        int operator()(cv::InputArray _image, cv::InputArray _mask,
                       std::vector<cv::KeyPoint> &_keypoints,
                       cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

        // detect features for frame
        // 给某个单独的Frame调用的接口
        void operator()(Frame *frame,
                        std::vector<cv::KeyPoint> &keypoints,
                        cv::OutputArray descriptors,
                        KeyPointMethod method,
                        bool leftEye = true // 是否是左眼（如果是双目的左眼，就要考虑现有的特征点，如果是右眼就可以随便提
        );

        int inline GetLevels() //获取图像金字塔
        {
            return nImgPyrlevel;
        }

        int inline GetFeatureLevels() //获取特征点金字塔
        {
            return nFeaturePyrlevels;
        }

        int inline GetGridSize()
        {
            return mGridSize;
        }

        float inline GetScaleFactor()
        {
            return scaleFactor;
        }

        std::vector<float> inline GetScaleFactors()
        {
            return mvScaleFactor;
        }

        std::vector<float> inline GetInverseScaleFactors()
        {
            return mvInvScaleFactor;
        }

        std::vector<float> inline GetScaleSigmaSquares()
        {
            return mvLevelSigma2;
        }

        std::vector<float> inline GetInverseScaleSigmaSquares()
        {
            return mvInvLevelSigma2;
        }

        std::vector<cv::Mat> mvImagePyramid;

        void ComputePyramid(cv::Mat image);

    protected:
        // use the original fast lib to compute keypoints, faster than Opencv's implementation
        void ComputeKeyPointsFast(std::vector<std::vector<cv::KeyPoint>> &allKeypoints,
                                  std::vector<cv::KeyPoint> &exist_kps);
        // Shi-Tomasi 分数，这个分数越高则特征越优先
        float ShiTomasiScore(const cv::Mat &img, const int &u, const int &v) const;

        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                    const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

        void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);

        // Single level DSO
        // 单层的DSO，只有原始图像分辨率下的特征点
        void ComputeKeyPointsDSOSingleLevel(
            std::vector<cv::KeyPoint> &allKeypoints,
            std::vector<cv::KeyPoint> &exist_kps);
        int mnGridSize{-1}; // dynamic grid size used in DSO
        cv::Mat mOccupancy;

        std::vector<cv::Point> pattern;

        int nfeatures;
        double scaleFactor;
        int nFeaturePyrlevels;
        int nImgPyrlevel;
        int mGridSize;
        int iniThFAST;
        int minThFAST;

        std::vector<int> mnFeaturesPerLevel;

        std::vector<int> umax;

        std::vector<float> mvScaleFactor;
        std::vector<float> mvInvScaleFactor;
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;

        // grid fast
        /// 注意这里网格大小对特征重复性影响非常明显，一定不要调得太大！
        /// 特征重复性会直接影响到新地图点的生成。在5的时候可以生成大约100+个点，10的时候就只有20-50个点了,20时一般为个位数
        /// 然而网格太小则使得地图点过多，影响性能
        int mnGridCols = 0;
        int mnGridRows = 0;
        std::vector<bool> mvbGridOccupancy;
    };

} // namespace ORB_SLAM3

#endif
