#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <opencv2/opencv.hpp>
#include "MapPoint.h"
#include "Frame.h"
#include "GeometricCamera.h"
#include "Converter.h"
#include "feature.h"
#include <sophus/se3.h>
#include "feature_alignment.h"

namespace ORB_SLAM3
{
    class IterativeMatcher
    {
        /// A candidate is a point that projects into the image plane and for which we
        /// will search a maching feature in the image.
        struct Candidate
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            MapPoint *pt;       //!< 3D point.
            Eigen::Vector2d px; //!< projected 2D pixel location.
            Candidate(MapPoint *pt, Eigen::Vector2d &px) : pt(pt), px(px) {}
        };

        typedef std::list<Candidate> Cell;
        typedef std::vector<Cell *> CandidateGrid;

        /// The grid stores a set of candidate matches. For every grid cell we try to find one match.
        struct Grid
        {
            CandidateGrid cells;
            std::vector<int> cell_order;
            int cell_size;
            int grid_n_cols;
            int grid_n_rows;
        };

    public:
        static const int halfpatch_size_ = 4;
        static const int patch_size_ = 8;
        uint8_t patch_[patch_size_ * patch_size_] __attribute__((aligned(16)));
        uint8_t patch_with_border_[(patch_size_ + 2) * (patch_size_ + 2)] __attribute__((aligned(16)));

        int n_level_;
        int mCell_size;
        int mWidth;
        int mHeight;

        size_t n_matches_;
        size_t n_trials_;
        int search_level_;

        Eigen::Matrix2d A_cur_ref_;

        IterativeMatcher(int n_level, int cell_size = 10, int width = 640, int height = 480);
        ~IterativeMatcher();

        int ReprojectMap(Frame &frame, std::vector<MapPoint *> &local_points);
        bool GetCloseViewObs(MapPoint *pt, const Eigen::Vector3d &framepos, Feature &ref_ftr);
        void createPatchFromPatchWithBorder();
        bool FindMatchDirect(MapPoint *pt, const Frame &cur_frame, Eigen::Vector2d &px_cur, int &level);

    private:
        void InitializeGrid();
        void ResetGrid();
        bool ReprojectCell(Cell &cell, Frame &frame);

        Grid grid_;
    };

    /// Warp a patch from the reference view to the current view.
    namespace warp
    {
        void getWarpMatrixAffine(
            GeometricCamera *cam_ref,
            GeometricCamera *cam_cur,
            const Eigen::Vector2d &px_ref,
            const Eigen::Vector3d &f_ref,
            const Sophus::SE3 &T_cur_ref,
            const int level_ref,
            Eigen::Matrix2d &A_cur_ref);

        int getBestSearchLevel(
            Eigen::Matrix2d &A_cur_ref,
            const int max_level);

        void warpAffine(
            Eigen::Matrix2d &A_cur_ref,
            const cv::Mat &img_ref,
            const Eigen::Vector2d &px_ref,
            const int level_ref,
            const int level_cur,
            const int halfpatch_size,
            uint8_t *patch);

    } // namespace warp
} // namespace ORB_SLAM3
