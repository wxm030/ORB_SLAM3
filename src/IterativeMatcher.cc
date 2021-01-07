
#include "IterativeMatcher.h"

using namespace ORB_SLAM3;

IterativeMatcher::IterativeMatcher(int n_level, int cell_size, int width, int height) : n_level_(n_level), mCell_size(cell_size), mWidth(width), mHeight(height)
{
    InitializeGrid();
}

IterativeMatcher::~IterativeMatcher()
{
}

void IterativeMatcher::InitializeGrid()
{
    grid_.cell_size = mCell_size;
    grid_.grid_n_cols = std::ceil(static_cast<double>(mWidth) / mCell_size);
    grid_.grid_n_rows = std::ceil(static_cast<double>(mHeight) / mCell_size);
    grid_.cells.resize(grid_.grid_n_cols * grid_.grid_n_rows);
    std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell *&c) { c = new Cell; });
    grid_.cell_order.resize(grid_.cells.size());
    for (size_t i = 0; i < grid_.cells.size(); ++i)
    {
        grid_.cell_order[i] = i;
    }
    std::random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end()); // maybe we should do it at every iteration!
}

void IterativeMatcher::ResetGrid()
{
    n_matches_ = 0;
    std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell *c) { c->clear(); });
}

int IterativeMatcher::ReprojectMap(Frame &frame, std::vector<MapPoint *> &local_points)
{
    ResetGrid();
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::cout << "local_points.size() ==== " << local_points.size() << std::endl;
    // Now project all point candidates
    //网格中保存的为　3D点->投影的2D点
    int cn = 0;
    for (auto iter = local_points.begin(); iter != local_points.end(); iter++)
    {
        MapPoint *pMP = *iter;
        if (pMP->isBad() || !pMP)
        {
            continue;
        }
        Eigen::Vector2d p2d;
        if (frame.isInFrame(pMP, p2d, 8))
        {
            const int k = static_cast<int>(p2d[1] / grid_.cell_size) * grid_.grid_n_cols + static_cast<int>(p2d[0] / grid_.cell_size);
            //每个cell中最多保存2个点
            if (grid_.cells.at(k)->size() < 1)
            {
                grid_.cells.at(k)->push_back(Candidate(pMP, p2d));
            }
            cn++;
        }
    }
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    for (size_t i = 0; i < grid_.cells.size(); ++i)
    {
        if (ReprojectCell(*grid_.cells.at(grid_.cell_order[i]), frame))
        {
            ++n_matches_;
        }
        if (n_matches_ > 150)
        {
            break;
        }
    }
    std::cout << "n_matches_==== " << n_matches_ << "," << cn << std::endl;
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    std::cout << "ReprojectMap  time    projectPoint cost ==  " << std::to_string(std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count()) << std::endl;
    std::cout << "ReprojectMap  time    ReprojectCell cost ==  " << std::to_string(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()) << std::endl;

    frame.N = frame.mvKeys.size();
    frame.mvuRight.resize(frame.N, -1);
    frame.UndistortKeyPoints();
    assert(frame.mvKeys.size() > 0);
    return n_matches_;
}

bool IterativeMatcher::GetCloseViewObs(MapPoint *pt, const Eigen::Vector3d &framepos, Feature &ref_ftr)
{
    cv::Mat p3d = pt->GetWorldPos();

    Eigen::Vector3d pos_3d = {p3d.at<float>(0), p3d.at<float>(1), p3d.at<float>(2)};
    std::map<KeyFrame *, std::tuple<int, int>> obs = pt->GetObservations();

    // TODO: get frame with same point of view AND same pyramid level!
    Eigen::Vector3d obs_dir(framepos - pos_3d);
    obs_dir.normalize();

    auto min_it = obs.begin();
    double min_cos_angle = 0;
    for (auto it = obs.begin(), ite = obs.end(); it != ite; ++it)
    {
        Eigen::Vector3d dir(it->first->pos() - pos_3d);
        dir.normalize();
        double cos_angle = obs_dir.dot(dir);
        if (cos_angle > min_cos_angle)
        {
            min_cos_angle = cos_angle;
            min_it = it;
        }
    }
    const int leftIndex = std::get<0>(min_it->second);
    if (min_cos_angle < 0.5 || leftIndex == -1) // assume that observations larger than 60° are useless
    {
        return false;
    }

    Eigen::Vector2d p2d;
    if (!min_it->first->isInFrame(pt, p2d, halfpatch_size_ + 2))
    {
        return false;
    }

    cv::Point2f p2d_cv = min_it->first->mvKeysUn[leftIndex].pt;
    Eigen::Vector2d p2d_eigen = {p2d_cv.x, p2d_cv.y};

    ref_ftr.level = min_it->first->mvKeysUn[leftIndex].octave;
    ref_ftr.f = pos_3d;
    ref_ftr.grad = {1, 0};
    ref_ftr.frame = min_it->first;
    ref_ftr.px = p2d_eigen;

    return true;
}

void IterativeMatcher::createPatchFromPatchWithBorder()
{
    uint8_t *ref_patch_ptr = patch_;
    for (int y = 1; y < patch_size_ + 1; ++y, ref_patch_ptr += patch_size_)
    {
        uint8_t *ref_patch_border_ptr = patch_with_border_ + y * (patch_size_ + 2) + 1;
        for (int x = 0; x < patch_size_; ++x)
            ref_patch_ptr[x] = ref_patch_border_ptr[x];
    }
}

//获取与当前帧夹角最小的关键帧进行匹配//
// bool IterativeMatcher::FindMatchDirect(MapPoint *pt, const Frame &cur_frame, Eigen::Vector2d &px_cur, int &level)
// {
//     ///获取当前2D投影点的光线方向，找到地图点的观测中的光线方向最小的点
//     std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

//     Feature ref_ftr;
//     if (!GetCloseViewObs(pt, cur_frame.pos(), ref_ftr))
//     {
//         std::cout << "GetCloseViewObs false 1111111111111111 == " << std::endl;
//         return false;
//     }
//     std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//     //std::cout << "Time cost GetCloseViewObs  =  " << std::to_string(std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count()) << std::endl;

//     level = ref_ftr.level;
//     // warp affine
//     Eigen::Matrix4d T1 = Converter::toMatrix4d(cur_frame.mTcw);
//     Eigen::Matrix4d T2 = Converter::toMatrix4d(ref_ftr.frame->GetPose()).inverse();
//     warp::getWarpMatrixAffine(
//         ref_ftr.frame->mpCamera, cur_frame.mpCamera, ref_ftr.px, ref_ftr.f,
//         Converter::toSE3(T1 * T2), ref_ftr.level, A_cur_ref_);
//     search_level_ = warp::getBestSearchLevel(A_cur_ref_, n_level_ - 1); //todo  Config::nPyrLevels()-1  3 - 1
//     // search_level_ = level;
//     warp::warpAffine(A_cur_ref_, ref_ftr.frame->mvImagePyramid[ref_ftr.level], ref_ftr.px,
//                      ref_ftr.level, search_level_, halfpatch_size_ + 1, patch_with_border_);
//     createPatchFromPatchWithBorder();

//     //std::cout << "search_level_   1111111111111111 == " << search_level_ << "," << level << "," << ref_ftr.frame->mnFrameId << std::endl;
//     // px_cur should be set
//     Eigen::Vector2d px_scaled(px_cur / (1 << search_level_));

//     std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//     //std::cout << "Time cost getWarpMatrixAffine  =  " << std::to_string(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()) << std::endl;

//     bool success = false;
//     success = feature_alignment::align2D(
//         cur_frame.mvImagePyramid[search_level_], patch_with_border_, patch_,
//         10, px_scaled);

//     std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
//     //std::cout << "Time cost align2D  =  " << std::to_string(std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()) << std::endl;

//     // std::cout << "feature_alignment::align2D   success " << success << std::endl;
//     px_cur = px_scaled * (1 << search_level_);
//     return success;
// }

//获取每个地图点所有的关键帧与当前帧进行匹配
bool IterativeMatcher::FindMatchDirect(MapPoint *pt, const Frame &cur_frame, Eigen::Vector2d &px_cur, int &level)
{
    std::map<KeyFrame *, std::tuple<int, int>> obs = pt->GetObservations();
    vector<std::pair<KeyFrame *, std::tuple<int, int>>> sorted_obs;
    for (auto &o : obs)
    {
        if (!o.first->isBad())
            sorted_obs.push_back(make_pair(o.first, o.second));
    }
    // 按照id排序
    // 然而这里选最近的点会容易导致飘移
    sort(sorted_obs.begin(), sorted_obs.end(),
         [](const pair<KeyFrame *, std::tuple<int, int>> &p1, const pair<KeyFrame *, std::tuple<int, int>> &p2) {
             return p1.first->mnId > p2.first->mnId;
         });
    //如果观测大于５个，则只保留最近的５个
    if ((int)sorted_obs.size() > 5)
    {
        sorted_obs = vector<std::pair<KeyFrame *, std::tuple<int, int>>>(sorted_obs.begin(), sorted_obs.begin() + 5);
    }

    std::vector<Eigen::Vector2d> matched_pixels;
    cv::Mat p3d = pt->GetWorldPos();
    for (auto &o : sorted_obs)
    {
        Feature ref_ftr;
        const int leftIndex = std::get<0>(o.second);
        ref_ftr.level = o.first->mvKeysUn[leftIndex].octave;

        cv::Mat Rcw = o.first->GetPose().rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = o.first->GetPose().rowRange(0, 3).col(3);
        cv::Mat p3d_ref = Rcw * p3d + tcw;

        ref_ftr.f = {p3d_ref.at<float>(0), p3d_ref.at<float>(1), p3d_ref.at<float>(2)}; //参考帧坐标系下的点
        ref_ftr.grad = {1, 0};
        ref_ftr.frame = o.first;
        cv::Point2f p2d_cv = o.first->mvKeysUn[leftIndex].pt;
        Eigen::Vector2d p2d_eigen = {p2d_cv.x, p2d_cv.y};
        ref_ftr.px = p2d_eigen;

        level = ref_ftr.level;
        // warp affine
        Eigen::Matrix4d T1 = Converter::toMatrix4d(cur_frame.mTcw);
        Eigen::Matrix4d T2 = Converter::toMatrix4d(ref_ftr.frame->GetPose()).inverse();
        warp::getWarpMatrixAffine(
            ref_ftr.frame->mpCamera, cur_frame.mpCamera, ref_ftr.px, ref_ftr.f,
            Converter::toSE3(T1 * T2), ref_ftr.level, A_cur_ref_);
        search_level_ = warp::getBestSearchLevel(A_cur_ref_, n_level_ - 1); //todo  Config::nPyrLevels()-1  3 - 1
        //search_level_ = level;
        warp::warpAffine(A_cur_ref_, ref_ftr.frame->mvImagePyramid[ref_ftr.level], ref_ftr.px,
                         ref_ftr.level, search_level_, halfpatch_size_ + 1, patch_with_border_);
        createPatchFromPatchWithBorder();

        // std::cout << "search_level_   1111111111111111 == " << search_level_ << "," << level << "," << ref_ftr.frame->mnFrameId << std::endl;
        // px_cur should be set
        Eigen::Vector2d px_scaled(px_cur / (1 << search_level_));

        bool success = false;
        success = feature_alignment::align2D(
            cur_frame.mvImagePyramid[search_level_], patch_with_border_, patch_,
            10, px_scaled);
        if (success)
        {
            matched_pixels.push_back(px_scaled * (1 << search_level_));
        }
    }
    if (!matched_pixels.empty())
    {
        // 有成功追踪到的点，取平均像素位置为测量
        Eigen::Vector2d px_ave(0, 0);
        for (Eigen::Vector2d &p : matched_pixels)
            px_ave += p;
        px_ave = px_ave / matched_pixels.size();

        px_cur = px_ave;
        std::cout << "match successs222222" << std::endl;
        return true;
    }
    else
    {
        std::cout << "match failed11111111" << std::endl;
        return false;
    }
}

bool IterativeMatcher::ReprojectCell(Cell &cell, Frame &frame)
{
    Cell::iterator it = cell.begin();
    while (it != cell.end())
    {
        // std::cout << "before cur_2d == " << it->px << std::endl;
        int level = 0;
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        bool found_match = FindMatchDirect(it->pt, frame, it->px, level);
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        //std::cout << "Time cost FindMatchDirect  =  " << std::to_string(std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count()) << std::endl;
        if (!found_match)
        {
            it = cell.erase(it);
            continue;
        }

        // 丢弃位于太边缘的地方的点，否则在创建关键帧，计算描述子时可能导致溢出
        if (it->px[0] > 20 && it->px[1] > 20 && it->px[0] < frame.mvImagePyramid[0].cols - 20 && it->px[1] < frame.mvImagePyramid[0].rows - 20)
        {
            // insert a feature and assign it to a map point
            frame.mvKeys.push_back(cv::KeyPoint(cv::Point2f(it->px[0], it->px[1]), 7, -1, 0, search_level_)); //与地图点匹配到的点
            frame.mvpMapPoints.push_back(it->pt);
            frame.mvDepth.push_back(-1);
            frame.mvbOutlier.push_back(false);
        }

        // If the keyframe is selected and we reproject the rest, we don't have to
        // check this point anymore.
        it = cell.erase(it);

        // Maximum one point per cell.
        return true;
    }
    return false;
}

float interpolateMat_8u(const cv::Mat &mat, float u, float v)
{
    assert(mat.type() == CV_8U);
    int x = std::floor(u);
    int y = std::floor(v);
    float subpix_x = u - x;
    float subpix_y = v - y;

    float w00 = (1.0f - subpix_x) * (1.0f - subpix_y);
    float w01 = (1.0f - subpix_x) * subpix_y;
    float w10 = subpix_x * (1.0f - subpix_y);
    float w11 = 1.0f - w00 - w01 - w10;

    const int stride = mat.step.p[0];
    unsigned char *ptr = mat.data + y * stride + x;
    return w00 * ptr[0] + w01 * ptr[stride] + w10 * ptr[1] + w11 * ptr[stride + 1];
}

namespace ORB_SLAM3
{
    namespace warp
    {
        void getWarpMatrixAffine(
            GeometricCamera *cam_ref,
            GeometricCamera *cam_cur,
            const Eigen::Vector2d &px_ref,
            const Eigen::Vector3d &f_ref,
            const Sophus::SE3 &T_cur_ref,
            const int level_ref,
            Eigen::Matrix2d &A_cur_ref)
        {
            // Compute affine warp matrix A_ref_cur
            const int halfpatch_size = 5;
            const Eigen::Vector3d xyz_ref(f_ref);

            Eigen::Vector2d p2d_eigen_du = px_ref + Eigen::Vector2d(halfpatch_size, 0) * (1 << level_ref);
            cv::Point2f p2D_du(p2d_eigen_du(0), p2d_eigen_du(1));
            cv::Point3f p3D_du = cam_ref->unproject(p2D_du);

            Eigen::Vector3d xyz_du_ref = {p3D_du.x, p3D_du.y, p3D_du.z};
            Eigen::Vector2d p2d_eigen_dv = px_ref + Eigen::Vector2d(0, halfpatch_size) * (1 << level_ref);
            cv::Point2f p2D_dv(p2d_eigen_dv(0), p2d_eigen_dv(1));
            cv::Point3f p3D_dv = cam_ref->unproject(p2D_dv);
            Eigen::Vector3d xyz_dv_ref = {p3D_dv.x, p3D_dv.y, p3D_dv.z};

            xyz_du_ref *= xyz_ref[2] / xyz_du_ref[2];
            xyz_dv_ref *= xyz_ref[2] / xyz_dv_ref[2];

            Eigen::Vector3d p3d_cur_eigen = T_cur_ref * (xyz_ref);
            cv::Point3f p3d_cur(p3d_cur_eigen(0), p3d_cur_eigen(1), p3d_cur_eigen(2));
            cv::Point2f p2d_cur = cam_cur->project(p3d_cur);
            const Eigen::Vector2d px_cur = {p2d_cur.x, p2d_cur.y};

            Eigen::Vector3d p3d_du_eigen = T_cur_ref * (xyz_du_ref);
            cv::Point3f p3d_px_du(p3d_du_eigen(0), p3d_du_eigen(1), p3d_du_eigen(2));
            cv::Point2f p2d_px_du = cam_cur->project(p3d_px_du);
            const Eigen::Vector2d px_du = {p2d_px_du.x, p2d_px_du.y};

            Eigen::Vector3d p3d_dv_eigen = T_cur_ref * (xyz_dv_ref);
            cv::Point3f p3d_px_dv(p3d_dv_eigen(0), p3d_dv_eigen(1), p3d_dv_eigen(2));
            cv::Point2f p2d_px_dv = cam_cur->project(p3d_px_dv);
            const Eigen::Vector2d px_dv = {p2d_px_dv.x, p2d_px_dv.y};

            A_cur_ref.col(0) = (px_du - px_cur) / halfpatch_size;
            A_cur_ref.col(1) = (px_dv - px_cur) / halfpatch_size;
        }

        int getBestSearchLevel(
            Eigen::Matrix2d &A_cur_ref,
            const int max_level)
        {
            // Compute patch level in other image
            int search_level = 0;
            double D = A_cur_ref.determinant();
            while (D > 3.0 && search_level < max_level)
            {
                search_level += 1;
                D *= 0.25;
            }
            return search_level;
        }

        void warpAffine(
            Eigen::Matrix2d &A_cur_ref,
            const cv::Mat &img_ref,
            const Eigen::Vector2d &px_ref,
            const int level_ref,
            const int search_level,
            const int halfpatch_size,
            uint8_t *patch)
        {
            const int patch_size = halfpatch_size * 2;
            Eigen::Matrix2d A_ref_cur = A_cur_ref.inverse();
            if (std::isnan(A_ref_cur(0, 0)))
            {
                printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
                return;
            }

            // Perform the warp on a larger patch.
            uint8_t *patch_ptr = patch;
            Eigen::Vector2d px_ref_pyr = px_ref / (1 << level_ref);
            for (int y = 0; y < patch_size; ++y)
            {
                for (int x = 0; x < patch_size; ++x, ++patch_ptr)
                {
                    Eigen::Vector2d px_patch = {x - halfpatch_size, y - halfpatch_size};
                    px_patch *= (1 << search_level);
                    Eigen::Vector2d px = A_ref_cur * px_patch + px_ref_pyr;
                    if (px[0] < 0 || px[1] < 0 || px[0] >= img_ref.cols - 1 || px[1] >= img_ref.rows - 1)
                        *patch_ptr = 0;
                    else
                        *patch_ptr = (uint8_t)interpolateMat_8u(img_ref, px[0], px[1]);
                }
            }
        }

    } // namespace warp

} // namespace ORB_SLAM3
