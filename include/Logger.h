/**
 * @file   Logger.h
 * @brief  Logging output information.
 * @author wxm
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include "Converter.h"

namespace ORB_SLAM3
{
    /* -------------------------------------------------------------------------- */
    // Open files with name output_filename, and checks that it is valid
    static void OpenFile(const std::string &output_filename,
                         std::ofstream *output_file,
                         bool append_mode = false)
    {
        CHECK_NOTNULL(output_file);
        output_file->open(output_filename.c_str(),
                          append_mode ? std::ios_base::app : std::ios_base::out);
        output_file->precision(20);
        CHECK(output_file->is_open()) << "Cannot open file: " << output_filename;
        CHECK(output_file->good()) << "File in bad state: " << output_filename;
    }

    // Wrapper for std::ofstream to open/close it when created/destructed.
    class OfstreamWrapper
    {
    public:
        OfstreamWrapper(const std::string &filename,
                        const bool &open_file_in_append_mode = false);
        virtual ~OfstreamWrapper();
        void closeAndOpenLogFile();

    public:
        std::ofstream ofstream_;
        const std::string filename_;
        const std::string output_path_;
        const bool open_file_in_append_mode = false;

    protected:
        void openLogFile(const std::string &output_file_name,
                         bool open_file_in_append_mode = false);
    };

    /**
 * @brief Logs ground-truth info from Euroc dataset, just copy-paste
 * ground-truth csv file.
 */
    class EurocGtLogger
    {
    public:
        EurocGtLogger();
        virtual ~EurocGtLogger() = default;

        /**
   * @brief logGtData Simply copy-pastes the file given as input to the
   * file output_gt_poses.csv inside the output folder.
   * @param gt_file file where the data.csv file in Euroc is located.
   * Usually inside `mav0/state_groundtruth_estimate0/data.csv`.
   */
        void logGtData(const std::string &gt_file);

    private:
        // Filenames to be saved in the output folder.
        OfstreamWrapper output_gt_poses_csv_;
    };

    class TrackingLogger
    {
    public:
        TrackingLogger();
        virtual ~TrackingLogger() = default;

        void logTrackingPoseCSV(const std::int64_t &timestamp, const cv::Mat &pose);
        void logScaleValueCSV(const std::int64_t &timestamp, const double &scale);
        void logPreIntegratedIMUPoseCSV(const std::int64_t &timestamp, const cv::Mat &relativepose, const std::int64_t &last_timestamp);
        void logTrackReferenceFramePoseCSV(const double &timestamp, const cv::Mat &pose);
        void logTrackMotionModelPoseCSV(const double &timestamp, const cv::Mat &pose);
        void logTrackSparseImageCSV(const double &timestamp, const cv::Mat &pose);
        void logInitialIMUdataCSV(std::string &cam_or_im, Eigen::Vector3d &dR, Eigen::Vector3d &dV, Eigen::Vector3d &dP,
                                  float &scale, float &dt, Eigen::Vector3d &g);

    private:
        // Filenames to be saved in the output folder.
        OfstreamWrapper output_poses_vio_csv_; //tracking 输出的pose
        OfstreamWrapper output_scale_csv_;     //tracking 输出的pose
        OfstreamWrapper output_poses_pim_csv_; //imu预积分的相邻两帧之间的pose
        OfstreamWrapper output_poses_track_reference_frame_csv_;
        OfstreamWrapper output_poses_track_motion_model_csv_;
        OfstreamWrapper output_poses_track_sparse_img_align_csv_;
        OfstreamWrapper output_init_imu_data_csv_; //tracking 输出的pose

        double timing_loggerTracking_;
        bool is_header_written_scale_ = false;
        bool is_header_written_poses_vio_ = false;
        bool is_header_written_poses_pim_ = false;
        bool is_header_written_poses_track_reference_frame_ = false;
        bool is_header_written_poses_track_motion_model_ = false;
        bool is_header_written_poses_track_sparse_img_align_ = false;
        bool is_header_written_init_imu_data_ = false;
    };

} // namespace ORB_SLAM3

#endif