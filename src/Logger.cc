
#include "Logger.h"

#include <fstream>
#include <memory>
#include <string>

#include <boost/filesystem.hpp> // to create folders
#include <boost/foreach.hpp>

#include <gflags/gflags.h>

DEFINE_string(output_path, "/home/wxm/Documents/code/ORB_SLAM3/data/output_logs/", "Path where to store VIO's log output.");

namespace ORB_SLAM3
{
    /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
    // This constructor will directly open the log file when called.
    OfstreamWrapper::OfstreamWrapper(const std::string &filename,
                                     const bool &open_file_in_append_mode)
        : filename_(filename), output_path_(FLAGS_output_path)
    {
        openLogFile(filename);
    }

    // This destructor will directly close the log file when the wrapper is
    // destructed. So no need to explicitly call .close();
    OfstreamWrapper::~OfstreamWrapper()
    {
        LOG(INFO) << "Closing output file: " << filename_.c_str();
        ofstream_.close();
    }

    void OfstreamWrapper::closeAndOpenLogFile()
    {
        ofstream_.close();
        openLogFile(filename_);
    }

    void OfstreamWrapper::openLogFile(const std::string &output_file_name,
                                      bool open_file_in_append_mode)
    {
        CHECK(!output_file_name.empty());
        LOG(INFO) << "Opening output file: " << output_file_name.c_str();
        OpenFile(output_path_ + '/' + output_file_name,
                 &ofstream_,
                 open_file_in_append_mode);
    }

    /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
    EurocGtLogger::EurocGtLogger() : output_gt_poses_csv_("traj_gt.csv") {}

    void EurocGtLogger::logGtData(const std::string &file_path)
    {
        std::ifstream f_in(file_path.c_str());
        CHECK(f_in.is_open()) << "Cannot open file: " << file_path;
        // Drop first line, we want to use our own header.
        std::string dummy_header;
        std::getline(f_in, dummy_header);

        std::ofstream &output_stream = output_gt_poses_csv_.ofstream_;
        // First, write header
        output_stream << "#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz,"
                      << "bgx,bgy,bgz,bax,bay,baz" << std::endl;
        // Then, copy all gt data to file
        output_stream << f_in.rdbuf();

        // Clean
        f_in.close();
    }

    /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
    TrackingLogger::TrackingLogger()
        : output_poses_vio_csv_("poses_vio.csv"),
          output_scale_csv_("scale.csv"),
          output_poses_pim_csv_("poses_pim.csv"),
          output_poses_track_reference_frame_csv_("pose_track_reference_frame.csv"),
          output_poses_track_motion_model_csv_("poses_track_motion_model.csv"),
          output_poses_track_sparse_img_align_csv_("poses_track_sparse_img_align.csv"),
          output_init_imu_data_csv_("init_imu_data.csv") {}

    void TrackingLogger::logTrackingPoseCSV(const std::int64_t &timestamp, const cv::Mat &pose, const cv::Mat &velocity)
    {
        // We log the poses in csv format for later alignement and analysis.
        std::ofstream &output_stream = output_poses_vio_csv_.ofstream_;
        bool &is_header_written = is_header_written_poses_vio_;
        // First, write header, but only once.
        if (!is_header_written)
        {
            output_stream << "#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz" << std::endl;
            is_header_written = true;
        }
        if (!pose.empty() && !velocity.empty())
        {
            cv::Mat Rwc = pose.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * pose.rowRange(0, 3).col(3);
            std::vector<float> q = Converter::toQuaternion(Rwc);
            output_stream << timestamp << ","             //
                          << twc.at<float>(0) << ","      //
                          << twc.at<float>(1) << ","      //
                          << twc.at<float>(2) << ","      //
                          << q[0] << ","                  // q_w
                          << q[1] << ","                  // q_x
                          << q[2] << ","                  // q_y
                          << q[3] << ","                  // q_z
                          << velocity.at<float>(0) << "," //vx
                          << velocity.at<float>(1) << "," //vy
                          << velocity.at<float>(2)        //vz
                          << std::endl;
        }
    }

    void TrackingLogger::logInitialIMUdataCSV(std::string cam_or_im, Eigen::Vector3d dR, Eigen::Vector3d dV, Eigen::Vector3d dP,
                                              float scale, float dt, Eigen::Vector3d g)
    {
        // We log the poses in csv format for later alignement and analysis.
        std::ofstream &output_stream = output_init_imu_data_csv_.ofstream_;
        bool &is_header_written = is_header_written_init_imu_data_;
        // First, write header, but only once.
        if (!is_header_written)
        {
            output_stream << "#timestamp,dR_x,dR_y,dR_z,dV_x, dV_y, dV_z,dP_x, dP_y, dP_z, scale,dt,g_x,g_y,g_z" << std::endl;
            is_header_written = true;
        }
        output_stream << std::endl
                      << cam_or_im << ","
                      << dR[0] << "," << dR[1] << "," << dR[2] << ","
                      << dV[0] << "," << dV[1] << "," << dV[2] << ","
                      << dP[0] << "," << dP[1] << "," << dP[2] << ","
                      << scale << ","
                      << dt << ","
                      << g[0] << "," << g[1] << "," << g[2] << ","
                      << std::endl;
    }

    void TrackingLogger::logScaleValueCSV(const std::int64_t &timestamp, const double &scale)
    {
        // We log the poses in csv format for later alignement and analysis.
        std::ofstream &output_stream = output_scale_csv_.ofstream_;
        bool &is_header_written = is_header_written_scale_;
        // First, write header, but only once.
        if (!is_header_written)
        {
            output_stream << "#timestamp,scale" << std::endl;
            is_header_written = true;
        }

        output_stream << timestamp << ","
                      << scale
                      << std::endl;
    }

    void TrackingLogger::logPreIntegratedIMUPoseCSV(const std::int64_t &timestamp, const cv::Mat &relativepose, const std::int64_t &last_timestamp)
    {
        // We log the poses in csv format for later alignement and analysis.
        std::ofstream &output_stream = output_poses_pim_csv_.ofstream_;
        bool &is_header_written = is_header_written_poses_pim_;
        // First, write header, but only once.
        if (!is_header_written)
        {
            output_stream << "#timestamp,x,y,z,qw,qx,qy,qz,last_timestamp" << std::endl;
            is_header_written = true;
        }
        if (!relativepose.empty())
        {
            cv::Mat Rwc = relativepose.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * relativepose.rowRange(0, 3).col(3);
            std::vector<float> q = Converter::toQuaternion(Rwc);
            output_stream << timestamp << ","        //
                          << twc.at<float>(0) << "," //
                          << twc.at<float>(1) << "," //
                          << twc.at<float>(2) << "," //
                          << q[0] << ","             // q_w
                          << q[1] << ","             // q_x
                          << q[2] << ","             // q_y
                          << q[3] << ","             // q_z
                          << last_timestamp
                          << std::endl;
        }
    }

    void TrackingLogger::logTrackReferenceFramePoseCSV(const double &timestamp, const cv::Mat &pose)
    {
        // We log the poses in csv format for later alignement and analysis.
        std::ofstream &output_stream = output_poses_track_reference_frame_csv_.ofstream_;
        bool &is_header_written = is_header_written_poses_track_reference_frame_;
        // First, write header, but only once.
        if (!is_header_written)
        {
            output_stream << "#timestamp,x,y,z,qw,qx,qy,qz" << std::endl;
            is_header_written = true;
        }
        if (!pose.empty())
        {
            cv::Mat Rwc = pose.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * pose.rowRange(0, 3).col(3);
            std::vector<float> q = Converter::toQuaternion(Rwc);
            output_stream << timestamp << ","        //
                          << twc.at<float>(0) << "," //
                          << twc.at<float>(1) << "," //
                          << twc.at<float>(2) << "," //
                          << q[0] << ","             // q_w
                          << q[1] << ","             // q_x
                          << q[2] << ","             // q_y
                          << q[3]                    // q_z
                          << std::endl;
        }
    }

    void TrackingLogger::logTrackMotionModelPoseCSV(const double &timestamp, const cv::Mat &pose)
    {
        // We log the poses in csv format for later alignement and analysis.
        std::ofstream &output_stream = output_poses_track_motion_model_csv_.ofstream_;
        bool &is_header_written = is_header_written_poses_track_motion_model_;
        // First, write header, but only once.
        if (!is_header_written)
        {
            output_stream << "#timestamp,x,y,z,qw,qx,qy,qz" << std::endl;
            is_header_written = true;
        }
        if (!pose.empty())
        {
            cv::Mat Rwc = pose.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * pose.rowRange(0, 3).col(3);
            std::vector<float> q = Converter::toQuaternion(Rwc);
            output_stream << timestamp << ","        //
                          << twc.at<float>(0) << "," //
                          << twc.at<float>(1) << "," //
                          << twc.at<float>(2) << "," //
                          << q[0] << ","             // q_w
                          << q[1] << ","             // q_x
                          << q[2] << ","             // q_y
                          << q[3]                    // q_z
                          << std::endl;
        }
    }

    void TrackingLogger::logTrackSparseImageCSV(const double &timestamp, const cv::Mat &pose)
    {
        // We log the poses in csv format for later alignement and analysis.
        std::ofstream &output_stream = output_poses_track_sparse_img_align_csv_.ofstream_;
        bool &is_header_written = is_header_written_poses_track_sparse_img_align_;
        // First, write header, but only once.
        if (!is_header_written)
        {
            output_stream << "#timestamp,x,y,z,qw,qx,qy,qz" << std::endl;
            is_header_written = true;
        }
        if (!pose.empty())
        {
            cv::Mat Rwc = pose.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * pose.rowRange(0, 3).col(3);
            std::vector<float> q = Converter::toQuaternion(Rwc);
            output_stream << timestamp << ","        //
                          << twc.at<float>(0) << "," //
                          << twc.at<float>(1) << "," //
                          << twc.at<float>(2) << "," //
                          << q[0] << ","             // q_w
                          << q[1] << ","             // q_x
                          << q[2] << ","             // q_y
                          << q[3]                    // q_z
                          << std::endl;
        }
    }

    /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
} // namespace ORB_SLAM3