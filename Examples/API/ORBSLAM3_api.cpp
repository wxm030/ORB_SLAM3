#include "ORBSLAM3_api.hpp"
#include <iostream>
#include <memory>
#include <thread>
#include <glog/logging.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <sstream>
#include <opencv2/core/core.hpp>

#include <System.h>
#include "Converter.h"
#include "ImuTypes.h"

void CreateSLAMSystem(void **slam_handle, const string &strVocFile, const string &strSettingsFile)
{
    google::InitGoogleLogging("ORBSLAM3API");

    //std::shared_ptr<ORB_SLAM3::System> system = std::make_shared<ORB_SLAM3::System>(strVocFile, strSettingsFile, sensor);
    ORB_SLAM3::System *handle = new ORB_SLAM3::System(strVocFile, strSettingsFile, ORB_SLAM3::System::IMU_MONOCULAR, true);
    *slam_handle = (void *)handle;
}

//for monoIMU
SLAMResult ProcessImageMonoIMU(void *slam_handle, ImageDATA &image_data, std::vector<IMU_DATA> &imu)
{
    SLAMResult result;

    std::vector<ORB_SLAM3::IMU::Point> vecImuMes;
    for (int i = 0; i < imu.size(); i++)
    {
        ORB_SLAM3::IMU::Point imu_one = ORB_SLAM3::IMU::Point(imu[i].acc[0], imu[i].acc[1], imu[i].acc[2],
                                                              imu[i].gyr[0], imu[i].gyr[1], imu[i].gyr[2],
                                                              imu[i].timestamp);
        vecImuMes.push_back(imu_one);
    }

    double tframe = image_data.timestamp;
    cv::Mat im(image_data.height, image_data.width, CV_8UC1);
    im.data = image_data.data;

    ORB_SLAM3::System *handle = (ORB_SLAM3::System *)slam_handle;
    cv::Mat Twc = handle->TrackMonocular(im, tframe, vecImuMes); // TODO change to monocular_inertial
    int status = handle->GetTrackingState();
    if (status == 2)
    {
        Eigen::Map<Eigen::Matrix4d> T_wc(result.T_wc);
        T_wc = ORB_SLAM3::Converter::toMatrix4d(Twc);
        result.slam_status = SLAMStatus::OK;
    }
    result.timestamp = tframe;

    return result;
}

void ReleaseSLAMSystem(void *slam_handle)
{
    ORB_SLAM3::System *handle = (ORB_SLAM3::System *)slam_handle;
    // Stop all threads
    handle->Shutdown();
    delete handle;
    google::ShutdownGoogleLogging();
}