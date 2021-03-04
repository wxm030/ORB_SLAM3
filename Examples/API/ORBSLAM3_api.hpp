

#ifndef ORBSLAM3_API_H
#define ORBSLAM3_API_H

#include <string>
#include <vector>
#include <array>

/**
 * @brief IMU data
 */
struct IMU_DATA
{
    double timestamp;
    std::array<double, 3> acc;
    std::array<double, 3> gyr;
};

/**
 * @brief image data
 */
struct ImageDATA
{
    double timestamp;
    int width;
    int height;
    int stride;
    u_char *data;
};

enum class SLAMStatus : int
{
    NOT_INITIALIZED = 1,
    OK = 2,
    RECENT_LOST = 3,
    LOST = 4
};

// Input sensor
enum class eSensor : int
{
    MONOCULAR = 0,
    STEREO = 1,
    RGBD = 2,
    IMU_MONOCULAR = 3,
    IMU_STEREO = 4
};

/**
 * @brief SLAM返回结果
 */
struct SLAMResult
{
    double timestamp;
    double T_wc[16];
    // double T_wi[16];
    SLAMStatus slam_status;
};

/**
 * @brief 创建SLAM系统
 * @param slam_handle 
 * @param yaml_config_file 
 */
void CreateSLAMSystem(void **slam_handle, const std::string &strVocFile, const std::string &strSettingsFile);

/**
 * @brief 计算每帧位姿并返回slam的结果
 * @param slam_handle 
 * @param image_data
 * @param imu_data
 */
SLAMResult ProcessImageMonoIMU(void *slam_handle, ImageDATA &image_data, std::vector<IMU_DATA> &imu);

/**
 * @brief 释放SLAM系统
 * @param slam_handle 
 */
void ReleaseSLAMSystem(void *slam_handle);

#endif //ORBSLAM3_API_H