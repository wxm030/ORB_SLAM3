#include "test_api.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <System.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

void LoadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps, double timeshift, double start_t, double end_t);

void LoadIMU(const std::string &strImuPath, std::vector<double> &vTimeStamps, std::vector<IMU_DATA> &vImus);

//
std::vector<std::string> vstrImageFilenames;
std::vector<double> vTimestampsCam;
std::vector<IMU_DATA> vIMuAll;
std::vector<double> vTimestampsImu;
int first_imu = 0;

TestApi::TestApi(const std::string &yaml_config_fname)
{
    Run(yaml_config_fname);
}

TestApi::~TestApi()
{
    Stop();
}

void TestApi::Run(const std::string &yaml_config_fname)
{
    std::unique_lock<std::mutex> lock(stop_mutex_);
    stop_ = false;
    lock.unlock();
    handle_ = nullptr;

    // load config params from yaml
    cv::FileStorage fs;
    if (!fs.open(yaml_config_fname, cv::FileStorage::READ))
    {
        std::cout << "cannot open config file: " << yaml_config_fname << std::endl;
    }

    double timeshift = fs["Timeshift"]; //t_imu = t_cam + timeshift
    std::string strVocFile = fs["VocFilePath"];
    std::string strSettingsFile = fs["SettingFilePath"];
    double start_t = fs["start_t"]; //s
    double end_t = fs["end_t"];     //s
    std::string pathSeq = fs["DataSetPath"];
    std::string pathTimeStamps = fs["DataTimestampTxtFile"];
    std::string pathCam0 = pathSeq + "/mav0/cam0/data";
    std::string pathImu = pathSeq + "/mav0/imu0/data.csv";

    LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames, vTimestampsCam, timeshift, start_t, end_t);
    std::cout << "Image LOADED!" << std::endl;
    LoadIMU(pathImu, vTimestampsImu, vIMuAll);
    std::cout << "IMU LOADED!" << std::endl;

    // Find first imu to be considered, supposing imu measurements start first
    while (vTimestampsImu[first_imu] <= vTimestampsCam[0])
        first_imu++;
    first_imu--; // first imu measurement to be considered

    CreateSLAMSystem(&handle_, strVocFile, strSettingsFile);
    data_process_thread_ = std::thread(&TestApi::ProcessMonoImu, this, handle_);
}

void TestApi::Stop()
{
    data_process_thread_.join();
    std::unique_lock<std::mutex> lock(stop_mutex_);
    stop_ = true;
    lock.unlock();
    ReleaseSLAMSystem(handle_);
}

void TestApi::ProcessMonoImu(void *slam_handle)
{
    std::vector<IMU_DATA> vImuMeas;
    ImageDATA imgdata;
    for (int ni = 0; ni < vstrImageFilenames.size(); ni++)
    {
        // Read image from file
        cv::Mat im = cv::imread(vstrImageFilenames[ni], 0); //gray image

        double tframe = vTimestampsCam[ni];
        imgdata.height = im.rows;
        imgdata.width = im.cols;
        imgdata.stride = im.cols;
        imgdata.data = im.data;
        imgdata.timestamp = tframe;

        if (im.empty())
        {
            std::cerr << std::endl
                      << "Failed to load image at: "
                      << vstrImageFilenames[ni] << std::endl;
            return;
        }

        // Load imu measurements from previous frame
        vImuMeas.clear();

        if (ni > 0)
        {
            std::cout << "first_imu==  " << first_imu << "," << vTimestampsImu.size() << std::endl;
            while (vTimestampsImu[first_imu] <= vTimestampsCam[ni] && first_imu < vTimestampsImu.size())
            {
                vImuMeas.push_back(vIMuAll[first_imu]);
                first_imu++;
            }
        }

        std::cout << "vImuMeas.size() == " << vImuMeas.size() << std::endl;
        SLAMResult result = ProcessImageMonoIMU(slam_handle, imgdata, vImuMeas); //imu and camra data  is sync
    }
}

void LoadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps, double timeshift, double start_t, double end_t)
{
    std::ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    //get the first timestamp
    std::string t0_str;
    std::getline(fTimes, t0_str);
    double time0 = stod(t0_str);
    double t_start = time0 / 1e9 + timeshift + start_t;
    double t_end = time0 / 1e9 + timeshift + end_t;
    while (!fTimes.eof())
    {
        std::string s;
        std::getline(fTimes, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            double t_cam_after_shift = t / 1e9 + timeshift; //s
            if (t_cam_after_shift > t_start && t_cam_after_shift < t_end)
            {
                vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
                vTimeStamps.push_back(t_cam_after_shift);
            }
        }
    }
}

void LoadIMU(const std::string &strImuPath, std::vector<double> &vTimeStamps, std::vector<IMU_DATA> &vImus)
{
    std::ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vImus.reserve(5000);

    while (!fImu.eof())
    {
        std::string s;
        std::getline(fImu, s);
        if (s[0] == '#')
            continue;

        if (!s.empty())
        {
            std::string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != std::string::npos)
            {
                item = s.substr(0, pos);
                data[count++] = std::stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = std::stod(item);

            vTimeStamps.push_back(data[0] / 1e9);
            IMU_DATA imu_one;
            imu_one.timestamp = vTimestampsImu[first_imu];
            imu_one.acc = {data[4], data[5], data[6]};
            imu_one.gyr = {data[1], data[2], data[3]};
            vImus.push_back(imu_one);
        }
    }
}
