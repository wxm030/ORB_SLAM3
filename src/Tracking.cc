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

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Initializer.h"
#include "G2oTypes.h"
#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>

#include <mutex>
#include <chrono>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>
#include <include/MLPnPsolver.h>

using namespace std;

namespace ORB_SLAM3
{

    Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor, const string &_nameSeq) : mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
                                                                                                                                                                                                                              mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
                                                                                                                                                                                                                              mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys), mpViewer(NULL),
                                                                                                                                                                                                                              mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
                                                                                                                                                                                                                              mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr)
    {
        // Load camera parameters from settings file
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool b_parse_cam = ParseCamParamFile(fSettings);
        if (!b_parse_cam)
        {
            std::cout << "*Error with the camera parameters in the config file*" << std::endl;
        }

        // Load ORB parameters
        bool b_parse_orb = ParseORBParamFile(fSettings);
        if (!b_parse_orb)
        {
            std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
        }

        initID = 0;
        lastID = 0;

        // Load IMU parameters
        bool b_parse_imu = true;
        if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO)
        {
            b_parse_imu = ParseIMUParamFile(fSettings);
            if (!b_parse_imu)
            {
                std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
            }

            mnFramesToResetIMU = mMaxFrames;
        }

        mbInitWith3KFs = false;

        //Load other config parameters
        //默认采用原版的orb描述子匹配方法
        if (!fSettings["UseDirectSVO"].empty() && fSettings["UseDirectSVO"].isReal())
        {
            mUseDirectSVO = fSettings["UseDirectSVO"].real();
        }
        else
        {
            mUseDirectSVO = false;
        }

        //Rectification parameters
        /*mbNeedRectify = false;
    if((mSensor == System::STEREO || mSensor == System::IMU_STEREO) && sCameraName == "PinHole")
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fSettings["LEFT.K"] >> K_l;
        fSettings["RIGHT.K"] >> K_r;

        fSettings["LEFT.P"] >> P_l;
        fSettings["RIGHT.P"] >> P_r;

        fSettings["LEFT.R"] >> R_l;
        fSettings["RIGHT.R"] >> R_r;

        fSettings["LEFT.D"] >> D_l;
        fSettings["RIGHT.D"] >> D_r;

        int rows_l = fSettings["LEFT.height"];
        int cols_l = fSettings["LEFT.width"];
        int rows_r = fSettings["RIGHT.height"];
        int cols_r = fSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty()
                || rows_l==0 || cols_l==0 || rows_r==0 || cols_r==0)
        {
            mbNeedRectify = false;
        }
        else
        {
            mbNeedRectify = true;
            // M1r y M2r son los outputs (igual para l)
            // M1r y M2r son las matrices relativas al mapeo correspondiente a la rectificación de mapa en el eje X e Y respectivamente
            //cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
            //cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
        }


    }
    else
    {
        int cols = 752;
        int rows = 480;
        cv::Mat R_l = cv::Mat::eye(3, 3, CV_32F);
    }*/

        mnNumDataset = 0;

        if (!b_parse_cam || !b_parse_orb || !b_parse_imu)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch (exception &e)
            {
            }
        }

        //f_track_stats.open("tracking_stats"+ _nameSeq + ".txt");
        /*f_track_stats.open("tracking_stats.txt");
    f_track_stats << "# timestamp, Num KF local, Num MP local, time" << endl;
    f_track_stats << fixed ;*/

#ifdef SAVE_TIMES
        f_track_times.open("tracking_times.txt");
        f_track_times << "# ORB_Ext(ms), Stereo matching(ms), Preintegrate_IMU(ms), Pose pred(ms), LocalMap_track(ms), NewKF_dec(ms)" << endl;
        f_track_times << fixed;
#endif
    }

    Tracking::~Tracking()
    {
        //f_track_stats.close();
#ifdef SAVE_TIMES
        f_track_times.close();
#endif
    }

    bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
    {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
        cout << endl
             << "Camera Parameters: " << endl;
        bool b_miss_params = false;

        string sCameraName = fSettings["Camera.type"];
        if (sCameraName == "PinHole")
        {
            float fx, fy, cx, cy;

            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera.fx"];
            if (!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.fy"];
            if (!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cx"];
            if (!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cy"];
            if (!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera.k1"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(0) = node.real();
            }
            else
            {
                std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k2"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(1) = node.real();
            }
            else
            {
                std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.p1"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(2) = node.real();
            }
            else
            {
                std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.p2"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(3) = node.real();
            }
            else
            {
                std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k3"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.resize(5);
                mDistCoef.at<float>(4) = node.real();
            }

            if (b_miss_params)
            {
                return false;
            }

            vector<float> vCamCalib{fx, fy, cx, cy};

            mpCamera = new Pinhole(vCamCalib);

            mpAtlas->AddCamera(mpCamera);

            std::cout << "- Camera: Pinhole" << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
            std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;

            std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
            std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

            if (mDistCoef.rows == 5)
                std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

            mK = cv::Mat::eye(3, 3, CV_32F);
            mK.at<float>(0, 0) = fx;
            mK.at<float>(1, 1) = fy;
            mK.at<float>(0, 2) = cx;
            mK.at<float>(1, 2) = cy;
        }
        else if (sCameraName == "KannalaBrandt8")
        {
            float fx, fy, cx, cy;
            float k1, k2, k3, k4;

            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera.fx"];
            if (!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera.fy"];
            if (!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cx"];
            if (!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cy"];
            if (!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera.k1"];
            if (!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera.k2"];
            if (!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k3"];
            if (!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k4"];
            if (!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            if (!b_miss_params)
            {
                vector<float> vCamCalib{fx, fy, cx, cy, k1, k2, k3, k4};
                mpCamera = new KannalaBrandt8(vCamCalib);

                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                mK = cv::Mat::eye(3, 3, CV_32F);
                mK.at<float>(0, 0) = fx;
                mK.at<float>(1, 1) = fy;
                mK.at<float>(0, 2) = cx;
                mK.at<float>(1, 2) = cy;
            }

            if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
            {
                // Right camera
                // Camera calibration parameters
                cv::FileNode node = fSettings["Camera2.fx"];
                if (!node.empty() && node.isReal())
                {
                    fx = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }
                node = fSettings["Camera2.fy"];
                if (!node.empty() && node.isReal())
                {
                    fy = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.cx"];
                if (!node.empty() && node.isReal())
                {
                    cx = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.cy"];
                if (!node.empty() && node.isReal())
                {
                    cy = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                // Distortion parameters
                node = fSettings["Camera2.k1"];
                if (!node.empty() && node.isReal())
                {
                    k1 = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }
                node = fSettings["Camera2.k2"];
                if (!node.empty() && node.isReal())
                {
                    k2 = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.k3"];
                if (!node.empty() && node.isReal())
                {
                    k3 = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.k4"];
                if (!node.empty() && node.isReal())
                {
                    k4 = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                int leftLappingBegin = -1;
                int leftLappingEnd = -1;

                int rightLappingBegin = -1;
                int rightLappingEnd = -1;

                node = fSettings["Camera.lappingBegin"];
                if (!node.empty() && node.isInt())
                {
                    leftLappingBegin = node.operator int();
                }
                else
                {
                    std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
                }
                node = fSettings["Camera.lappingEnd"];
                if (!node.empty() && node.isInt())
                {
                    leftLappingEnd = node.operator int();
                }
                else
                {
                    std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
                }
                node = fSettings["Camera2.lappingBegin"];
                if (!node.empty() && node.isInt())
                {
                    rightLappingBegin = node.operator int();
                }
                else
                {
                    std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
                }
                node = fSettings["Camera2.lappingEnd"];
                if (!node.empty() && node.isInt())
                {
                    rightLappingEnd = node.operator int();
                }
                else
                {
                    std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
                }

                node = fSettings["Tlr"];
                if (!node.empty())
                {
                    mTlr = node.mat();
                    if (mTlr.rows != 3 || mTlr.cols != 4)
                    {
                        std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                        b_miss_params = true;
                    }
                }
                else
                {
                    std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                    b_miss_params = true;
                }

                if (!b_miss_params)
                {
                    static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                    static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                    mpFrameDrawer->both = true;

                    vector<float> vCamCalib2{fx, fy, cx, cy, k1, k2, k3, k4};
                    mpCamera2 = new KannalaBrandt8(vCamCalib2);

                    static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                    static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                    std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                    std::cout << std::endl
                              << "Camera2 Parameters:" << std::endl;
                    std::cout << "- Camera: Fisheye" << std::endl;
                    std::cout << "- fx: " << fx << std::endl;
                    std::cout << "- fy: " << fy << std::endl;
                    std::cout << "- cx: " << cx << std::endl;
                    std::cout << "- cy: " << cy << std::endl;
                    std::cout << "- k1: " << k1 << std::endl;
                    std::cout << "- k2: " << k2 << std::endl;
                    std::cout << "- k3: " << k3 << std::endl;
                    std::cout << "- k4: " << k4 << std::endl;

                    std::cout << "- mTlr: \n"
                              << mTlr << std::endl;

                    std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
                }
            }

            if (b_miss_params)
            {
                return false;
            }

            mpAtlas->AddCamera(mpCamera);
            mpAtlas->AddCamera(mpCamera2);
        }
        else
        {
            std::cerr << "*Not Supported Camera Sensor*" << std::endl;
            std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
        }

        if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
        {
            cv::FileNode node = fSettings["Camera.bf"];
            if (!node.empty() && node.isReal())
            {
                mbf = node.real();
            }
            else
            {
                std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
        }

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalisation
        mMinFrames = 0;
        mMaxFrames = fps;

        cout << "- fps: " << fps << endl;

        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;

        if (mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO)
        {
            float fx = mpCamera->getParameter(0);
            cv::FileNode node = fSettings["ThDepth"];
            if (!node.empty() && node.isReal())
            {
                mThDepth = node.real();
                mThDepth = mbf * mThDepth / fx;
                cout << endl
                     << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
            }
            else
            {
                std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
        }

        if (mSensor == System::RGBD)
        {
            cv::FileNode node = fSettings["DepthMapFactor"];
            if (!node.empty() && node.isReal())
            {
                mDepthMapFactor = node.real();
                if (fabs(mDepthMapFactor) < 1e-5)
                    mDepthMapFactor = 1;
                else
                    mDepthMapFactor = 1.0f / mDepthMapFactor;
            }
            else
            {
                std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
        }

        if (b_miss_params)
        {
            return false;
        }

        return true;
    }

    bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
    {
        bool b_miss_params = false;
        int nFeatures, nPyrLevels, mGridSize, fIniThFAST, fMinThFAST;
        float fScaleFactor;

        cv::FileNode node = fSettings["ORBextractor.nFeatures"];
        if (!node.empty() && node.isInt())
        {
            nFeatures = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.scaleFactor"];
        if (!node.empty() && node.isReal())
        {
            fScaleFactor = node.real();
        }
        else
        {
            std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.n_pyr_levels"];
        if (!node.empty() && node.isInt())
        {
            nPyrLevels = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.n_pyr_levels parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.klt_max_level"];
        if (!node.empty() && node.isInt())
        {
            nKltMaxLevel = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.klt_max_level parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.klt_min_level"];
        if (!node.empty() && node.isInt())
        {
            nKltMinLevel = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.klt_min_level parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.grid_size"];
        if (!node.empty() && node.isInt())
        {
            mGridSize = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.grid_size parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.iniThFAST"];
        if (!node.empty() && node.isInt())
        {
            fIniThFAST = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.minThFAST"];
        if (!node.empty() && node.isInt())
        {
            fMinThFAST = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        if (b_miss_params)
        {
            return false;
        }

        //图像构建５层金子塔，用于sparseImgAlign的快速追踪，特征点只构建３层．
        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nPyrLevels, std::max(nPyrLevels, nKltMaxLevel + 1), mGridSize, fIniThFAST, fMinThFAST);

        if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nPyrLevels, std::max(nPyrLevels, nKltMaxLevel + 1), mGridSize, fIniThFAST, fMinThFAST);

        if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
            mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nPyrLevels, std::max(nPyrLevels, nKltMaxLevel + 1), mGridSize, fIniThFAST, fMinThFAST);

        cout << endl
             << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << std::max(nPyrLevels, nKltMaxLevel + 1) << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- mGridSize: " << mGridSize << endl;
        cout << "- nKltMaxLevel: " << nKltMaxLevel << endl;
        cout << "- nKltMinLevel: " << nKltMinLevel << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        return true;
    }

    bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
    {
        bool b_miss_params = false;

        cv::Mat Tbc;
        cv::FileNode node = fSettings["Tbc"];
        if (!node.empty())
        {
            Tbc = node.mat();
            if (Tbc.rows != 4 || Tbc.cols != 4)
            {
                std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
                b_miss_params = true;
            }
        }
        else
        {
            std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
            b_miss_params = true;
        }

        cout << endl;

        cout << "Left camera to Imu Transform (Tbc): " << endl
             << Tbc << endl;

        float freq, Ng, Na, Ngw, Naw;

        node = fSettings["IMU.Frequency"];
        if (!node.empty() && node.isInt())
        {
            freq = node.operator int();
        }
        else
        {
            std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.NoiseGyro"];
        if (!node.empty() && node.isReal())
        {
            Ng = node.real();
        }
        else
        {
            std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.NoiseAcc"];
        if (!node.empty() && node.isReal())
        {
            Na = node.real();
        }
        else
        {
            std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.GyroWalk"];
        if (!node.empty() && node.isReal())
        {
            Ngw = node.real();
        }
        else
        {
            std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.AccWalk"];
        if (!node.empty() && node.isReal())
        {
            Naw = node.real();
        }
        else
        {
            std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        if (b_miss_params)
        {
            return false;
        }

        const float sf = sqrt(freq);
        cout << endl;
        cout << "IMU frequency: " << freq << " Hz" << endl;
        cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
        cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
        cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
        cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

        mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);

        return true;
    }

    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper = pLocalMapper;
    }

    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
    {
        mpLoopClosing = pLoopClosing;
    }

    void Tracking::SetViewer(Viewer *pViewer)
    {
        mpViewer = pViewer;
    }

    void Tracking::SetStepByStep(bool bSet)
    {
        bStepByStep = bSet;
    }

    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename)
    {
        mImGray = imRectLeft;
        cv::Mat imGrayRight = imRectRight;
        mImRight = imRectRight;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }

        if (mSensor == System::STEREO && !mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
        else if (mSensor == System::STEREO && mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr);
        else if (mSensor == System::IMU_STEREO && !mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);
        else if (mSensor == System::IMU_STEREO && mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr, &mLastFrame, *mpImuCalib);

        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

        Track();

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        double t_track = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();

        /*cout << "trracking time: " << t_track << endl;
    f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
        f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
        f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
        f_track_times << mTime_PreIntIMU << ",";
        f_track_times << mTime_PosePred << ",";
        f_track_times << mTime_LocalMapTrack << ",";
        f_track_times << mTime_NewKF_Dec << ",";
        f_track_times << t_track << endl;
#endif

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp, string filename)
    {
        mImGray = imRGB;
        cv::Mat imDepth = imD;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);

        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

        Track();

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        double t_track = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();

        /*f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
        f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
        f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
        f_track_times << mTime_PreIntIMU << ",";
        f_track_times << mTime_PosePred << ",";
        f_track_times << mTime_LocalMapTrack << ",";
        f_track_times << mTime_NewKF_Dec << ",";
        f_track_times << t_track << endl;
#endif

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
    {
        mImGray = im;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if (mSensor == System::MONOCULAR)
        {
            if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET || (lastID - initID) < mMaxFrames)
                mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
            else
                mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
        }
        else if (mSensor == System::IMU_MONOCULAR)
        {
            if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            {
                mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
            }
            else
                mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
        }

        if (mState == NO_IMAGES_YET)
            t0 = timestamp;

        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

        lastID = mCurrentFrame.mnId;
        Track();

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        double t_track = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();

        /*f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
        f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
        f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
        f_track_times << mTime_PreIntIMU << ",";
        f_track_times << mTime_PosePred << ",";
        f_track_times << mTime_LocalMapTrack << ",";
        f_track_times << mTime_NewKF_Dec << ",";
        f_track_times << t_track << endl;
#endif

        return mCurrentFrame.mTcw.clone();
    }

    void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
    {
        unique_lock<mutex> lock(mMutexImuQueue);
        mlQueueImuData.push_back(imuMeasurement);
    }

    void Tracking::PreintegrateIMU()
    {
        //cout << "start preintegration" << endl;

        if (!mCurrentFrame.mpPrevFrame)
        {
            Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
            mCurrentFrame.setIntegrated();
            return;
        }

        // cout << "start loop. Total meas:" << mlQueueImuData.size() << endl;

        mvImuFromLastFrame.clear();
        mvImuFromLastFrame.reserve(mlQueueImuData.size());
        if (mlQueueImuData.size() == 0)
        {
            Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
            mCurrentFrame.setIntegrated();
            return;
        }

        while (true)
        {
            bool bSleep = false;
            {
                unique_lock<mutex> lock(mMutexImuQueue);
                if (!mlQueueImuData.empty())
                {
                    IMU::Point *m = &mlQueueImuData.front();
                    cout.precision(17);
                    if (m->t < mCurrentFrame.mpPrevFrame->mTimeStamp - 0.001l)
                    {
                        mlQueueImuData.pop_front();
                    }
                    else if (m->t < mCurrentFrame.mTimeStamp - 0.001l)
                    {
                        mvImuFromLastFrame.push_back(*m);
                        mlQueueImuData.pop_front();
                    }
                    else
                    {
                        mvImuFromLastFrame.push_back(*m);
                        break;
                    }
                }
                else
                {
                    break;
                    bSleep = true;
                }
            }
            if (bSleep)
                usleep(500);
        }

        const int n = mvImuFromLastFrame.size() - 1;
        IMU::Preintegrated *pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

        for (int i = 0; i < n; i++)
        {
            float tstep;
            cv::Point3f acc, angVel;
            if ((i == 0) && (i < (n - 1)))
            {
                float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
                float tini = mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
                acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                       (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tini / tab)) *
                      0.5f;
                angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                          (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tini / tab)) *
                         0.5f;
                tstep = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
            }
            else if (i < (n - 1))
            {
                acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
                angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
                tstep = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            }
            else if ((i > 0) && (i == (n - 1)))
            {
                float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
                float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mTimeStamp;
                acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                       (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tend / tab)) *
                      0.5f;
                angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                          (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tend / tab)) *
                         0.5f;
                tstep = mCurrentFrame.mTimeStamp - mvImuFromLastFrame[i].t;
            }
            else if ((i == 0) && (i == (n - 1)))
            {
                acc = mvImuFromLastFrame[i].a;
                angVel = mvImuFromLastFrame[i].w;
                tstep = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
            }

            if (!mpImuPreintegratedFromLastKF)
                cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
            mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
            pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
        }

        mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

        mCurrentFrame.setIntegrated();

        Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
    }

    bool Tracking::PredictStateIMU()
    {
        if (!mCurrentFrame.mpPrevFrame)
        {
            Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        if (mbMapUpdated && mpLastKeyFrame)
        {
            const cv::Mat twb1 = mpLastKeyFrame->GetImuPosition();
            const cv::Mat Rwb1 = mpLastKeyFrame->GetImuRotation();
            const cv::Mat Vwb1 = mpLastKeyFrame->GetVelocity();

            const cv::Mat Gz = (cv::Mat_<float>(3, 1) << 0, 0, -IMU::GRAVITY_VALUE);
            const float t12 = mpImuPreintegratedFromLastKF->dT;

            cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
            cv::Mat twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
            cv::Mat Vwb2 = Vwb1 + t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
            mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);
            mCurrentFrame.mPredRwb = Rwb2.clone();
            mCurrentFrame.mPredtwb = twb2.clone();
            mCurrentFrame.mPredVwb = Vwb2.clone();
            mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
            mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
            return true;
        }
        else if (!mbMapUpdated)
        {
            const cv::Mat twb1 = mLastFrame.GetImuPosition();
            const cv::Mat Rwb1 = mLastFrame.GetImuRotation();
            const cv::Mat Vwb1 = mLastFrame.mVw;
            const cv::Mat Gz = (cv::Mat_<float>(3, 1) << 0, 0, -IMU::GRAVITY_VALUE);
            const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

            cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
            cv::Mat twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
            cv::Mat Vwb2 = Vwb1 + t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

            mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);
            mCurrentFrame.mPredRwb = Rwb2.clone();
            mCurrentFrame.mPredtwb = twb2.clone();
            mCurrentFrame.mPredVwb = Vwb2.clone();
            mCurrentFrame.mImuBias = mLastFrame.mImuBias;
            mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
            return true;
        }
        else
            cout << "not IMU prediction!!" << endl;

        return false;
    }

    void Tracking::ComputeGyroBias(const vector<Frame *> &vpFs, float &bwx, float &bwy, float &bwz)
    {
        const int N = vpFs.size();
        vector<float> vbx;
        vbx.reserve(N);
        vector<float> vby;
        vby.reserve(N);
        vector<float> vbz;
        vbz.reserve(N);

        cv::Mat H = cv::Mat::zeros(3, 3, CV_32F);
        cv::Mat grad = cv::Mat::zeros(3, 1, CV_32F);
        for (int i = 1; i < N; i++)
        {
            Frame *pF2 = vpFs[i];
            Frame *pF1 = vpFs[i - 1];
            cv::Mat VisionR = pF1->GetImuRotation().t() * pF2->GetImuRotation();
            cv::Mat JRg = pF2->mpImuPreintegratedFrame->JRg;
            cv::Mat E = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaRotation().t() * VisionR;
            cv::Mat e = IMU::LogSO3(E);
            assert(fabs(pF2->mTimeStamp - pF1->mTimeStamp - pF2->mpImuPreintegratedFrame->dT) < 0.01);

            cv::Mat J = -IMU::InverseRightJacobianSO3(e) * E.t() * JRg;
            grad += J.t() * e;
            H += J.t() * J;
        }

        cv::Mat bg = -H.inv(cv::DECOMP_SVD) * grad;
        bwx = bg.at<float>(0);
        bwy = bg.at<float>(1);
        bwz = bg.at<float>(2);

        for (int i = 1; i < N; i++)
        {
            Frame *pF = vpFs[i];
            pF->mImuBias.bwx = bwx;
            pF->mImuBias.bwy = bwy;
            pF->mImuBias.bwz = bwz;
            pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias);
            pF->mpImuPreintegratedFrame->Reintegrate();
        }
    }

    void Tracking::ComputeVelocitiesAccBias(const vector<Frame *> &vpFs, float &bax, float &bay, float &baz)
    {
        const int N = vpFs.size();
        const int nVar = 3 * N + 3; // 3 velocities/frame + acc bias
        const int nEqs = 6 * (N - 1);

        cv::Mat J(nEqs, nVar, CV_32F, cv::Scalar(0));
        cv::Mat e(nEqs, 1, CV_32F, cv::Scalar(0));
        cv::Mat g = (cv::Mat_<float>(3, 1) << 0, 0, -IMU::GRAVITY_VALUE);

        for (int i = 0; i < N - 1; i++)
        {
            Frame *pF2 = vpFs[i + 1];
            Frame *pF1 = vpFs[i];
            cv::Mat twb1 = pF1->GetImuPosition();
            cv::Mat twb2 = pF2->GetImuPosition();
            cv::Mat Rwb1 = pF1->GetImuRotation();
            cv::Mat dP12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaPosition();
            cv::Mat dV12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaVelocity();
            cv::Mat JP12 = pF2->mpImuPreintegratedFrame->JPa;
            cv::Mat JV12 = pF2->mpImuPreintegratedFrame->JVa;
            float t12 = pF2->mpImuPreintegratedFrame->dT;
            // Position p2=p1+v1*t+0.5*g*t^2+R1*dP12
            J.rowRange(6 * i, 6 * i + 3).colRange(3 * i, 3 * i + 3) += cv::Mat::eye(3, 3, CV_32F) * t12;
            J.rowRange(6 * i, 6 * i + 3).colRange(3 * N, 3 * N + 3) += Rwb1 * JP12;
            e.rowRange(6 * i, 6 * i + 3) = twb2 - twb1 - 0.5f * g * t12 * t12 - Rwb1 * dP12;
            // Velocity v2=v1+g*t+R1*dV12
            J.rowRange(6 * i + 3, 6 * i + 6).colRange(3 * i, 3 * i + 3) += -cv::Mat::eye(3, 3, CV_32F);
            J.rowRange(6 * i + 3, 6 * i + 6).colRange(3 * (i + 1), 3 * (i + 1) + 3) += cv::Mat::eye(3, 3, CV_32F);
            J.rowRange(6 * i + 3, 6 * i + 6).colRange(3 * N, 3 * N + 3) -= Rwb1 * JV12;
            e.rowRange(6 * i + 3, 6 * i + 6) = g * t12 + Rwb1 * dV12;
        }

        cv::Mat H = J.t() * J;
        cv::Mat B = J.t() * e;
        cv::Mat x(nVar, 1, CV_32F);
        cv::solve(H, B, x);

        bax = x.at<float>(3 * N);
        bay = x.at<float>(3 * N + 1);
        baz = x.at<float>(3 * N + 2);

        for (int i = 0; i < N; i++)
        {
            Frame *pF = vpFs[i];
            x.rowRange(3 * i, 3 * i + 3).copyTo(pF->mVw);
            if (i > 0)
            {
                pF->mImuBias.bax = bax;
                pF->mImuBias.bay = bay;
                pF->mImuBias.baz = baz;
                pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias);
            }
        }
    }

    void Tracking::ResetFrameIMU()
    {
        // TODO To implement...
    }

    void Tracking::Track()
    {
#ifdef SAVE_TIMES
        mTime_PreIntIMU = 0;
        mTime_PosePred = 0;
        mTime_LocalMapTrack = 0;
        mTime_NewKF_Dec = 0;
#endif

        if (bStepByStep)
        {
            while (!mbStep)
                usleep(500);
            mbStep = false;
        }

        if (mpLocalMapper->mbBadImu)
        {
            cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
            mpSystem->ResetActiveMap();
            return;
        }

        Map *pCurrentMap = mpAtlas->GetCurrentMap();

        if (mState != NO_IMAGES_YET)
        {
            if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp)
            {
                cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
                unique_lock<mutex> lock(mMutexImuQueue);
                mlQueueImuData.clear();
                CreateMapInAtlas();
                return;
            }
            else if (mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0)
            {
                cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
                if (mpAtlas->isInertial())
                {

                    if (mpAtlas->isImuInitialized())
                    {
                        cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                        if (!pCurrentMap->GetIniertialBA2())
                        {
                            mpSystem->ResetActiveMap();
                        }
                        else
                        {
                            CreateMapInAtlas();
                        }
                    }
                    else
                    {
                        cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                        mpSystem->ResetActiveMap();
                    }
                }

                return;
            }
        }

        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mpLastKeyFrame)
            mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

        if (mState == NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState = mState;

        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && !mbCreatedMap)
        {
#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#endif
            PreintegrateIMU();
#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            mTime_PreIntIMU = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();
#endif
        }
        mbCreatedMap = false;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

        mbMapUpdated = false;

        int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
        int nMapChangeIndex = pCurrentMap->GetLastMapChange();
        if (nCurMapChangeIndex > nMapChangeIndex)
        {
            // cout << "Map update detected" << endl;
            pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
            mbMapUpdated = true;
        }

        if (mState == NOT_INITIALIZED)
        {
            if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO)
                StereoInitialization();
            else
            {
                // MonocularInitialization();
                MonocularInitializationDirect();
            }

            mpFrameDrawer->Update(this);

            if (mState != OK) // If rightly initialized, mState=OK
            {
                mLastFrame = Frame(mCurrentFrame);
                return;
            }

            if (mpAtlas->GetAllMaps().size() == 1)
            {
                mnFirstFrameId = mCurrentFrame.mnId;
            }
        }
        else
        {
            // System is initialized. Track Frame.
            bool bOK;

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            if (!mbOnlyTracking)
            {
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point timeStartPosePredict = std::chrono::steady_clock::now();
#endif

                // State OK
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.
                if (mState == OK)
                {

                    // Local Mapping might have changed some MapPoints tracked in last frame
                    CheckReplacedInLastFrame();

                    if ((mVelocity.empty() && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                    {
                        //Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                        bOK = TrackReferenceKeyFrame();
                    }
                    else
                    {
                        //Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                        bOK = TrackWithSparseImgAlign();
                        // bOK = TrackWithMotionModel();
                        if (!bOK)
                        {
                            bOK = TrackReferenceKeyFrame();
                        }
                    }

                    if (!bOK)
                    {
                        if (mCurrentFrame.mnId <= (mnLastRelocFrameId + mnFramesToResetIMU) &&
                            (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))
                        {
                            mState = LOST;
                        }
                        else if (pCurrentMap->KeyFramesInMap() > 10)
                        {
                            cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                            mState = RECENTLY_LOST;
                            mTimeStampLost = mCurrentFrame.mTimeStamp;
                            //mCurrentFrame.SetPose(mLastFrame.mTcw);
                        }
                        else
                        {
                            mState = LOST;
                        }
                    }
                }
                else
                {

                    if (mState == RECENTLY_LOST)
                    {
                        Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                        bOK = true;
                        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))
                        {
                            if (pCurrentMap->isImuInitialized())
                                PredictStateIMU();
                            else
                                bOK = false;

                            if (mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost)
                            {
                                mState = LOST;
                                Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                                bOK = false;
                            }
                        }
                        else
                        {
                            // TODO fix relocalization
                            bOK = Relocalization();
                            if (!bOK)
                            {
                                mState = LOST;
                                Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                                bOK = false;
                            }
                        }
                    }
                    else if (mState == LOST)
                    {

                        Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                        if (pCurrentMap->KeyFramesInMap() < 10)
                        {
                            mpSystem->ResetActiveMap();
                            cout << "Reseting current map..." << endl;
                        }
                        else
                            CreateMapInAtlas();

                        if (mpLastKeyFrame)
                            mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

                        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                        return;
                    }
                }

#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point timeEndPosePredict = std::chrono::steady_clock::now();

                mTime_PosePred = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(timeEndPosePredict - timeStartPosePredict).count();
#endif
            }
            else
            {
                // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
                if (mState == LOST)
                {
                    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                        Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                    bOK = Relocalization();
                }
                else
                {
                    if (!mbVO)
                    {
                        // In last frame we tracked enough MapPoints in the map
                        if (!mVelocity.empty())
                        {
                            // bOK = TrackWithMotionModel();
                            bOK = TrackWithSparseImgAlign();
                        }
                        else
                        {
                            bOK = TrackReferenceKeyFrame();
                        }
                    }
                    else
                    {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint *> vpMPsMM;
                        vector<bool> vbOutMM;
                        cv::Mat TcwMM;
                        if (!mVelocity.empty())
                        {
                            bOKMM = TrackWithMotionModel();
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.mTcw.clone();
                        }
                        bOKReloc = Relocalization();

                        if (bOKMM && !bOKReloc)
                        {
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if (mbVO)
                            {
                                for (int i = 0; i < mCurrentFrame.N; i++)
                                {
                                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                    {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        }
                        else if (bOKReloc)
                        {
                            mbVO = false;
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if (!mbOnlyTracking)
            {
                if (bOK)
                {
#ifdef SAVE_TIMES
                    std::chrono::steady_clock::time_point time_StartTrackLocalMap = std::chrono::steady_clock::now();
#endif
                    // bOK = TrackLocalMap();
                    bOK = TrackLocalMapDirect();
                    // bOK = TrackLocalMapDirectXiang();
#ifdef SAVE_TIMES
                    std::chrono::steady_clock::time_point time_EndTrackLocalMap = std::chrono::steady_clock::now();

                    mTime_LocalMapTrack = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndTrackLocalMap - time_StartTrackLocalMap).count();
#endif
                }
                if (!bOK)
                {
                    cout << "Fail to track local map!" << endl;
                }
            }
            else
            {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                if (bOK && !mbVO)
                {
                    // bOK = TrackLocalMap();
                    bOK = TrackLocalMapDirect();
                    // bOK = TrackLocalMapDirectXiang();
                }
            }

            if (bOK)
                mState = OK;
            else if (mState == OK)
            {
                if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                {
                    Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                    if (!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                    {
                        cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                        mpSystem->ResetActiveMap();
                    }

                    mState = RECENTLY_LOST;
                }
                else
                    mState = LOST; // visual to lost

                if (mCurrentFrame.mnId > mnLastRelocFrameId + mMaxFrames)
                {
                    mTimeStampLost = mCurrentFrame.mTimeStamp;
                }
            }

            // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
            if ((mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) && ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && pCurrentMap->isImuInitialized())
            {
                // TODO check this situation
                Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
                Frame *pF = new Frame(mCurrentFrame);
                pF->mpPrevFrame = new Frame(mLastFrame);

                // Load preintegration
                pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
            }

            if (pCurrentMap->isImuInitialized())
            {
                if (bOK)
                {
                    if (mCurrentFrame.mnId == (mnLastRelocFrameId + mnFramesToResetIMU))
                    {
                        cout << "RESETING FRAME!!!" << endl;
                        ResetFrameIMU();
                    }
                    else if (mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
                        mLastBias = mCurrentFrame.mImuBias;
                }
            }

            // Update drawer
            mpFrameDrawer->Update(this);
            if (!mCurrentFrame.mTcw.empty())
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            if (bOK || mState == RECENTLY_LOST)
            {
                // Update motion model
                if (!mLastFrame.mTcw.empty() && !mCurrentFrame.mTcw.empty())
                {
                    cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                    mVelocity = mCurrentFrame.mTcw * LastTwc;
                }
                else
                    mVelocity = cv::Mat();

                if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                // Clean VO matches
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (pMP)
                        if (pMP->Observations() < 1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // Delete temporal MapPoints
                for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
                {
                    MapPoint *pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();

#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point timeStartNewKF = std::chrono::steady_clock::now();
#endif
                bool bNeedKF = NeedNewKeyFrame();
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point timeEndNewKF = std::chrono::steady_clock::now();

                mTime_NewKF_Dec = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(timeEndNewKF - timeStartNewKF).count();
#endif

                // Check if we need to insert a new keyframe
                if (bNeedKF && (bOK || (mState == RECENTLY_LOST && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))))
                {
                    CreateNewKeyFrame();
                }

                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame. Only has effect if lastframe is tracked
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            if (mState == LOST)
            {
                if (pCurrentMap->KeyFramesInMap() <= 5)
                {
                    mpSystem->ResetActiveMap();
                    return;
                }
                if ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO))
                    if (!pCurrentMap->isImuInitialized())
                    {
                        Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                        mpSystem->ResetActiveMap();
                        return;
                    }

                CreateMapInAtlas();
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            mLastFrame = Frame(mCurrentFrame);
        }

        if (mState == OK || mState == RECENTLY_LOST)
        {
            // Store frame pose information to retrieve the complete camera trajectory afterwards.
            if (!mCurrentFrame.mTcw.empty())
            {
                cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
                mlRelativeFramePoses.push_back(Tcr);
                mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
                mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
                mlbLost.push_back(mState == LOST);
            }
            else
            {
                // This can happen if tracking is lost
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back(mState == LOST);
            }
        }
    }

    void Tracking::StereoInitialization()
    {
        if (mCurrentFrame.mbFeatureExtracted == false)
        {
            mCurrentFrame.ExtractFeatures();
        }

        if (mCurrentFrame.N > 500)
        {
            if (mSensor == System::IMU_STEREO)
            {
                if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
                {
                    cout << "not IMU meas" << endl;
                    return;
                }

                if (cv::norm(mCurrentFrame.mpImuPreintegratedFrame->avgA - mLastFrame.mpImuPreintegratedFrame->avgA) < 0.5)
                {
                    cout << "not enough acceleration" << endl;
                    return;
                }

                if (mpImuPreintegratedFromLastKF)
                    delete mpImuPreintegratedFromLastKF;

                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            // Set Frame pose to the origin (In case of inertial SLAM to imu)
            if (mSensor == System::IMU_STEREO)
            {
                cv::Mat Rwb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0, 3).colRange(0, 3).clone();
                cv::Mat twb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0, 3).col(3).clone();
                mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, cv::Mat::zeros(3, 1, CV_32F));
            }
            else
                mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

            // Create KeyFrame
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

            // Insert KeyFrame in the map
            mpAtlas->AddKeyFrame(pKFini);

            // Create MapPoints and asscoiate to KeyFrame
            if (!mpCamera2)
            {
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    float z = mCurrentFrame.mvDepth[i];
                    if (z > 0)
                    {
                        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                        MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                        pNewMP->AddObservation(pKFini, i);
                        pKFini->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpAtlas->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    }
                }
            }
            else
            {
                for (int i = 0; i < mCurrentFrame.Nleft; i++)
                {
                    int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                    if (rightIndex != -1)
                    {
                        cv::Mat x3D = mCurrentFrame.mvStereo3Dpoints[i];

                        MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                        pNewMP->AddObservation(pKFini, i);
                        pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft);

                        pKFini->AddMapPoint(pNewMP, i);
                        pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft);

                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpAtlas->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
                    }
                }
            }

            Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

            mpLocalMapper->InsertKeyFrame(pKFini);

            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId = mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;
            mnLastRelocFrameId = mCurrentFrame.mnId;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

            mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            mState = OK;
        }
    }

    bool Tracking::ProcessFirstFrame()
    {
        //提取第一帧的特征点
        mCurrentFrame.ExtractFeatures();
        std::cout << "11111  == " << mCurrentFrame.mvKeys.size() << "," << mCurrentFrame.mvImagePyramid.size() << "," << std::endl;
        cv::Mat img;
        cv::cvtColor(mCurrentFrame.mvImagePyramid[0], img, CV_GRAY2RGB);
        for (size_t i = 0; i < mCurrentFrame.mvKeys.size(); i++)
        {
            cv::Point p;
            p.x = mCurrentFrame.mvKeys[i].pt.x;
            p.y = mCurrentFrame.mvKeys[i].pt.y;
            cv::circle(img, p, 2, cv::Scalar(0, 255, 0));
        }
        cv::imshow("first_img", img);
        // cv::waitKey(0);

        // Set Reference Frame
        if (mCurrentFrame.mvKeys.size() > 100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            std::cout << "22222  == " << mInitialFrame.mvKeys.size() << std::endl;

            mvbPrevMatched.resize(mCurrentFrame.mvKeys.size());
            for (size_t i = 0; i < mCurrentFrame.mvKeys.size(); i++)
            {
                mvbPrevMatched[i] = mCurrentFrame.mvKeys[i].pt;
            }

            if (mpInitializer)
                delete mpInitializer;

            mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

            std::fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            if (mSensor == System::IMU_MONOCULAR)
            {
                if (mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            return true;
        }
        return false;
    }

    bool Tracking::ProcessSecondFrame()
    {
        std::cout << "start to calcOpticalFlowPyrLK " << std::endl;
        std::cout << "mInitialFrame.img_  " << mInitialFrame.mvImagePyramid[0].empty() << "," << mCurrentFrame.mvImagePyramid[0].empty() << std::endl;
        std::cout << "mInitialFrame.img_  " << mInitialFrame.mvImagePyramid.size() << "," << mCurrentFrame.mvImagePyramid[0].size() << std::endl;
        std::vector<cv::Point2f> px_ref;
        px_ref.clear();
        std::vector<cv::Point2f> px_cur;
        px_cur.clear();
        std::vector<int> idx_ref;
        idx_ref.clear();
        int num = 0;
        for (int i = 0; i < int(mInitialFrame.mvKeys.size()); i++)
        {
            if (mInitialFrame.mvbOutlier[i])
            {
                continue;
            }
            cv::Point2f p = cv::Point2f(mInitialFrame.mvKeys[i].pt.x, mInitialFrame.mvKeys[i].pt.y);
            px_ref.push_back(p);
            px_cur.push_back(p);
            idx_ref.push_back(i);
            //std::cout << "i = " << i << "," << idx_ref[num] << "p=" << p << "," << px_ref.size() << "," << px_cur.size() << "," << idx_ref.size() << "," << px_ref[num] << "," << px_cur[num] << std::endl;
            num++;
        }
        std::cout << "before opticalFlow lk ==" << px_ref.size() << std::endl;
        const double klt_win_size = 30.0;
        const int klt_max_iter = 30;
        const double klt_eps = 0.001;
        std::vector<uchar> status;
        std::vector<float> error;
        std::vector<float> min_eig_vec;
        cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, klt_max_iter, klt_eps);
        cv::calcOpticalFlowPyrLK(mInitialFrame.mvImagePyramid[0], mCurrentFrame.mvImagePyramid[0],
                                 px_ref, px_cur,
                                 status, error,
                                 cv::Size2i(klt_win_size, klt_win_size),
                                 4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

        std::vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
        std::vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
        std::vector<double> disparities;
        disparities.clear();
        disparities.reserve(px_cur.size());
        int bad = 0;
        for (int i = 0; px_ref_it != px_ref.end(); ++i)
        {
            if (!status[i])
            {
                //std::cout << "bad   ===== " << idx_ref[i] << "," << px_cur_it->x << "," << px_cur_it->y << "------>" << px_ref_it->x << "," << px_ref_it->y << "," << bad << std::endl;
                mInitialFrame.mvbOutlier[idx_ref[i]] = true;
                ++px_ref_it;
                ++px_cur_it;
                bad++;
                continue;
            }
            disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
            //std::cout << "optical flow point xy = " << px_cur_it->x << "," << px_cur_it->y << "------>" << px_ref_it->x << "," << px_ref_it->y << "," << bad << std::endl;
            ++px_ref_it;
            ++px_cur_it;
        }

        double disparity = getMedian(disparities);
        std::cout << "disparity == " << disparity << "," << disparities.size() << std::endl;
        // if (disparity < 50)
        // {
        //     fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
        //     return false;
        // }
        //mvIniMatches
        std::vector<cv::DMatch> matches1to2;
        mvIniMatches = std::vector<int>(mInitialFrame.mvKeys.size(), -1);
        int nmatches = px_cur.size();
        std::cout << "nmatches == " << nmatches << std::endl;
        mCurrentFrame.mvKeys.reserve(nmatches);
        int count = 0;
        for (size_t i = 0; i < px_cur.size(); i++)
        {
            if (!status[i] || px_cur[i].x < 0 || px_cur[i].x > 640 || px_cur[i].y < 0 || px_cur[i].y > 480)
            {
                continue;
            }
            mvIniMatches[idx_ref[i]] = count;
            mInitialFrame.mvKeys[idx_ref[i]].octave = 0; //前两帧的level置为０
            cv::KeyPoint p1 = mInitialFrame.mvKeys[idx_ref[i]];
            cv::KeyPoint p2 = p1; //主要用于赋值p1的size个level等参数
            p2.pt.x = px_cur[i].x;
            p2.pt.y = px_cur[i].y;
            mCurrentFrame.mvKeys.push_back(p2);

            cv::DMatch m;
            m.queryIdx = idx_ref[i];
            m.trainIdx = count;
            matches1to2.push_back(m);

            count++;
            // std::cout << " i==" << i << ",  idx_ref[i] ==" << idx_ref[i] << std::endl;
            // std::cout << "p1-> p2　 == " << p1.pt << "," << p2.pt << std::endl;
        }
        mCurrentFrame.SetKeypoints();
        std::cout << "光流匹配 mCurrentFrame　 == " << nmatches << "," << mCurrentFrame.mvKeysUn.size() << "," << mCurrentFrame.mvKeys.size() << std::endl;
        std::cout << "光流匹配 mInitialFrame　 == " << nmatches << "," << mInitialFrame.mvKeysUn.size() << "," << mInitialFrame.mvKeys.size() << std::endl;
        // {
        //     /////////////////////////////绘制匹配及3D重投影结果////////////////////////////////
        //     cv::Mat img_1, img_2;
        //     cv::cvtColor(mInitialFrame.mvImagePyramid[0], img_1, CV_GRAY2RGB);
        //     cv::cvtColor(mCurrentFrame.mvImagePyramid[0], img_2, CV_GRAY2RGB);
        //     double fx = 493.899502203882;
        //     double fy = 493.810934883376;
        //     double cx = 318.692071563937;
        //     double cy = 240.389706485301;
        //     cv::Mat img_RR_matches;
        //     std::cout << "matches1to2.size() == " << matches1to2.size() << std::endl;
        //     cv::drawMatches(img_1, mInitialFrame.mvKeysUn, img_2, mCurrentFrame.mvKeysUn, matches1to2, img_RR_matches);
        //     cv::imshow("光流匹配", img_RR_matches);
        // }

        // Check if there are enough correspondences
        if (nmatches < 100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
            return false;
        }

        cv::Mat Rcw;                      // Current Camera Rotation
        cv::Mat tcw;                      // Current Camera Translation
        std::vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
        if (mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn, mCurrentFrame.mvKeysUn, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            std::cout << "mpCamera->ReconstructWithTwoViews " << std::endl;
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
            {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
                {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // set ouelier flag
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
            {
                if (mvIniMatches[i] == -1)
                {
                    mInitialFrame.mvbOutlier[i] = true;
                }
            }
            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
            Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
            tcw.copyTo(Tcw.rowRange(0, 3).col(3));
            mCurrentFrame.SetPose(Tcw);

            std::cout << "1111三角化　mInitialFrame == " << mInitialFrame.mTimeStamp << std::endl;
            std::cout << "1111三角化　mCurrentFrame == " << mCurrentFrame.mTimeStamp << std::endl;
            std::cout << "1111Tcw  == " << Tcw << std::endl;
            std::cout << "3D size   == " << nmatches << std::endl;
            CreateInitialMapMonocular();
        }
        else
        {
            return false;
        }
        /////////////////////////////绘制匹配及3D重投影结果////////////////////////////////
        cv::Mat img_1, img_2;
        cv::cvtColor(mInitialFrame.mvImagePyramid[0], img_1, CV_GRAY2RGB);
        cv::cvtColor(mCurrentFrame.mvImagePyramid[0], img_2, CV_GRAY2RGB);
        double fx = 493.899502203882;
        double fy = 493.810934883376;
        double cx = 318.692071563937;
        double cy = 240.389706485301;
        cv::Mat img_RR_matches;
        std::cout << "matches1to2.size() == " << matches1to2.size() << std::endl;
        cv::drawMatches(img_1, mInitialFrame.mvKeysUn, img_2, mCurrentFrame.mvKeysUn, matches1to2, img_RR_matches);
        for (size_t i = 0; i < mvIniP3D.size(); i++)
        {
            if (!vbTriangulated[i])
            {
                continue;
            }
            cv::Mat p3d = (cv::Mat_<float>(3, 1)
                               << mvIniP3D[i].x,
                           mvIniP3D[i].y, mvIniP3D[i].z);
            cv::Mat p_cam = (Rcw * p3d + tcw);
            double u = p_cam.at<float>(0, 0) / p_cam.at<float>(2, 0) * fx + cx;
            double v = p_cam.at<float>(1, 0) / p_cam.at<float>(2, 0) * fy + cy;
            cv::circle(img_2, cv::Point2f(u, v), 2, cv::Scalar(0, 255, 0));
            cv::circle(img_2, mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt, 2, cv::Scalar(0, 0, 255));
            cv::line(img_2, cv::Point2f(u, v), mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt, cv::Scalar(255, 0, 0));

            if (sqrt((u - mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.x) * (u - mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.x) + (v - mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.y) * (v - mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt.y)) > 5)
            {
                std::cout << p3d << std::endl;
                std::cout << cv::Point2f(u, v) << std::endl;
                std::cout << mCurrentFrame.mvKeysUn[mvIniMatches[i]].pt << std::endl;
                std::cout << "--------------------" << std::endl;
            }
        }
        cv::imshow("loop match", img_RR_matches);
        cv::imshow("pnp_result", img_2);
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        return true;
    }

    void Tracking::MonocularInitializationDirect()
    {
        //processFirstFrame
        if (!mpInitializer)
        {
            if (!ProcessFirstFrame())
            {
                return;
            }
        } //processSecondFrame
        else
        {
            if (!ProcessSecondFrame())
            {
                return;
            }
        }
    }

    void Tracking::MonocularInitialization()
    {
        if (mCurrentFrame.mbFeatureExtracted == false)
        {
            mCurrentFrame.ExtractFeatures();
        }

        if (!mpInitializer)
        {
            // Set Reference Frame
            if (mCurrentFrame.mvKeys.size() > 100)
            {
                mInitialFrame = Frame(mCurrentFrame);
                mLastFrame = Frame(mCurrentFrame);
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

                if (mpInitializer)
                    delete mpInitializer;

                mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                if (mSensor == System::IMU_MONOCULAR)
                {
                    if (mpImuPreintegratedFromLastKF)
                    {
                        delete mpImuPreintegratedFromLastKF;
                    }
                    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
                }
                return;
            }
        }
        else
        {
            if (((int)mCurrentFrame.mvKeys.size() <= 100) || ((mSensor == System::IMU_MONOCULAR) && (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0)))
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                return;
            }

            std::cout << "mCurrentFrame in initialization  " << std::to_string(mCurrentFrame.mTimeStamp) << std::endl;
            // Find correspondences
            ORBmatcher matcher(0.9, true);
            int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

            // Check if there are enough correspondences
            if (nmatches < 100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
                return;
            }

            cv::Mat Rcw;                 // Current Camera Rotation
            cv::Mat tcw;                 // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            if (mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn, mCurrentFrame.mvKeysUn, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
            {
                for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
                {
                    if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i] = -1;
                        nmatches--;
                    }
                }
                std::cout << "Initialize finished  " << mvIniMatches.size() << std::endl;

                // Set Frame Poses
                mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
                Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
                tcw.copyTo(Tcw.rowRange(0, 3).col(3));
                mCurrentFrame.SetPose(Tcw);

                CreateInitialMapMonocular();

                // Just for video
                // bStepByStep = true;
            }
        }
    }

    void Tracking::CreateInitialMapMonocular()
    {
        // Create KeyFrames
        KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
        //第二帧光流跟踪的，还没有进行描述子计算
        if (mCurrentFrame.mbFeatureExtracted == false)
        {
            mCurrentFrame.ExtractFeatures();
        }
        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        if (mSensor == System::IMU_MONOCULAR)
            pKFini->mpImuPreintegrated = (IMU::Preintegrated *)(NULL);

        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // Insert KFs in the map
        mpAtlas->AddKeyFrame(pKFini);
        mpAtlas->AddKeyFrame(pKFcur);

        for (size_t i = 0; i < mvIniMatches.size(); i++)
        {
            if (mvIniMatches[i] < 0)
                continue;

            //Create MapPoint.
            cv::Mat worldPos(mvIniP3D[i]);
            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

            // std::cout << "地图点两帧的level == " << pKFini->mvKeys[i].octave << "," << pKFcur->mvKeys[mvIniMatches[i]].octave << std::endl;
            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            //Add to Map
            mpAtlas->AddMapPoint(pMP);
        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        std::set<MapPoint *> sMPs;
        sMPs = pKFini->GetMapPoints();

        // Bundle Adjustment
        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
        Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

        pKFcur->PrintPointDistribution();

        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth;
        if (mSensor == System::IMU_MONOCULAR)
            invMedianDepth = 4.0f / medianDepth; // 4.0f
        else
            invMedianDepth = 1.0f / medianDepth;

        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50) // TODO Check, originally 100 tracks
        {
            Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_NORMAL);
            mpSystem->ResetActiveMap();
            return;
        }

        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
        {
            if (vpAllMapPoints[iMP])
            {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
                pMP->UpdateNormalAndDepth();
            }
        }

        if (mSensor == System::IMU_MONOCULAR)
        {
            pKFcur->mPrevKF = pKFini;
            pKFini->mNextKF = pKFcur;
            pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);
        mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;
        mnLastRelocFrameId = mInitialFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        // Compute here initial velocity
        vector<KeyFrame *> vKFs = mpAtlas->GetAllKeyFrames();

        cv::Mat deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
        mVelocity = cv::Mat();
        Eigen::Vector3d phi = LogSO3(Converter::toMatrix3d(deltaT.rowRange(0, 3).colRange(0, 3)));

        double aux = (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) / (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
        phi *= aux;

        mLastFrame = Frame(mCurrentFrame);

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;

        initID = pKFcur->mnId;
    }

    void Tracking::CreateMapInAtlas()
    {
        mnLastInitFrameId = mCurrentFrame.mnId;
        mpAtlas->CreateNewMap();
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
            mpAtlas->SetInertialSensor();
        mbSetInit = false;

        mnInitialFrameId = mCurrentFrame.mnId + 1;
        mState = NO_IMAGES_YET;

        // Restart the variable with information about the last KF
        mVelocity = cv::Mat();
        mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
        Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId + 1), Verbose::VERBOSITY_NORMAL);
        mbVO = false; // Init value for know if there are enough MapPoints in the last KF
        if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
        {
            if (mpInitializer)
                delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
        }

        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mpImuPreintegratedFromLastKF)
        {
            delete mpImuPreintegratedFromLastKF;
            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        }

        if (mpLastKeyFrame)
            mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

        if (mpReferenceKF)
            mpReferenceKF = static_cast<KeyFrame *>(NULL);

        mLastFrame = Frame();
        mCurrentFrame = Frame();
        mvIniMatches.clear();

        mbCreatedMap = true;
    }

    void Tracking::CheckReplacedInLastFrame()
    {
        for (int i = 0; i < mLastFrame.N; i++)
        {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP)
            {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep)
                {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }

    bool Tracking::TrackWithSparseImgAlign()
    {
        std::cout << "TrackWithSparseImgAlign======" << std::to_string(mCurrentFrame.mTimeStamp) << std::endl;
        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame();

        // std::cout << "TrackWithSparseImgAlign == " << mLastFrame.mTcw << std::endl;
        //初始位姿可以用上一帧的位姿，也可以用速度模型估计，还可以用imu预积分估计当前帧的初始位姿
        // mCurrentFrame.SetPose(mLastFrame.mTcw);
        mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);
        std::cout << "mVelocity == " << mVelocity << "," << mLastFrame.mTcw << std::endl;

        // check if last frame have enough observations
        size_t inliers_in_last_frame = 0;
        for (int i = 0; i < mLastFrame.N; i++)
            if (mLastFrame.mvpMapPoints[i] && mLastFrame.mvpMapPoints[i]->isBad() == false &&
                mLastFrame.mvbOutlier[i] == false)
                inliers_in_last_frame++;
        if (inliers_in_last_frame < 30)
        {
            std::cout << "Last frame have less observations: " << inliers_in_last_frame
                      << ", sparse alignment may have a erroneous result, return back to feature method." << endl;
            return false;
        }

        // sparse image align
        SparseImgAlign img_align(nKltMaxLevel, nKltMinLevel, 10, SparseImgAlign::GaussNewton, false, false);
        cv::Mat Tmat = img_align.run(&mLastFrame, &mCurrentFrame);
        // std::cout << "SparseImgAlign mCurrentFrame Tcw == " << mCurrentFrame.mTcw << std::endl;
        // std::cout << "SparseImgAlign mLastFrame Tcw == " << mLastFrame.mTcw << std::endl;
        // if (ret == false)
        // {
        //     std::cout << "Failed. return back to feature methods" << endl;
        //     mCurrentFrame.SetPose(mLastFrame.mTcw);
        //     return false;
        // }
        mCurrentFrame.SetPose(Tmat);

        return true;
    }

    bool Tracking::TrackReferenceKeyFrame()
    {
        std::cout << "TrackReferenceKeyFrame ===== " << std::to_string(mCurrentFrame.mTimeStamp) << std::endl;
        // When tracking with reference keyframe, we need features and key points, direct method cannot work
        if (mCurrentFrame.mbFeatureExtracted == false)
        {
            mCurrentFrame.ExtractFeatures(); // 未提特征则加入新的特征
        }
        else
        {
            // 已经提过就算了
        }

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches;

        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

        if (nmatches < 15)
        {
            cout << "TRACK_REF_KF: Less than 15 matches!!\n";
            return false;
        }

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);

        //mCurrentFrame.PrintPointDistribution();

        // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            //if(i >= mCurrentFrame.Nleft) break;
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    if (i < mCurrentFrame.Nleft)
                    {
                        pMP->mbTrackInView = false;
                    }
                    else
                    {
                        pMP->mbTrackInViewR = false;
                    }
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        // TODO check these conditions
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            return true;
        else
            return nmatchesMap >= 10;
    }

    /**
     * @brief 双目或rgbd摄像头根据深度值为上一帧产生新的MapPoints
     *
     * 在双目和rgbd情况下，选取一些深度小一些的点（可靠一些） \n
     * 可以通过深度值产生一些新的MapPoints
    */
    void Tracking::UpdateLastFrame()
    {
        // Update pose according to reference keyframe
        KeyFrame *pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();
        mLastFrame.SetPose(Tlr * pRef->GetPose());

        std::cout << "last keyframe pose == " << Tlr << "\n"
                  << pRef->GetPose() << "\n"
                  << mLastFrame.mTcw << std::endl;

        if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR || !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float, int>> vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for (int i = 0; i < mLastFrame.N; i++)
        {
            float z = mLastFrame.mvDepth[i];
            if (z > 0)
            {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (vDepthIdx.empty())
            return;

        sort(vDepthIdx.begin(), vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx.size(); j++)
        {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint *pMP = mLastFrame.mvpMapPoints[i];
            if (!pMP)
                bCreateNew = true;
            else if (pMP->Observations() < 1)
            {
                bCreateNew = true;
            }

            if (bCreateNew)
            {
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                MapPoint *pNewMP = new MapPoint(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);

                mLastFrame.mvpMapPoints[i] = pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                nPoints++;
            }

            if (vDepthIdx[j].first > mThDepth && nPoints > 100)
            {
                break;
            }
        }
    }

    bool Tracking::TrackWithMotionModel()
    {
        std::cout << "TrackWithMotionModel ===== " << std::to_string(mCurrentFrame.mTimeStamp) << std::endl;
        if (mCurrentFrame.mbFeatureExtracted == false)
        {
            mCurrentFrame.ExtractFeatures();
        }

        ORBmatcher matcher(0.9, true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame();

        if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU))
        {
            // Predict ste with IMU if it is initialized and it doesnt need reset
            PredictStateIMU();
            return true;
        }
        else
        {
            mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);
        }

        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Project points seen in previous frame
        int th;

        if (mSensor == System::STEREO)
            th = 7;
        else
            th = 15;

        int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);

        // If few matches, uses a wider window search
        if (nmatches < 20)
        {
            Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
            fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

            nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);
            Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);
        }

        if (nmatches < 20)
        {
            Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                return true;
            else
                return false;
        }

        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    if (i < mCurrentFrame.Nleft)
                    {
                        pMP->mbTrackInView = false;
                    }
                    else
                    {
                        pMP->mbTrackInViewR = false;
                    }
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        if (mbOnlyTracking)
        {
            mbVO = nmatchesMap < 10;
            return nmatches > 20;
        }

        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            return true;
        else
            return nmatchesMap >= 10;
    }

    bool Tracking::TrackLocalMap()
    {
        std::cout << "TrackLocalMap====" << std::to_string(mCurrentFrame.mTimeStamp) << std::endl;
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        mTrackedFr++;
        if (mvpLocalMapPoints.size() == 0)
            UpdateLocalMap();

        if (mCurrentFrame.mbFeatureExtracted == false)
        {
            mCurrentFrame.N = 0;
            mCurrentFrame.ExtractFeatures();
        }

        SearchLocalPoints();

        // TOO check outliers before PO
        int aux1 = 0, aux2 = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
            if (mCurrentFrame.mvpMapPoints[i])
            {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i])
                    aux2++;
            }

        int inliers;
        if (!mpAtlas->isImuInitialized())
            Optimizer::PoseOptimization(&mCurrentFrame);
        else
        {
            if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU)
            {
                Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
                Optimizer::PoseOptimization(&mCurrentFrame);
            }
            else
            {
                // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
                if (!mbMapUpdated) //  && (mnMatchesInliers>30))
                {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                    inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                }
                else
                {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                    inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                }
            }
        }

        aux1 = 0, aux2 = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
            if (mCurrentFrame.mvpMapPoints[i])
            {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i])
                    aux2++;
            }

        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking)
                    {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
                else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }
        std::cout << "当前帧的内点数目　＝＝　" << mnMatchesInliers << std::endl;
        UpdateLocalMap();
        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        mpLocalMapper->mnMatchesInliers = mnMatchesInliers;
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
            return false;

        if ((mnMatchesInliers > 10) && (mState == RECENTLY_LOST))
            return true;

        if (mSensor == System::IMU_MONOCULAR)
        {
            if (mnMatchesInliers < 15)
            {
                return false;
            }
            else
                return true;
        }
        else if (mSensor == System::IMU_STEREO)
        {
            if (mnMatchesInliers < 15)
            {
                return false;
            }
            else
                return true;
        }
        else
        {
            if (mnMatchesInliers < 30)
                return false;
            else
                return true;
        }
    }

    bool Tracking::TrackLocalMapDirectXiang()
    {
        std::cout << "TrackLocalMapDirectXiang====" << std::to_string(mCurrentFrame.mTimeStamp) << std::endl;
        // project the local map points into current frame, then search with direct align
        // 这步把 local map points 投影至当前帧并确定其位置
        if (mCurrentFrame.mbFeatureExtracted == true) // 此帧已经提了特征，用特征点法的local mapping来处理
        {
            // if we have extracted features, do it with feature matching
            std::cout << "This frame already have features, using Track Local Map instead." << endl;
            return TrackLocalMap();
        }
        int count1 = 0;
        int count2 = 0;
        for (MapPoint *mp : mvpLocalMapPoints)
        {
            // 后续和上面是一样的
            if (!mp)
            {
                count1++;
                continue;
            }
            count2++;
        }
        std::cout << "mvpLocalMapPoints  count1===count2 ====" << mvpLocalMapPoints.size() << "," << count1 << "," << count2 << std::endl;

        SearchLocalPointsDirect();
        UpdateLocalKeyFrames();

        // compute the stereo key point and RGBD key point, pose optimizer will use that information
        if (mSensor == System::RGBD)
            mCurrentFrame.ComputeStereoFromRGBD(mCurrentFrame.mImDepth);
        else if (mSensor == System::STEREO)
            mCurrentFrame.ComputeStereoMatches();

        // TOO check outliers before PO
        int aux1 = 0, aux2 = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i])
                {
                    aux2++;
                }
            }
        }

        std::cout << "before   aux1, aux2 ==  " << aux1 << "," << aux2 << std::endl;

        int inliers;
        if (!mpAtlas->isImuInitialized())
            Optimizer::PoseOptimization(&mCurrentFrame);
        else
        {
            if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU)
            {
                Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
                Optimizer::PoseOptimization(&mCurrentFrame);
            }
            else
            {
                // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
                if (!mbMapUpdated) //  && (mnMatchesInliers>30))
                {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                    inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                }
                else
                {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                    inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                }
            }
        }

        aux1 = 0, aux2 = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i])
                    aux2++;
            }
        }
        std::cout << " 11111 track local map direct mState = " << mState << std::endl;

        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking)
                    {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        {
                            mnMatchesInliers++;
                        }
                    }
                    else
                    {
                        mnMatchesInliers++;
                    }
                }
                else if (mSensor == System::STEREO)
                {
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    // 这个投影是outlier，从cache里移除
                    auto iter = mvpDirectMapPointsCache.find(mCurrentFrame.mvpMapPoints[i]);
                    if (iter != mvpDirectMapPointsCache.end())
                        mvpDirectMapPointsCache.erase(iter);
                }
                else
                {
                    // 这个投影是outlier，从cache里移除
                    auto iter = mvpDirectMapPointsCache.find(mCurrentFrame.mvpMapPoints[i]);
                    if (iter != mvpDirectMapPointsCache.end())
                        mvpDirectMapPointsCache.erase(iter);
                }
            }
        }
        std::cout << "当前帧的内点数目　＝＝　" << mnMatchesInliers << std::endl;

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        mpLocalMapper->mnMatchesInliers = mnMatchesInliers;
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
        {
            return false;
        }

        if ((mnMatchesInliers > 10) && (mState == RECENTLY_LOST))
        {
            return true;
        }

        std::cout << " 11111 track local map direct mState = " << mState << std::endl;

        if (mSensor == System::IMU_MONOCULAR)
        {
            if (mnMatchesInliers < 15)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else if (mSensor == System::IMU_STEREO)
        {
            if (mnMatchesInliers < 15)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            if (mnMatchesInliers < 30)
            {
                std::cout << " failed track local map direct  ===  mnMatchesInliers = " << mnMatchesInliers << std::endl;
                std::cout << "mCurrentFrame.N == " << mCurrentFrame.N << std::endl;
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    void Tracking::SearchLocalPointsDirect()
    {
        int cntSuccess = 0;
        // use grid to evaluate the coverage of feature points
        const int grid_size = 5;
        const int grid_rows = mCurrentFrame.mvImagePyramid[0].rows / grid_size;
        const int grid_cols = mCurrentFrame.mvImagePyramid[0].cols / grid_size;
        vector<bool> grid(grid_rows * grid_cols, false);

        ORBmatcher matcher;
        std::cout << "mvpDirectMapPointsCache before1111 === " << mvpDirectMapPointsCache.size() << "   mvpLocalMapPoints.size() = " << mvpLocalMapPoints.size() << std::endl;
        if (!mvpDirectMapPointsCache.empty())
        {
            // 缓存不空，则在缓存中搜索
            for (auto iter = mvpDirectMapPointsCache.begin(); iter != mvpDirectMapPointsCache.end();)
            {
                MapPoint *mp = *iter;
                if (mp->isBad() || mCurrentFrame.isInFrustum(mp, 0.5) == false) // 坏蛋点或者在视野外
                {
                    iter = mvpDirectMapPointsCache.erase(iter);
                    continue;
                }

                int gx = static_cast<int>(mp->mTrackProjX / grid_size);
                int gy = static_cast<int>(mp->mTrackProjY / grid_size);
                int k = gy * grid_cols + gx;

                if (grid[k] == true)
                {
                    iter++;   // 这个点不知道能不能匹配，所以先保留在cache里头
                    continue; // already exist a projection
                }

                // try align it with current frame
                auto obs = mp->GetObservations();
                auto obs_sorted = SelectNearestKeyframe(obs, 5); //通过关键帧的id选择最近的关键帧
                std::vector<Eigen::Vector2d> matched_pixels;
                for (auto &o : obs_sorted)
                {
                    Eigen::Vector2d px_curr(mp->mTrackProjX, mp->mTrackProjY);
                    int level = mp->mnTrackScaleLevel;

                    if (matcher.FindDirectProjection(o.first, &mCurrentFrame, mp, px_curr, level))
                    {
                        if (px_curr[0] < 20 || px_curr[1] < 20 || px_curr[0] >= mCurrentFrame.mvImagePyramid[0].cols - 20 || px_curr[1] >= mCurrentFrame.mvImagePyramid[0].rows - 20)
                            continue; // 丢弃位于太边缘的地方的点，否则在创建关键帧，计算描述子时可能导致溢出
                        matched_pixels.push_back(px_curr);
                        break;
                    }
                }
                if (!matched_pixels.empty())
                {
                    //std::cout << "matched_pixed == " << matched_pixels.size() << "," << obs_sorted.size() << std::endl;
                    // 有成功追踪到的点，取平均像素位置为测量
                    Eigen::Vector2d px_ave(0, 0);
                    for (Eigen::Vector2d &p : matched_pixels)
                        px_ave += p;
                    px_ave = px_ave / matched_pixels.size();

                    // insert a feature and assign it to a map point
                    mCurrentFrame.mvKeys.push_back(
                        cv::KeyPoint(cv::Point2f(px_ave[0], px_ave[1]), 7, -1, 0, 0));
                    mCurrentFrame.mvKeysUn.push_back(
                        cv::KeyPoint(cv::Point2f(px_ave[0], px_ave[1]), 7, -1, 0, 0));
                    mCurrentFrame.mvpMapPoints.push_back(mp);
                    mCurrentFrame.mvDepth.push_back(-1);
                    mCurrentFrame.mvbOutlier.push_back(false);

                    int gx = static_cast<int>(px_ave[0] / grid_size);
                    int gy = static_cast<int>(px_ave[1] / grid_size);
                    int k = gy * grid_cols + gx;
                    grid[k] = true;

                    iter++;
                    cntSuccess++;
                }
                else
                {
                    // 一次都没匹配上
                    iter = mvpDirectMapPointsCache.erase(iter);
                }
            }
        }
        std::cout << "mvpDirectMapPointsCache after2222 === " << mvpDirectMapPointsCache.size() << "   mvpLocalMapPoints.size() = " << mvpLocalMapPoints.size() << std::endl;
        std::cout << "cntSuccess == " << cntSuccess << std::endl;
        if (cntSuccess > mnCacheHitTh)
        {
            // 从缓存中就得到了足够的匹配点，直接返回
            // we matched enough points in cache, then do pose optimization
            mCurrentFrame.N = mCurrentFrame.mvKeys.size();
            mCurrentFrame.mvuRight.resize(mCurrentFrame.N, -1);
            return;
        }

        // 否则，更新地图点，并从地图点中拿到更多的点
        // no enough projections, search more in local map points
        UpdateLocalMap();

        std::cout << "UpdateLocalMap == " << mvpLocalMapPoints.size() << std::endl;

        int rejected = 0;
        int outside = 0;
        for (MapPoint *mp : mvpLocalMapPoints)
        {
            if (mvpDirectMapPointsCache.find(mp) != mvpDirectMapPointsCache.end())
            {
                // 已经在缓存中（同时说明已经匹配）
                // already in cache (means already matched)
                continue;
            }
            // 后续和上面是一样的
            if (mp->isBad()) // 坏蛋点
                continue;
            if (mCurrentFrame.isInFrustum(mp, 0.5) == false) // 在视野外或视线角太大
            {
                outside++;
                continue;
            }

            // 如果上面那个判断通过，那么这个地图点在此帧的投影位置就有一个大致的估计
            // 比较这个点的某次观测与当前帧的图像
            ORBmatcher matcher;
            // 我们比较最近一些观测
            auto obs = mp->GetObservations();
            auto obs_sorted = SelectNearestKeyframe(obs, 5);
            std::vector<Eigen::Vector2d> matched_pixels;
            for (auto &o : obs_sorted)
            {
                Eigen::Vector2d px_curr(mp->mTrackProjX, mp->mTrackProjY);
                int level = mp->mnTrackScaleLevel;
                if (matcher.FindDirectProjection(o.first, &mCurrentFrame, mp, px_curr, level))
                {
                    if (px_curr[0] < 20 || px_curr[1] < 20 || px_curr[0] >= mCurrentFrame.mvImagePyramid[0].cols - 20 || px_curr[1] >= mCurrentFrame.mvImagePyramid[0].rows - 20)
                        continue; // 丢弃位于太边缘的地方的点，否则在创建关键帧，计算描述子时可能导致溢出
                    matched_pixels.push_back(px_curr);

                    break;
                }
            }

            if (!matched_pixels.empty())
            {
                // 有成功追踪到的点，取平均像素位置为测量
                Eigen::Vector2d px_ave(0, 0);
                for (Eigen::Vector2d &p : matched_pixels)
                    px_ave += p;
                px_ave = px_ave / matched_pixels.size();

                // insert a feature and assign it to a map point
                mCurrentFrame.mvKeys.push_back(
                    cv::KeyPoint(cv::Point2f(px_ave[0], px_ave[1]), 7, -1, 0, 0));
                mCurrentFrame.mvKeysUn.push_back(
                    cv::KeyPoint(cv::Point2f(px_ave[0], px_ave[1]), 7, -1, 0, 0));
                mCurrentFrame.mvpMapPoints.push_back(mp);
                mCurrentFrame.mvDepth.push_back(-1);
                mCurrentFrame.mvbOutlier.push_back(false);

                mvpDirectMapPointsCache.insert(mp);
            }
            else
            {
                // 一次都没匹配上，该点无效
                rejected++;
            }
        }

        mCurrentFrame.N = mCurrentFrame.mvKeys.size();
        mCurrentFrame.mvuRight.resize(mCurrentFrame.N, -1);
        std::cout << "mCurrentFrame.N ===  == " << mCurrentFrame.N << "," << rejected << "," << outside << std::endl;
    }

    vector<std::pair<KeyFrame *, std::tuple<int, int>>>
    Tracking::SelectNearestKeyframe(const std::map<KeyFrame *, std::tuple<int, int>> &observations, int n)
    {
        vector<std::pair<KeyFrame *, std::tuple<int, int>>> s;
        for (auto &o : observations)
        {
            if (!o.first->isBad() && o.first != mpLastKeyFrame)
                s.push_back(make_pair(o.first, o.second));
        }
        // 按照id排序
        // 然而这里选最近的点会容易导致飘移
        // sort( s.begin(), s.end(),
        // [](const pair<KeyFrame*, size_t>& p1, const pair<KeyFrame*, size_t>& p2) {return p1.first->mnId > p2.first->mnId; } );
        sort(s.begin(), s.end(),
             [](const pair<KeyFrame *, std::tuple<int, int>> &p1, const pair<KeyFrame *, std::tuple<int, int>> &p2) {
                 return p1.first->mnId > p2.first->mnId;
             });

        if ((int)s.size() < n)
            return s;
        else
            return vector<std::pair<KeyFrame *, std::tuple<int, int>>>(s.begin(), s.begin() + n);
    }

    bool Tracking::TrackLocalMapDirect()
    {
        mTrackedFr++;

        std::cout << "TrackLocalMapDirectXiang====" << std::to_string(mCurrentFrame.mTimeStamp) << std::endl;
        // project the local map points into current frame, then search with direct align
        // 这步把 local map points 投影至当前帧并确定其位置
        if (mCurrentFrame.mbFeatureExtracted == true) // 此帧已经提了特征，用特征点法的local mapping来处理
        {
            // if we have extracted features, do it with feature matching
            std::cout << "This frame already have features, using Track Local Map instead." << endl;
            return TrackLocalMap();
        }

        //将局部地图点全部投影到当前帧的网格中　3D点－＞投影的2D点．　　
        //在网格中找到3D点的观测帧与当前帧最近的帧．进行迭代匹配，得到精确的匹配点
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        std::fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
        IterativeMatcher iterativeMatcher(mpORBextractorLeft->GetFeatureLevels(), mpORBextractorLeft->GetGridSize(), mCurrentFrame.mImGray.cols, mCurrentFrame.mImGray.rows);

        int n_match = iterativeMatcher.ReprojectMap(mCurrentFrame, mvpLocalMapPoints);

        std::cout << "before mvpLocalMapPointsTen.size() == " << mvpLocalMapPoints.size() << std::endl;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        //PoseOptimization　　optimizeGaussNewton　＋　optimizeStructure
        UpdateLocalMap();
        std::cout << "after UpdateLocalMap  mvpLocalMapPoints.size() ==  " << mvpLocalMapPoints.size() << "," << n_match << std::endl;
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        std::cout << "TrackLocalMapDirect  time    ReprojectMap cost ==  " << std::to_string(std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count()) << std::endl;
        std::cout << "TrackLocalMapDirect  time    UpdateLocalMap cost ==  " << std::to_string(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()) << std::endl;

        // TOO check outliers before PO
        int aux1 = 0, aux2 = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i])
                {
                    aux2++;
                }
            }
        }

        std::cout << "before   aux1, aux2 ==  " << aux1 << "," << aux2 << std::endl;
        /////////////////////////////绘制3D重投影结果////////////////////////////////
        // cv::Mat img_2;
        // cv::cvtColor(mCurrentFrame.mvImagePyramid[0], img_2, CV_GRAY2RGB);
        // double fx = 493.899502203882;
        // double fy = 493.810934883376;
        // double cx = 318.692071563937;
        // double cy = 240.389706485301;
        // for (size_t i = 0; i < mCurrentFrame.mvpMapPoints.size(); i++)
        // {
        //     if (mCurrentFrame.mvbOutlier[i])
        //     {
        //         continue;
        //     }
        //     MapPoint *p3d = mCurrentFrame.mvpMapPoints[i];
        //     cv::Mat p3d_w = p3d->GetWorldPos();
        //     cv::Mat p_cam = (mCurrentFrame.mRcw * p3d_w + mCurrentFrame.mtcw);
        //     double u = p_cam.at<float>(0, 0) / p_cam.at<float>(2, 0) * fx + cx;
        //     double v = p_cam.at<float>(1, 0) / p_cam.at<float>(2, 0) * fy + cy;
        //     cv::circle(img_2, cv::Point2f(u, v), 2, cv::Scalar(0, 255, 255));
        //     cv::circle(img_2, mCurrentFrame.mvKeysUn[i].pt, 2, cv::Scalar(255, 0, 255));
        //     cv::line(img_2, cv::Point2f(u, v), mCurrentFrame.mvKeysUn[i].pt, cv::Scalar(255, 255, 0));

        //     if (!set_anchor)
        //     {
        //         if (u > 50 && u < 400 && v > 50 && v < 400)
        //         {
        //             anchor_3d = p3d_w;
        //             set_anchor = true;
        //             std::cout << "set anchor ----------------------------------------------------------------------" << std::endl;
        //         }
        //     }
        // }
        // if (set_anchor)
        // {
        //     cv::Mat anchor_pc = mCurrentFrame.mRcw * anchor_3d + mCurrentFrame.mtcw;
        //     double u = anchor_pc.at<float>(0, 0) / anchor_pc.at<float>(2, 0) * fx + cx;
        //     double v = anchor_pc.at<float>(1, 0) / anchor_pc.at<float>(2, 0) * fy + cy;
        //     cv::drawMarker(img_2, cv::Point2f(u, v), cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 2);
        //     std::cout << "anchor 2d === " << u << "," << v << std::endl;
        //     std::cout << "anchor 3d === " << anchor_3d << std::endl;
        // }
        // cv::imshow("pnp_result_optimize_before", img_2);
        //////////////////////////////////////////////////////////////////////////////////////////////////////

        int inliers;
        if (!mpAtlas->isImuInitialized())
            Optimizer::PoseOptimization(&mCurrentFrame);
        else
        {
            if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU)
            {
                Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
                Optimizer::PoseOptimization(&mCurrentFrame);
            }
            else
            {
                // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
                if (!mbMapUpdated) //  && (mnMatchesInliers>30))
                {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                    inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                }
                else
                {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                    inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                }
            }
        }

        // /////////////////////////////绘制3D重投影结果////////////////////////////////
        // // cv::Mat img_2;
        // img_2.setTo(0);
        // cv::cvtColor(mCurrentFrame.mvImagePyramid[0], img_2, CV_GRAY2RGB);
        // for (size_t i = 0; i < mCurrentFrame.mvpMapPoints.size(); i++)
        // {
        //     if (mCurrentFrame.mvbOutlier[i])
        //     {
        //         continue;
        //     }
        //     MapPoint *p3d = mCurrentFrame.mvpMapPoints[i];
        //     cv::Mat p3d_w = p3d->GetWorldPos();
        //     cv::Mat p_cam = (mCurrentFrame.mRcw * p3d_w + mCurrentFrame.mtcw);
        //     double u = p_cam.at<float>(0, 0) / p_cam.at<float>(2, 0) * fx + cx;
        //     double v = p_cam.at<float>(1, 0) / p_cam.at<float>(2, 0) * fy + cy;
        //     cv::circle(img_2, cv::Point2f(u, v), 2, cv::Scalar(0, 255, 0));
        //     cv::circle(img_2, mCurrentFrame.mvKeysUn[i].pt, 2, cv::Scalar(0, 0, 255));
        //     cv::line(img_2, cv::Point2f(u, v), mCurrentFrame.mvKeysUn[i].pt, cv::Scalar(255, 0, 0));

        //     if (!set_anchor)
        //     {
        //         if (u > 50 && u < 400 && v > 50 && v < 400)
        //         {
        //             anchor_3d = p3d_w;
        //             set_anchor = true;
        //             std::cout << "set anchor ----------------------------------------------------------------------" << std::endl;
        //         }
        //     }
        // }
        // if (set_anchor)
        // {
        //     cv::Mat anchor_pc = mCurrentFrame.mRcw * anchor_3d + mCurrentFrame.mtcw;
        //     double u = anchor_pc.at<float>(0, 0) / anchor_pc.at<float>(2, 0) * fx + cx;
        //     double v = anchor_pc.at<float>(1, 0) / anchor_pc.at<float>(2, 0) * fy + cy;
        //     cv::drawMarker(img_2, cv::Point2f(u, v), cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 2);
        //     std::cout << "anchor 2d === " << u << "," << v << std::endl;
        //     std::cout << "anchor 3d === " << anchor_3d << std::endl;
        // }
        // cv::imshow("pnp_result_optimize", img_2);
        // //////////////////////////////////////////////////////////////////////////////////////////////////////
        aux1 = 0, aux2 = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i])
                    aux2++;
            }
        }
        std::cout << " 11111 track local map direct mState = " << mState << std::endl;

        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking)
                    {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        {
                            mnMatchesInliers++;
                        }
                    }
                    else
                    {
                        mnMatchesInliers++;
                    }
                }
                else if (mSensor == System::STEREO)
                {
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }
        }
        std::cout << "当前帧的内点数目　＝＝　" << mnMatchesInliers << std::endl;

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        mpLocalMapper->mnMatchesInliers = mnMatchesInliers;
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
        {
            return false;
        }

        if ((mnMatchesInliers > 10) && (mState == RECENTLY_LOST))
        {
            return true;
        }

        std::cout << " 11111 track local map direct mState = " << mState << std::endl;

        if (mSensor == System::IMU_MONOCULAR)
        {
            if (mnMatchesInliers < 15)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else if (mSensor == System::IMU_STEREO)
        {
            if (mnMatchesInliers < 15)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            if (mnMatchesInliers < 30)
            {
                std::cout << " failed track local map direct  ===  mnMatchesInliers = " << mnMatchesInliers << std::endl;
                std::cout << "mCurrentFrame.N == " << mCurrentFrame.N << std::endl;
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    bool Tracking::NeedNewKeyFrame()
    {
        //当前帧与上一个关键帧的时间戳大于0.25s, 大约为7帧间隔
        if (((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && !mpAtlas->GetCurrentMap()->isImuInitialized())
        {
            if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
                return true;
            else if (mSensor == System::IMU_STEREO && (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
                return true;
            else
                return false;
        }

        if (mbOnlyTracking)
        {
            return false;
        }

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        {
            return false;
        }

        // Return false if IMU is initialazing
        if (mpLocalMapper->IsInitializing())
        {
            return false;
        }
        const int nKFs = mpAtlas->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        //地图中的关键帧数大于给定的最大关键帧数
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        {
            return false;
        }

        // Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if (nKFs <= 2)
        {
            nMinObs = 2;
        }
        //参考关键帧跟踪的地图点的数
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        std::cout << " NeedNewKeyFrame ================================ " << mCurrentFrame.N << std::endl;
        // Local Mapping accept keyframes?
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose = 0;

        if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR)
        {
            int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
            for (int i = 0; i < N; i++)
            {
                if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose;
        bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Thresholds
        float thRefRatio = 0.75f;
        if (nKFs < 2)
            thRefRatio = 0.4f;

        if (mSensor == System::MONOCULAR)
            thRefRatio = 0.9f;

        if (mpCamera2)
            thRefRatio = 0.75f;

        if (mSensor == System::IMU_MONOCULAR)
        {
            if (mnMatchesInliers > 350) // Points tracked from the local map
            {
                thRefRatio = 0.75f;
            }
            else
            {
                thRefRatio = 0.90f;
            }
        }

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) && bLocalMappingIdle);
        //Condition 1c: tracking is weak
        const bool c1c = mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR && mSensor != System::IMU_STEREO && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) && mnMatchesInliers > 15);

        std::cout << "Need new keyframe   bLocalMappingIdle  == " << bLocalMappingIdle << std::endl;
        std::cout << "Need new keyframe   mCurrentFrame.mnId  == " << mCurrentFrame.mnId << std::endl;
        std::cout << "Need new keyframe   mnLastKeyFrameId == " << mnLastKeyFrameId << std::endl;
        std::cout << "Need new keyframe   mMaxFrames, mMinFrames == " << mMaxFrames << "," << mMinFrames << std::endl;
        std::cout << "Need new keyframe   mnMatchesInliers == " << mnMatchesInliers << std::endl;
        std::cout << "Need new keyframe   nRefMatches == " << nRefMatches << std::endl;
        std::cout << "Need new keyframe   thRefRatio == " << thRefRatio << std::endl;
        std::cout << "Nedd new keyframe nRefMatches * thRefRatio = " << nRefMatches * thRefRatio << std::endl;

        // Temporal condition for Inertial cases
        bool c3 = false;
        if (mpLastKeyFrame)
        {
            if (mSensor == System::IMU_MONOCULAR)
            {
                if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                    c3 = true;
            }
            else if (mSensor == System::IMU_STEREO)
            {
                if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                    c3 = true;
            }
        }

        bool c4 = false;
        if ((((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) || mState == RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR))) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
            c4 = true;
        else
            c4 = false;

        if (((c1a || c1b || c1c) && c2) || c3 || c4)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            if (bLocalMappingIdle)
            {
                return true;
            }
            else
            {
                mpLocalMapper->InterruptBA();
                if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR)
                {
                    if (mpLocalMapper->KeyframesInQueue() < 3)
                        return true;
                    else
                        return false;
                }
                else
                    return false;
            }
        }
        else
            return false;
    }

    void Tracking::CreateNewKeyFrame()
    {
        if (mpLocalMapper->IsInitializing())
            return;

        if (!mpLocalMapper->SetNotStop(true))
            return;

        // if we have not extracted features, we do feature extraction here and compute descripter
        if (mCurrentFrame.mbFeatureExtracted == false)
        {
            // this key frame is generated with direct tracking
            mCurrentFrame.ExtractFeatures();
            // update the map point cache ?
        }
        KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        if (mpAtlas->isImuInitialized())
            pKF->bImu = true;

        pKF->SetNewBias(mCurrentFrame.mImuBias);
        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        if (mpLastKeyFrame)
        {
            pKF->mPrevKF = mpLastKeyFrame;
            mpLastKeyFrame->mNextKF = pKF;
        }
        else
            Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

        // Reset preintegration from last KF (Create new object)
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        {
            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
        }

        if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
        {
            mCurrentFrame.UpdatePoseMatrices();
            // cout << "create new MPs" << endl;
            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            int maxPoint = 100;
            if (mSensor == System::IMU_STEREO)
                maxPoint = 100;

            vector<pair<float, int>> vDepthIdx;
            int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
            vDepthIdx.reserve(mCurrentFrame.N);
            for (int i = 0; i < N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)
                {
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty())
            {
                sort(vDepthIdx.begin(), vDepthIdx.end());

                int nPoints = 0;
                for (size_t j = 0; j < vDepthIdx.size(); j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1)
                    {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    if (bCreateNew)
                    {
                        cv::Mat x3D;

                        if (mCurrentFrame.Nleft == -1)
                        {
                            x3D = mCurrentFrame.UnprojectStereo(i);
                        }
                        else
                        {
                            x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                        }

                        MapPoint *pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                        pNewMP->AddObservation(pKF, i);

                        //Check if it is a stereo observation in order to not
                        //duplicate mappoints
                        if (mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0)
                        {
                            mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]] = pNewMP;
                            pNewMP->AddObservation(pKF, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                            pKF->AddMapPoint(pNewMP, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        }

                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpAtlas->AddMapPoint(pNewMP);
                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    }
                    else
                    {
                        nPoints++; // TODO check ???
                    }

                    if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint)
                    {
                        break;
                    }
                }

                Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
            }
        }

        mpLocalMapper->InsertKeyFrame(pKF);

        mpLocalMapper->SetNotStop(false);

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
        //cout  << "end creating new KF" << endl;
    }

    void Tracking::SearchLocalPoints()
    {
        // Do not search map points already matched
        for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
            {
                if (pMP->isBad())
                {
                    *vit = static_cast<MapPoint *>(NULL);
                }
                else
                {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                    pMP->mbTrackInViewR = false;
                }
            }
        }

        int nToMatch = 0;

        // Project points in frame and check its visibility
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;

            if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if (mCurrentFrame.isInFrustum(pMP, 0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
            }
            if (pMP->mbTrackInView)
            {
                mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
            }
        }

        if (nToMatch > 0)
        {
            ORBmatcher matcher(0.8);
            int th = 1;
            if (mSensor == System::RGBD)
                th = 3;
            if (mpAtlas->isImuInitialized())
            {
                if (mpAtlas->GetCurrentMap()->GetIniertialBA2())
                    th = 2;
                else
                    th = 3;
            }
            else if (!mpAtlas->isImuInitialized() && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))
            {
                th = 10;
            }

            // If the camera has been relocalised recently, perform a coarser search
            if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;

            if (mState == LOST || mState == RECENTLY_LOST) // Lost for less than 1 second
                th = 15;                                   // 15

            int cnt = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
            std::cout << "Search by projection returns " << cnt << endl;
        }
    }

    void Tracking::UpdateLocalMap()
    {
        // This is for visualization
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints()
    {
        mvpLocalMapPoints.clear();

        int count_pts = 0;

        for (vector<KeyFrame *>::const_reverse_iterator itKF = mvpLocalKeyFrames.rbegin(), itEndKF = mvpLocalKeyFrames.rend(); itKF != itEndKF; ++itKF)
        {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
            {

                MapPoint *pMP = *itMP;
                if (!pMP)
                    continue;
                if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;
                if (!pMP->isBad())
                {
                    count_pts++;
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }

        // 从last keyframe中也取一份
        for (MapPoint *mp : mpLastKeyFrame->GetMapPointMatches())
        {
            if (mp == nullptr || mp->isBad() || mp->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            mvpLocalMapPoints.push_back(mp);
        }
    }

    struct CmpByValue
    {
        bool operator()(const std::pair<KeyFrame *, int> &lhs, const std::pair<KeyFrame *, int> &rhs)
        {
            return lhs.second < rhs.second;
        }
    };

    void
    Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        //局部关键帧为当前帧或上一帧的3D点，能被观测到的关键帧　＋　附近关键帧的共视帧
        //局部关键帧的个数限制个数 80帧
        map<KeyFrame *, int> keyframeCounter;
        if (!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId < mnLastRelocFrameId + 2))
        {
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                {
                    if (!pMP->isBad())
                    {
                        const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                            keyframeCounter[it->first]++;
                    }
                    else
                    {
                        mCurrentFrame.mvpMapPoints[i] = NULL;
                    }
                }
            }
        }
        else
        {
            for (int i = 0; i < mLastFrame.N; i++)
            {
                // Using lastframe since current frame has not matches yet
                if (mLastFrame.mvpMapPoints[i])
                {
                    MapPoint *pMP = mLastFrame.mvpMapPoints[i];
                    if (!pMP)
                        continue;
                    if (!pMP->isBad())
                    {
                        const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                            keyframeCounter[it->first]++;
                    }
                    else
                    {
                        // MODIFICATION
                        mLastFrame.mvpMapPoints[i] = NULL;
                    }
                }
            }
        }

        if (keyframeCounter.empty())
            return;

        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
        {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            if (it->second > max)
            {
                max = it->second;
                pKFmax = pKF;
            }

            mvpLocalKeyFrames.push_back(pKF);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            // Limit the number of keyframes
            if (mvpLocalKeyFrames.size() > 80) // 80
                break;

            KeyFrame *pKF = *itKF;

            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
            {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad())
                {
                    if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
            {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad())
                {
                    if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame *pParent = pKF->GetParent();
            if (pParent)
            {
                if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // Add 10 last temporal KFs (mainly for IMU)
        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mvpLocalKeyFrames.size() < 80)
        {
            //cout << "CurrentKF: " << mCurrentFrame.mnId << endl;
            KeyFrame *tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

            const int Nd = 20;
            for (int i = 0; i < Nd; i++)
            {
                if (!tempKeyFrame)
                    break;
                //cout << "tempKF: " << tempKeyFrame << endl;
                if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(tempKeyFrame);
                    tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    tempKeyFrame = tempKeyFrame->mPrevKF;
                }
            }
        }

        if (pKFmax)
        {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    bool Tracking::Relocalization()
    {
        Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

        if (vpCandidateKFs.empty())
        {
            Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75, true);

        vector<MLPnPsolver *> vpMLPnPsolvers;
        vpMLPnPsolvers.resize(nKFs);

        vector<vector<MapPoint *>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates = 0;

        for (int i = 0; i < nKFs; i++)
        {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
                if (nmatches < 15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    MLPnPsolver *pSolver = new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991); //This solver needs at least 6 points
                    vpMLPnPsolvers[i] = pSolver;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        while (nCandidates > 0 && !bMatch)
        {
            for (int i = 0; i < nKFs; i++)
            {
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                MLPnPsolver *pSolver = vpMLPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint *> sFound;

                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++)
                    {
                        if (vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j] = NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if (nGood < 10)
                        continue;

                    for (int io = 0; io < mCurrentFrame.N; io++)
                        if (mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if (nGood < 50)
                    {
                        int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                        if (nadditional + nGood >= 50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if (nGood > 30 && nGood < 50)
                            {
                                sFound.clear();
                                for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                    if (mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                                // Final optimization
                                if (nGood + nadditional >= 50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for (int io = 0; io < mCurrentFrame.N; io++)
                                        if (mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                    }

                    // If the pose is supported by enough inliers stop ransacs and continue
                    if (nGood >= 50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if (!bMatch)
        {
            return false;
        }
        else
        {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            cout << "Relocalized!!" << endl;
            return true;
        }
    }

    void Tracking::Reset(bool bLocMap)
    {
        Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

        if (mpViewer)
        {
            mpViewer->RequestStop();
            while (!mpViewer->isStopped())
                usleep(3000);
        }

        // Reset Local Mapping
        if (!bLocMap)
        {
            Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
            mpLocalMapper->RequestReset();
            Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
        }

        // Reset Loop Closing
        Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
        mpLoopClosing->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear BoW Database
        Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
        mpKeyFrameDB->clear();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearAtlas();
        mpAtlas->CreateNewMap();
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
            mpAtlas->SetInertialSensor();
        mnInitialFrameId = 0;

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        if (mpInitializer)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
        }
        mbSetInit = false;

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();
        mCurrentFrame = Frame();
        mnLastRelocFrameId = 0;
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame *>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mvIniMatches.clear();

        mvpDirectMapPointsCache.clear();

        if (mpViewer)
            mpViewer->Release();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

    void Tracking::ResetActiveMap(bool bLocMap)
    {
        Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
        if (mpViewer)
        {
            mpViewer->RequestStop();
            while (!mpViewer->isStopped())
                usleep(3000);
        }

        Map *pMap = mpAtlas->GetCurrentMap();

        if (!bLocMap)
        {
            Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
            mpLocalMapper->RequestResetActiveMap(pMap);
            Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
        }

        // Reset Loop Closing
        Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
        mpLoopClosing->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear BoW Database
        Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
        mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearMap();

        //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
        //Frame::nNextId = mnLastInitFrameId;
        mnLastInitFrameId = Frame::nNextId;
        mnLastRelocFrameId = mnLastInitFrameId;
        mState = NO_IMAGES_YET; //NOT_INITIALIZED;

        if (mpInitializer)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
        }

        list<bool> lbLost;
        // lbLost.reserve(mlbLost.size());
        unsigned int index = mnFirstFrameId;
        cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
        for (Map *pMap : mpAtlas->GetAllMaps())
        {
            if (pMap->GetAllKeyFrames().size() > 0)
            {
                if (index > pMap->GetLowerKFID())
                    index = pMap->GetLowerKFID();
            }
        }

        //cout << "First Frame id: " << index << endl;
        int num_lost = 0;
        cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

        for (list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
        {
            if (index < mnInitialFrameId)
                lbLost.push_back(*ilbL);
            else
            {
                lbLost.push_back(true);
                num_lost += 1;
            }

            index++;
        }
        cout << num_lost << " Frames set to lost" << endl;

        mlbLost = lbLost;

        mnInitialFrameId = mCurrentFrame.mnId;
        mnLastRelocFrameId = mCurrentFrame.mnId;

        mCurrentFrame = Frame();
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame *>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mvIniMatches.clear();

        if (mpViewer)
            mpViewer->Release();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

    vector<MapPoint *> Tracking::GetLocalMapMPS()
    {
        return mvpLocalMapPoints;
    }

    void Tracking::ChangeCalibration(const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag)
    {
        mbOnlyTracking = flag;
    }

    void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame)
    {
        Map *pMap = pCurrentKeyFrame->GetMap();
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mlpReferences.begin();
        list<bool>::iterator lbL = mlbLost.begin();
        for (list<cv::Mat>::iterator lit = mlRelativeFramePoses.begin(), lend = mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            while (pKF->isBad())
            {
                pKF = pKF->GetParent();
            }

            if (pKF->GetMap() == pMap)
            {
                (*lit).rowRange(0, 3).col(3) = (*lit).rowRange(0, 3).col(3) * s;
            }
        }

        mLastBias = b;

        mpLastKeyFrame = pCurrentKeyFrame;

        mLastFrame.SetNewBias(mLastBias);
        mCurrentFrame.SetNewBias(mLastBias);

        cv::Mat Gz = (cv::Mat_<float>(3, 1) << 0, 0, -IMU::GRAVITY_VALUE);

        cv::Mat twb1;
        cv::Mat Rwb1;
        cv::Mat Vwb1;
        float t12;

        while (!mCurrentFrame.imuIsPreintegrated())
        {
            usleep(500);
        }

        if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
        {
            mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                          mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                          mLastFrame.mpLastKeyFrame->GetVelocity());
        }
        else
        {
            twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
            Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
            Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
            t12 = mLastFrame.mpImuPreintegrated->dT;

            mLastFrame.SetImuPoseVelocity(Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                          twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                          Vwb1 + Gz * t12 + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
        }

        if (mCurrentFrame.mpImuPreintegrated)
        {
            twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
            Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
            Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
            t12 = mCurrentFrame.mpImuPreintegrated->dT;

            mCurrentFrame.SetImuPoseVelocity(Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                             twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                             Vwb1 + Gz * t12 + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
        }

        mnFirstImuFrameId = mCurrentFrame.mnId;
    }

    cv::Mat Tracking::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
    {
        cv::Mat R1w = pKF1->GetRotation();
        cv::Mat t1w = pKF1->GetTranslation();
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();

        cv::Mat R12 = R1w * R2w.t();
        cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;

        cv::Mat t12x = Converter::tocvSkewMatrix(t12);

        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;

        return K1.t().inv() * t12x * R12 * K2.inv();
    }

    void Tracking::CreateNewMapPoints()
    {
        // Retrieve neighbor keyframes in covisibility graph
        const vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();

        ORBmatcher matcher(0.6, false);

        cv::Mat Rcw1 = mpLastKeyFrame->GetRotation();
        cv::Mat Rwc1 = Rcw1.t();
        cv::Mat tcw1 = mpLastKeyFrame->GetTranslation();
        cv::Mat Tcw1(3, 4, CV_32F);
        Rcw1.copyTo(Tcw1.colRange(0, 3));
        tcw1.copyTo(Tcw1.col(3));
        cv::Mat Ow1 = mpLastKeyFrame->GetCameraCenter();

        const float &fx1 = mpLastKeyFrame->fx;
        const float &fy1 = mpLastKeyFrame->fy;
        const float &cx1 = mpLastKeyFrame->cx;
        const float &cy1 = mpLastKeyFrame->cy;
        const float &invfx1 = mpLastKeyFrame->invfx;
        const float &invfy1 = mpLastKeyFrame->invfy;

        const float ratioFactor = 1.5f * mpLastKeyFrame->mfScaleFactor;

        int nnew = 0;

        // Search matches with epipolar restriction and triangulate
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF2 = vpKFs[i];
            if (pKF2 == mpLastKeyFrame)
                continue;

            // Check first that baseline is not too short
            cv::Mat Ow2 = pKF2->GetCameraCenter();
            cv::Mat vBaseline = Ow2 - Ow1;
            const float baseline = cv::norm(vBaseline);

            if ((mSensor != System::MONOCULAR) || (mSensor != System::IMU_MONOCULAR))
            {
                if (baseline < pKF2->mb)
                    continue;
            }
            else
            {
                const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
                const float ratioBaselineDepth = baseline / medianDepthKF2;

                if (ratioBaselineDepth < 0.01)
                    continue;
            }

            // Compute Fundamental Matrix
            cv::Mat F12 = ComputeF12(mpLastKeyFrame, pKF2);

            // Search matches that fullfil epipolar constraint
            vector<pair<size_t, size_t>> vMatchedIndices;
            matcher.SearchForTriangulation(mpLastKeyFrame, pKF2, F12, vMatchedIndices, false);

            cv::Mat Rcw2 = pKF2->GetRotation();
            cv::Mat Rwc2 = Rcw2.t();
            cv::Mat tcw2 = pKF2->GetTranslation();
            cv::Mat Tcw2(3, 4, CV_32F);
            Rcw2.copyTo(Tcw2.colRange(0, 3));
            tcw2.copyTo(Tcw2.col(3));

            const float &fx2 = pKF2->fx;
            const float &fy2 = pKF2->fy;
            const float &cx2 = pKF2->cx;
            const float &cy2 = pKF2->cy;
            const float &invfx2 = pKF2->invfx;
            const float &invfy2 = pKF2->invfy;

            // Triangulate each match
            const int nmatches = vMatchedIndices.size();
            for (int ikp = 0; ikp < nmatches; ikp++)
            {
                const int &idx1 = vMatchedIndices[ikp].first;
                const int &idx2 = vMatchedIndices[ikp].second;

                const cv::KeyPoint &kp1 = mpLastKeyFrame->mvKeysUn[idx1];
                const float kp1_ur = mpLastKeyFrame->mvuRight[idx1];
                bool bStereo1 = kp1_ur >= 0;

                const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                const float kp2_ur = pKF2->mvuRight[idx2];
                bool bStereo2 = kp2_ur >= 0;

                // Check parallax between rays
                cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
                cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0);

                cv::Mat ray1 = Rwc1 * xn1;
                cv::Mat ray2 = Rwc2 * xn2;
                const float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

                float cosParallaxStereo = cosParallaxRays + 1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;

                if (bStereo1)
                    cosParallaxStereo1 = cos(2 * atan2(mpLastKeyFrame->mb / 2, mpLastKeyFrame->mvDepth[idx1]));
                else if (bStereo2)
                    cosParallaxStereo2 = cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

                cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

                cv::Mat x3D;
                if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 && (bStereo1 || bStereo2 || cosParallaxRays < 0.9998))
                {
                    // Linear Triangulation Method
                    cv::Mat A(4, 4, CV_32F);
                    A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
                    A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
                    A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
                    A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

                    cv::Mat w, u, vt;
                    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

                    x3D = vt.row(3).t();

                    if (x3D.at<float>(3) == 0)
                        continue;

                    // Euclidean coordinates
                    x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
                }
                else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)
                {
                    x3D = mpLastKeyFrame->UnprojectStereo(idx1);
                }
                else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
                {
                    x3D = pKF2->UnprojectStereo(idx2);
                }
                else
                    continue; //No stereo and very low parallax

                cv::Mat x3Dt = x3D.t();

                //Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
                if (z1 <= 0)
                    continue;

                float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
                if (z2 <= 0)
                    continue;

                //Check reprojection error in first keyframe
                const float &sigmaSquare1 = mpLastKeyFrame->mvLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
                const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
                const float invz1 = 1.0 / z1;

                if (!bStereo1)
                {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                        continue;
                }
                else
                {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    float u1_r = u1 - mpLastKeyFrame->mbf * invz1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8 * sigmaSquare1)
                        continue;
                }

                //Check reprojection error in second keyframe
                const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
                const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
                const float invz2 = 1.0 / z2;
                if (!bStereo2)
                {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                        continue;
                }
                else
                {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float u2_r = u2 - mpLastKeyFrame->mbf * invz2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8 * sigmaSquare2)
                        continue;
                }

                //Check scale consistency
                cv::Mat normal1 = x3D - Ow1;
                float dist1 = cv::norm(normal1);

                cv::Mat normal2 = x3D - Ow2;
                float dist2 = cv::norm(normal2);

                if (dist1 == 0 || dist2 == 0)
                    continue;

                const float ratioDist = dist2 / dist1;
                const float ratioOctave = mpLastKeyFrame->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];

                if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
                    continue;

                // Triangulation is succesfull
                MapPoint *pMP = new MapPoint(x3D, mpLastKeyFrame, mpAtlas->GetCurrentMap());

                pMP->AddObservation(mpLastKeyFrame, idx1);
                pMP->AddObservation(pKF2, idx2);

                mpLastKeyFrame->AddMapPoint(pMP, idx1);
                pKF2->AddMapPoint(pMP, idx2);

                pMP->ComputeDistinctiveDescriptors();

                pMP->UpdateNormalAndDepth();

                mpAtlas->AddMapPoint(pMP);
                nnew++;
            }
        }
        TrackReferenceKeyFrame();
    }

    void Tracking::NewDataset()
    {
        mnNumDataset++;
    }

    int Tracking::GetNumberDataset()
    {
        return mnNumDataset;
    }

    int Tracking::GetMatchesInliers()
    {
        return mnMatchesInliers;
    }

} // namespace ORB_SLAM3
