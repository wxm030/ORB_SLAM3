运行指令
1. mono版本
   1.1 orb3
/home/wxm/Documents/code/ORB_SLAM3/Examples/Monocular/mono_euroc  "/home/wxm/sda2/codes/ORB_SLAM3/Vocabulary/ORBvoc.txt"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data/EuRoc2.yaml"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data/mav0/timestamps.txt" "dataset-Huawei_mono"  >log.txt

  1.2 YGZ-SLAM
"/home/wxm/Documents/code/ORB-YGZ-SLAM/Examples/Monocular/mono_euroc"   "/home/wxm/Documents/code/ORB-YGZ-SLAM/Vocabulary/ORBvoc.bin"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data/EuRoc.yaml"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data/mav0/cam0/data"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data/mav0/timestamps.txt"



2. rgbd_version
   2.1 orb3
/home/wxm/Documents/code/ORB_SLAM3/Examples/RGB-D/rgbd_tum   /home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin    /home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/meizu_bag_rgbd_data_60rgb_30depth/TUM1.yaml     /home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/meizu_bag_rgbd_data_60rgb_30depth/mav0/    /home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/meizu_bag_rgbd_data_60rgb_30depth/mav0/timestamp_sync.txt

2.2 YGZ-SLAM

/home/wxm/Documents/code/ORB-YGZ-SLAM/Examples/RGB-D/rgbd_tum  "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin" "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/meizu_bag_rgbd_data_60rgb_30depth/TUM1_ygz.yaml" "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/meizu_bag_rgbd_data_60rgb_30depth/mav0/" "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/meizu_bag_rgbd_data_60rgb_30depth/mav0/timestamp_sync.txt" >log_ygz.txt


3. stereo版本
3.1 orb3
/home/wxm/Documents/code/ORB_SLAM3/Examples/Stereo/stereo_euroc  "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/mav0/EuRoC.yaml" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/" "/home/wxm/Documents/code/ORB_SLAM3/Examples/Stereo/EuRoC_TimeStamps/V101.txt" >log.txt
3.2 YGZ-SLAM
/home/wxm/Documents/code/ORB-YGZ-SLAM/Examples/Stereo/stereo_euroc "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin" "/home/wxm/Documents/code/ORB_SLAM3/Examples/Stereo/EuRoC.yaml" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/mav0/cam0/data" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/mav0/cam1/data" "/home/wxm/Documents/code/ORB_SLAM3/Examples/Stereo/EuRoC_TimeStamps/V101.txt" >log_ygz.txt


4.mono_vi版本
4.1 orb3

mono_V101
/home/wxm/Documents/code/ORB_SLAM3/Examples/Monocular/mono_euroc  "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin"  "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/mav0/EuRoC_mono_imu.yaml" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/" "/home/wxm/Documents/code/ORB_SLAM3/Examples/Stereo/EuRoC_TimeStamps/V101.txt" "dataset-V101_mono"  >log_mono.txt

MONO_vi_V101
/home/wxm/Documents/code/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/mav0/EuRoC_mono_imu.yaml" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/" "/home/wxm/Documents/code/ORB_SLAM3/Examples/Stereo/EuRoC_TimeStamps/V101.txt" >log.txt

4.2 YGZ_SLAM
/home/wxm/Documents/code/ORB-YGZ-SLAM/Examples/Monocular/mono_euroc_vins "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/mav0/EuRoC_mono_imu_ygz.yaml" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/mav0/cam0/data" "/home/wxm/Documents/code/ORB_SLAM3/Examples/Stereo/EuRoC_TimeStamps/V101.txt" "/home/wxm/sda2/codes/Kimera-VIO/data/V1_01_easy/mav0/imu0/data.csv" >log_vi_ygz.txt


5.同一个数据集测试单目和单目VI的铆点稳定性
单目
/home/wxm/Documents/code/ORB_SLAM3/Examples/Monocular/mono_euroc  "/home/wxm/sda2/codes/ORB_SLAM3/Vocabulary/ORBvoc.txt"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data/EuRoc2.yaml"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data/mav0/timestamps.txt" "dataset-Huawei_mono"  >log.txt
单目＋IMU
/home/wxm/Documents/code/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc  "/home/wxm/sda2/codes/ORB_SLAM3/Vocabulary/ORBvoc.txt"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data/EuRoc2.yaml"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data"  "/home/wxm/sda2/codes/ORB_SLAM3/data/huawei_bag/Huawei_bag_data/mav0/timestamps.txt" >log_vi.txt


reno_vi_data
/home/wxm/Documents/code/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin" "/home/wxm/Documents/code/ORB_SLAM3/data/config/EuRoc2.yaml" "/home/wxm/Documents/code/ORB_SLAM3/data/06_05_15_43_Circle/" "/home/wxm/Documents/code/ORB_SLAM3/data/06_05_15_43_Circle/mav0/timestamps.txt"

/home/wxm/Documents/code/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin" "/home/wxm/Documents/code/ORB_SLAM3/data/config/EuRoc2.yaml" "/home/wxm/Documents/code/ORB_SLAM3/data/06_05_15_20_Square/" "/home/wxm/Documents/code/ORB_SLAM3/data/06_05_15_20_Square/mav0/timestamps.txt"











meiz_r17_vi_data
/home/wxm/Documents/code/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin" "/home/wxm/Documents/code/ORB_SLAM3/data/ZJU_3DV_VI_data/orb3_mono_imu_config_A.yaml" "/home/wxm/Documents/code/ORB_SLAM3/data/ZJU_3DV_VI_data/A1/" "/home/wxm/Documents/code/ORB_SLAM3/data/ZJU_3DV_VI_data/A1/mav0/timestamps.txt"

iphoneX
/home/wxm/Documents/code/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc "/home/wxm/Documents/code/ORB_SLAM3/Vocabulary/ORBvoc.bin" "/home/wxm/Documents/code/ORB_SLAM3/data/ZJU_3DV_VI_data/orb3_mono_imu_config_B.yaml" "/home/wxm/Documents/code/ORB_SLAM3/data/ZJU_3DV_VI_data/B2/" "/home/wxm/Documents/code/ORB_SLAM3/data/ZJU_3DV_VI_data/B2/mav0/timestamps.txt"


####################################测试的一些结论#######################################
1. 选取一个地图点作为铆点．绘制在2D图像上，观察铆点的移动情况．
  (1) 铆点是一个地图点的指针，跟着地图的更新而更新．在测试中，发现地图点在修正尺度时会修正，但在优化过程中，基本不会做明显修正．说明优化过程对3D点的调整比较小．
  (2) 铆点在测试过程中，会发生明显的漂移．由于观察到地图点在漂移过程中，基本没有改变，因此可以证明是当前帧的位姿估计不准确导致的漂移．
  (3) 当铆点位于中心区域时，位姿会和旧的位姿估计比较一致，当铆点快要移出视野时，铆点偏移比较明显．偏移呈现缓慢移动趋势．
  (4) 相机左右平移运动时，相邻帧提取的特征点位置不一致．很可能因此SearchByProjection()函数中3D点找到错误的2D点．从而导致位姿计算有偏移．
  (5)同一个数据集，有RGBD版本的铆点比较稳定．单目的版本会明显偏移．
  (6) 直接法的单目偏移比原始ORBSLAM3更明显

2. 





7. monno-vi版本的尺度不能快速收敛的原因分析及解决: 初步定位问题在于bias需要初步估计，且其权重不能固定．
8. RS相机模型支持
9. timeshift支持－－已支持
10. 是否可以在线标定相机参数
11. 原来的orb初始化模块还是需要对单目初始化过程中提取特征点5000,　其余帧提取1000个．　－－待修改回来
12. 光流初始化最好网格化特征点，太密的点反而效果不好．
13. 转向时容易丢失：　初步分析是由于，目前在转向时，由于关键帧没有及时添加进去，导致视觉丢失，这里其实可以利用IMU积分的pose作为估计pose，然后这个过程中也需要进行三角化．－－待办　　关键帧没有及时添加进去，检查LocalMapping线程是否阻塞？---localMapping线程阻塞，计算太慢,需要优化加速．另外加速之火，可能需要根据角速度达到一定的阈值之后也要增加关键帧．


1. 待办：　可能需要修改计算位姿的结构
分情形计算位姿（主要考虑快速转向＋快速震动情形，正常的移动情形）：
(1) 纯单目：　ORB描述子比较鲁棒trackReferenceFrame, 初始化前两帧svoｕ有时候会明显快一些．
(2) IMU初始化刚完成：trackReferenceFrame + TrackLocalMap
(3) imu+cam 出位姿： trackLocalMap失败，则直接用IMU的pose，不要直接重启．
(4)弄清楚状态发生的条件：  Lost: 视觉丢失且IMU达到一定时间．且重定位没有成功．
                          RecentLost: 视觉丢失（TrackLocalMap内点小于15), imu还能积分，且短时间．尝试重定位过程．
                          OK:
(5) 失败检测条件： 

　　　　　　　　　　TrackLocalMap内点数小于阈值则视觉丢失；
　　　　　　　　　　IMU独自积分时间大于阈值则系统丢失；
　　　　　　　　　　重定位啥时候进行？　是Lost还是RecentLost?
                   
(6) 封装slam接口，返回pose, slam状态，和点云（待定）
(7) svo容易丢失的原因分析，加在哪个地方更合理？svo位姿计算不准确的原因分析，有时候会漂移．
(8) orb3安卓移植
(9) meshroom的mesh生成模块，太虚ARdemo




#单目＋imu的API运行：
/home/wxm/Documents/code/ORB_SLAM3/lib/test_api_node /home/wxm/Documents/code/ORB_SLAM3/lib/input_config.yaml










