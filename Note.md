# FAST_ORB_SLAM3
在ORB3的基础上增加svo的直接法

# TODO
## mono version
- 待办
- 增加算法评估代码，利于分析算法模块的性能
- 调试RGBD版本(可以从配置文件读入depth的外参，算法中进行D2C)
- 鱼眼模型调试
- 畸变处理，高翔用的对整张图去畸变，svo和原版orb都用的对点去畸变,如果要对整张图去畸变，那么增加畸变表映射的方法．
- 去除代码中的warning
- 调试双目版本
- 调试imu-mono版本
- 移植到手机app上．先类似ORB-Android2的demo.
- 封装接口，算法返回每一帧的位姿，slam运行状态，（深度状态）等信息．
- meshroom作为另一个demo，移植到手机上．类似太虚AR.
- 稀疏平面检测移植．AR尺子demo.深度版本
- 支持rs相机



- 暂时不改
- 矩阵格式，用Eigen ? SE3? SE3f ? Mat?
- 将double类型的数据改成float，因为用不到那么高的精度。但g2o里内部用的double，没法改，所以只能cast一下。/////先不改



- cv::Mat 改为se3f（暂不修改，mat不会有Eigen的对齐问题）
- change the sophus lib into the templated version
- 将double类型的数据改成float，因为用不到那么高的精度。但g2o里内部用的double，没法改，所以只能cast一下。
- 修掉了原版ORB在初始化创建地图点后，未更新map point的min distance, max distance，导致Frame::InFrustum判定一直失败的问题。
- 修正了keyframe culling时无法删除的问题（纯视觉中）。这是由于loop closing在detectloop的时候，将keyframe的mbNotErase设置为真导致的。
- 特征提取部分增加了变网格/变阈值的FAST，现在提取更加稳定，且不容易有强行提的特征点
- 将位姿有关的计算替换成Sophus::SE3，修改了Thirdparty/g2o中的内容
- 全面Vector3d化，现在只有loop closing部分仍使用cv::Mat,但那一部分计算量不大，于是先保留不动了。
- 将Frame, MapPoint, KeyFrame中有关代数部分，从cv::Mat修改成Sophus::SE3和eigen::Vector



- 结论：直接法的框架鲁棒性确实不如原版ORB，一大原因是，Track Local Map Direct是直接依赖上一步估计的位姿的，假设地图点投影误差在几个像素以内，这件事情不是总能够保证的。然而，如果有特征点，那么总可以通过计算BoW匹配，从地图点中找到合适的匹配，而无需假设位姿已知。


## mono-imu version
- mono-vi版本的imu初始化过程调试，固定seed保证每次运行结果一致．
- monno-vi版本的尺度不能快速收敛的原因分析及解决．
- 初始化优化前先进行初始值估计


# 日志
## 2020.1.13
- 将svo的前端移植到orb-slam3中，前端加速．
- 显示部分增加铆点以及重投影前后的特征点显示，能更方便看到位姿的估计准确性．
- 替换原有的Dbow库，增加Fast库．这两个库都来自ORB-YGZ-SLAM.
- 测试mono, mono_vi, rgbd版本，可以正常运行．
## 2020.1.15
- 增加输出各个模块的log信息，详细分析算法的有效程度以及漂移的产生来源．
- imu初始化过程中的状态量可视化，便于分析
- 

