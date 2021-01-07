# FAST_ORB_SLAM3
在ORB3的基础上增加svo的直接法

# TODO
## mono version
- 畸变处理，对整张图去畸变再提取特征点？还是先提取特征点，再对特征点去畸变．
- cv::Mat 改为se3f（暂不修改，mat不会有Eigen的对齐问题）
- 图像金字塔现在为５，　改为８之后，trackLocalMapDirect的点数特别少，可能有问题．
- change the sophus lib into the templated version
- 将double类型的数据改成float，因为用不到那么高的精度。但g2o里内部用的double，没法改，所以只能cast一下。
- 将align中的SSE替换成普通的align，对光照效果更好一些。SSE实现的有点问题。
- 修掉了原版ORB在初始化创建地图点后，未更新map point的min distance, max distance，导致Frame::InFrustum判定一直失败的问题。
- 修正了keyframe culling时无法删除的问题（纯视觉中）。这是由于loop closing在detectloop的时候，将keyframe的mbNotErase设置为真导致的。
- 特征提取部分增加了变网格/变阈值的FAST，现在提取更加稳定，且不容易有强行提的特征点
- 将位姿有关的计算替换成Sophus::SE3，修改了Thirdparty/g2o中的内容
- 全面Vector3d化，现在只有loop closing部分仍使用cv::Mat,但那一部分计算量不大，于是先保留不动了。
- 将Frame, MapPoint, KeyFrame中有关代数部分，从cv::Mat修改成Sophus::SE3和eigen::Vector
- 将字典替换成ORBvoc.bin，加载更快速


- 结论待验证：直接法的框架鲁棒性确实不如原版ORB，一大原因是，Track Local Map Direct是直接依赖上一步估计的位姿的，假设地图点投影误差在几个像素以内，这件事情不是总能够保证的。然而，如果有特征点，那么总可以通过计算BoW匹配，从地图点中找到合适的匹配，而无需假设位姿已知。


## mono-imu version
- mono-vi版本要从配置文件读入imu_bias.
- mono-vi版本的imu初始化过程调试，固定seed保证每次运行结果一致．
- monno-vi版本的尺度不能快速收敛的原因分析及解决．

# 日志
## 2020.1.4

