## oil_filler_pose

该ros仓库的作用是对加油口进行检测，主要有两种方法：
1. 仅使用原始点云来计算加油口位姿
2. 先使用三维重建获得完整点云，再利用完整点云计算加油口位姿

## 环境
1. ros melodic
2. opencv
3. pcl
4. cuda

## 运行
```bash
# 不使用三维重建
roslaunch oil_pose_detect detect_realsence.launch # d435i相机
roslaunch oil_pose_detect detect_tuyang.launch # 图漾相机

# 使用三维重建
roslaunch oil_pose_detect detect_reconstruct_realsence.launch # d435i相机
roslaunch oil_pose_detect detect_reconstruct_tuyang.launch # 图漾相机
```
> 关键参数是与相机相关的话题名称