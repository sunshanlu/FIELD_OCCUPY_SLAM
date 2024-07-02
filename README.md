# Field_Occupy_SLAM

## 1. 介绍

基于似然场配准和栅格地图的2d-SLAM，并实现基于子地图的金字塔式回环闭合策略。参考高翔的[SADBook](https://github.com/gaoxiang12/slam_in_autonomous_driving)的第六章内容，本项目从头到尾复现了第6章的2d-SLAM内容。



1. 使用线程分离策略，将跟踪、可视化和回环闭合分为三个线程独立运行。
2. 测试了图像的双线性差值对模型优化的贡献，发现模型优化误差并没有因为差值而降低，因此该项目中并没有使用图像的双线性差值来获取浮点数部分的图像灰度。
3. 实现了地图的加载和保存，并将所有的超参数提取，以yaml文件的形式进行保存，可以更轻松的调整超参数，而不是修改源码或者使用gflag的方式。
4. 实现了重定位功能和仅跟踪模式，针对当前测试的数据包，重定位功能比较苛刻，因此会出现重定位失败的情况，如果有更加巧妙的重定位策略，欢迎提出issue。


## 2. 依赖
1. C++17
2. ROS-FOXY
3. [OpenCV 3.4.16](https://github.com/opencv/opencv/releases/tag/3.4.16)
4. [g2o_20201223](https://github.com/RainerKuemmerle/g2o/releases/tag/20201223_git)
5. Eigen3
6. yaml-cpp
7. Sophus
8. TBB

## 3. 编译
```shell
source /opt/ros/foxy/setup.bash
git clone https://github.com/sunshanlu/Field_Occupy_SLAM.git
cd Field_Occupy_SLAM
mkdir build && cd build
cmake ..
make -j8
```

## 4. 运行
根据需求，去配置项目参数文件，配置文件的示例在项目目录下的`config.yaml`中。
```yaml
---
laser:
    max_range: 15.0     # 激光雷达最大探测距离
    delta_angle: 30.0   # 激光雷达在原有扫描基础上减少的角度

submap:
    width: 1000         # 子地图宽度(px)
    height: 1000        # 子地图高度(px)
    resolution: 20      # 子地图分辨率(px/m)
    robot_width: 0.5    # 机器人宽度(m, x轴方向)
    robot_height: 1.0   # 机器人高度(m, y轴方向)
    method: 0           # 0: 代表bresenham栅格化，1: 代表模版栅格化
    occupy_range: 200   # 模版栅格化范围
    field_range: 20     # likelihood模版范围

tracking:
    keyframe_pos_th: 0.3 # 单位：m
    keyframe_ang_th: 15  # 单位：度
    keyframe_num_th: 50  # 子地图最大承载量
    range_th: 15         # scan的长度阈值
    motion_guss: True    # 是否使用运动模型
    reloc_th: 0.06       # 如果匹配内点结果小于0.1，进行重定位
    only_track: False    # 是否仅跟踪模式

viewer:
    use_viewer: True   # 是否使用可视化
    gp_max_size: 500   # 全局地图尺寸

loopcloser:
    use_loopcloser: True    # 是否使用闭环检测
    layer_num: 4            # 金字塔层级
    layer_ratio: 2.0        # 金字塔缩放比例
    inlier_ratio: 0.5       # 匹配内点比例
    error_th:               # 每一层误差阈值，单位为m
        - 0.8
        - 0.6
        - 0.3
        - 0.2
    distance_th: 15.0       # 回环闭合候选距离阈值
    submap_gap: 1           # 回环闭合候选子地图相隔距离
    loop_rk_delta: 1.0      # 位姿图优化核函数delta值

map:
    save_map: False
    load_map: False
    map_fp: "/home/rookie-lu/Project/Field_Occupy_SLAM/map"     
```

为了方便测试和展示效果，在项目的`example`目录下提供了一个数据包的使用方法，主要是使用`System`类完成雷达帧数据的接收和SLAM工作。读者可以按照`example`中的示例来使用本项目中的内容。

```shell
source /opt/ros/foxy/setup.bash
./bin/run_mapping_2d config_path bag_path
```

1. `config_path`： 配置文件路径
2. `bag_path`： 数据包路径

数据包的下载地址见[SADBook](https://github.com/gaoxiang12/slam_in_autonomous_driving)中的2dmapping数据。至于如何将rosbag1的数据包转换为rosbag2的问题，就留给读者们自行解决。