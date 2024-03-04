# tf2命令行基本使用教程

这里提供基础的tf2使用方法，包括命令行工具和launch代码。

## 查看tf树的基本用法

```
# 查看tf树
rosrun rqt_tf_tree rqt_tf_tree 
# 保存tf树为pdf文件
rosrun tf2_tools view_frames.py
# 使用命令行查询两个frame之间的变换关系（很方便，不用写代码编译）
rosrun tf tf_echo [reference_frame] [target_frame]
# 查看功能包的路径
rospack find [package_name]
```

## ROS编译的常用方法

```shell
# 编译指定功能包
catkin_make -DCATKIN_WHITELIST_PACKAGES="package_name1, package_name2”
# 但是如再次使用catkin_make编译所有功能包时会出现仅仅只编译上次设置的单独功能包，如果想要再次使用catkin_make编译所有功能包，需要执行：
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

## 发布静态tf

除了使用代码，更常用的一种方式是使用roslaunch。

```xml
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 link1_parent link1" />
</launch>
```

其中, args参数为位置和旋转，单位是米和弧度。旋转也可以将欧拉角换成四元数，具体含义如下。

```shell
# 官方tf2_ros库提供的可执行文件调用方法
static_transform_publisher x y z yaw(z) pitch(y) roll(x) frame_id child_frame_id
static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
```

## tf2工具包安装

```shell
sudo apt-get install ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-tf
```

