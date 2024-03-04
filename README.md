# ROS tf2使用教程

本项目主要介绍ros1和ros2中tf2的基本使用方法(发布静态和动态tf，坐标变换)，以及使用ros库函数将四元数和欧拉角相互转换。tf2是tf的升级版本，支持ros1和ros2，tf2主要由ROS官方的tf2和tf2_ros两个功能包组成。

**原理概述: **

ros将坐标变换系统构建为树结构（数据结构中的概念），树的节点称作坐标系tf，每个tf节点都必须有父节点（除根节点外），ros系统维护了整棵树，对外提供坐标变换服务。用户只需要发布tf节点即可，ros系统自动完成了树节点的插入和删除，用户只需要关注tf节点的发布即可。

本项目在ros1 noetic下构建，并使用官方的turtlesim小乌龟功能包作为演示。

## 发布静态tf
依赖安装：

```shell
sudo apt-get install ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-turtle-tf2
```

新建功能包learning_tf2

```shell
catkin_create_pkg learning_tf2 tf2 tf2_ros turtlesim roscpp
```

新建cpp文件 src/static_turtle_tf2_broadcaster.cpp，这里只写发布的核心代码，项目完整工程请参阅github仓库，链接见文末。

```cpp
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h> //tf2静态广播tf

#include <cstdio>

std::string static_turtle_name;

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_static_tf2_broadcaster");
    //核心代码
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    //数据填充: 父坐标、子坐标、相对position、相对rotation
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "turtle2";
    //发送静态tf
    static_broadcaster.sendTransform(static_transformStamped);
    
    ros::spin();
    return 0;
};
```

cmakelists.txt

```cmake
add_executable(tf2_sbc src/static_turtle_tf2_broadcaster.cpp)
target_link_libraries(tf2_sbc
  ${catkin_LIBRARIES}
)
```

## 发布动态tf

创建cpp源文件，src/turtle_tf2_broadcaster.cpp，下面写出关键代码。

```c++
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h> //tf2动态广播tf

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_tf2_broadcaster");
    
  //核心代码
  static tf2_ros::TransformBroadcaster br;
  //填充数据
  geometry_msgs::TransformStamped transformStamped;
  // 发布tf
  br.sendTransform(transformStamped);
  
  ros::spin();
  return 0;
};
```

总结：与静态tf基本一致，唯一不同是使用的发布者不一样。

## 坐标变换

基本步骤：
1. 创建tf2_ros::Buffer对象，用来收集tf节点数据
2. 创建tf2_ros::TransformListener对象，用来监听tf节点数据
3. 在tfBuffer中查询坐标变换

创建cpp源文件，src/turtle_tf2_listener.cpp，下面给出关键代码。
```cpp
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>  //tf2中tf监听器和转换器一起使用，需要把tf数据缓存起来，然后进行坐标变换

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf2_listener");

    tf2_ros::Buffer tfBuffer; //存储tf数据
    tf2_ros::TransformListener tfListener(tfBuffer);  // 监听记录所有的tf数据
    geometry_msgs::TransformStamped transformStamped;
    try {
      //查询turtle1在turtle2的相对坐标。
      transformStamped = tfBuffer.lookupTransform(
          "turtle2", "turtle1",
          ros::Time(0));  // turtle2是父坐标，turtle1是子坐标
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    return 0;
};
```
这是官方提供的一种方式，还有其他的方式，以后遇到了补充。

如何将激光雷达的点云坐标转换到自车坐标系下面，也通过tf可以实现，后面介绍。

## 欧拉角与四元数相互转换

创建cpp源文件，src/v2q_q2v.cpp，给出关键代码。

```c++
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h> //四元数转欧拉角
#include <tf2/LinearMath/Quaternion.h> //欧拉角转四元数

int main(int argc, char **argv) {
  ros::init(argc, argv, "v2q_q2v");
  double roll = 0, pitch = 0, yaw = M_PI_2;
  // 欧拉角转四元数
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  std::cout << "欧拉角(0,0,pi/2)转四元数为(x,y,z,w): "
            << "(" << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", "
            << q.getW() << ")" << std::endl;

  // 四元数转欧拉角
  tf2::Matrix3x3 R(q);
  roll = 1, pitch = 2, yaw = 3;
  R.getRPY(roll, pitch, yaw);
  std::cout << "四元数转欧拉角为(roll,pitch,yaw): "
            << "(" << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
  return 0;
}
```



## 本仓库使用方法

本仓库地址: https://github.com/ladissonlai/learning_tf2.git

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ladissonlai/learning_tf2.git
cd ..
catkin_make
source devel/setup.bash
roslaunch learning_tf2 start_demo.launch # 乌龟跟随案例
# rosrun learning_tf2 v2q_q2v 四元数和欧拉角的相互转化
```

## 参考链接

- http://wiki.ros.org/tf2/Tutorials
- https://wiki.ros.org/tf2/Tutorials/Introduction%20to%20tf2
- https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29