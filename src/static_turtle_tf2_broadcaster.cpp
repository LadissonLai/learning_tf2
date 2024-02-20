#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h> //tf2静态广播tf

#include <cstdio>

std::string static_turtle_name;

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_static_tf2_broadcaster");
  if (argc != 8) {
    ROS_ERROR(
        "Invalid number of parameters\nusage: static_turtle_tf2_broadcaster "
        "child_frame_name x y z roll pitch yaw");
    return -1;
  }
  if (strcmp(argv[1], "world") == 0) {
    ROS_ERROR("Your static turtle name cannot be 'world'");
    return -1;
  }
  static_turtle_name = argv[1];
  // ROS_INFO("打印参数列表："); //不支持中文
  std::cout << "打印参数列表：" << std::endl;
  for (int i = 0; i < argc; i++) {
    ROS_INFO("argv[%d] = %s", i, argv[i]);
  }
  //核心代码
  {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped; //数据
    //数据填充
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = static_turtle_name;
    static_transformStamped.transform.translation.x = atof(argv[2]);
    static_transformStamped.transform.translation.y = atof(argv[3]);
    static_transformStamped.transform.translation.z = atof(argv[4]);
    tf2::Quaternion quat; //这里讲欧拉角转成了四元数，常用的方法
    quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
    ROS_INFO("Spinning until killed publishing %s to world",
             static_turtle_name.c_str());
  }

  ros::spin();
  return 0;
};

//总结：只要这个节点存在，那么新的节点如果查询tf树，那么就算该节点没有重新发tf关系，也会收到该静态tf数据。
//发布静态tf，关键在于填充，geometry_msgs::TransformStamped