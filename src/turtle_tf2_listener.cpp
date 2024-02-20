#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>  //tf2中tf监听器和转换器一起使用，需要把tf数据缓存起来，然后进行坐标变换
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  // 服务的调用方法，快速回忆起来
  ros::service::waitForService("spawn");
  ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "turtle2";
  spawner.call(turtle);

  ros::Publisher turtle_vel =
      node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);  // 监听记录所有的tf数据

  ros::Rate rate(20.0);
  while (node.ok()) {
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer.lookupTransform(
          "turtle2", "turtle1",
          ros::Time(0));  // turtle2是父坐标，turtle1是子坐标
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x =
        0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                   pow(transformStamped.transform.translation.y, 2));

    if (fabs(vel_msg.linear.x) < 0.01) {
      // 把四元数转为欧拉角
      geometry_msgs::Quaternion quat = transformStamped.transform.rotation;
      tf2::Quaternion quaternion(quat.x, quat.y, quat.z, quat.w);
      tf2::Matrix3x3 matrix(quaternion);
      double roll, pitch, yaw;
      matrix.getRPY(roll, pitch, yaw);
      //   vel_msg.angular.z = tf2Radians(yaw) * 4;
      if (fabs(yaw) < 0.1) {
        vel_msg.angular.z = 0;
      } else {
        vel_msg.angular.z = yaw;
      }

    } else {
      vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                      transformStamped.transform.translation.x);
    }
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};