/**
 * @file v2q_q2v.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-03-04
 * 四元数转欧拉角，欧拉角转四元数例子。
 * @copyright Copyright (c) 2024
 *
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>   //四元数转欧拉角
#include <tf2/LinearMath/Quaternion.h>  // 欧拉角转四元数

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
