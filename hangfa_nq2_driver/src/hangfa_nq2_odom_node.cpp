#include <ros/ros.h>
#include <ros/time.h>
#include <boost/assign.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "CRobot.h"

#define PI 3.141592653589793

class odom_node {
public:
  odom_node();
  // ~robot_node();

  void odomPublish();

  // config parameters
  std::string odom_port;
  bool odom_tf_publish_flag;
  std::string odom_id;
  std::string child_id;
  int pub_rate;

protected:
  // data variables
  double robot_x;
  double robot_y;
  double robot_yaw;
  double robot_vx;
  double robot_vy;
  double robot_yaw_v;

private:
  ros::NodeHandle* m_nh;

  ros::Publisher m_odom_pub;

  tf::TransformBroadcaster m_odom_broadcaster;
  geometry_msgs::TransformStamped m_odom_trans;
  nav_msgs::Odometry m_odom_msg;
  CRobot *m_pRobot;
};

odom_node::odom_node() {
  std::vector<double> pose_cov_diag(6, 1e-6);
  std::vector<double> twist_cov_diag(6, 1e-6);

  m_nh = new ros::NodeHandle("~");
  m_nh->param<std::string>("~odom_port", odom_port, "/dev/ttyACM0");
  m_nh->param<std::string>("~odom_id", odom_id, "odom");
  m_nh->param<std::string>("~child_id", child_id, "base_link");
  m_nh->param<int>("~pub_rate", pub_rate, 30);
  m_nh->param<bool>("~tf_publish_flag", odom_tf_publish_flag, true);

  if (m_nh->hasParam("~pose_covariance_diagonal")) {
    XmlRpc::XmlRpcValue pose_cov_list;
    m_nh->getParam("~pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i) {
      // Read as string to handle no decimals and scientific notation
      std::ostringstream ostr;
      ostr << pose_cov_list[i];
      std::istringstream istr(ostr.str());
      istr >> pose_cov_diag[i];
    }
  } else {
    ROS_WARN("Pose covariance diagonals not specified for odometry integration. Defaulting to 1e-6.");
  }

  if (m_nh->hasParam("~twist_covariance_diagonal")) {
    XmlRpc::XmlRpcValue twist_cov_list;
    m_nh->getParam("~twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i) {
      // Read as string to handle no decimals and scientific notation
      std::ostringstream ostr;
      ostr << twist_cov_list[i];
      std::istringstream istr(ostr.str());
      istr >> twist_cov_diag[i];
    }
  } else {
    ROS_WARN("Twist covariance diagonals not specified for odometry integration. Defaulting to 1e-6.");
  }

  char *buf = new char[strlen(odom_port.c_str()) + 1];
  strcpy(buf, odom_port.c_str());
  m_pRobot = new CRobot(buf);
  // m_pRobot = new CRobot(robot_port.c_ptr()); // c++11不支持此用法


  // 发布器
  m_odom_pub = m_nh->advertise<nav_msgs::Odometry>("odom", pub_rate);

  // 初始化变量
  robot_x = 0.0;
  robot_y = 0.0;
  robot_yaw = 0.0;
  robot_vx = 0.0;
  robot_vy = 0.0;
  robot_yaw_v = 0.0;

  if (odom_tf_publish_flag) {
    m_odom_trans.header.frame_id = odom_id;
    m_odom_trans.child_frame_id = child_id;
    m_odom_trans.transform.translation.z = 0.0;
    m_odom_trans.transform.translation.x = robot_x;
    m_odom_trans.transform.translation.y = robot_y;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_yaw);
    m_odom_trans.transform.rotation = odom_quat;

    m_odom_broadcaster.sendTransform(m_odom_trans);
  }

  m_odom_msg.header.frame_id = odom_id;
  m_odom_msg.child_frame_id = child_id;
  m_odom_msg.pose.pose.position.z = 0.0;
  m_odom_msg.pose.covariance = boost::assign::list_of(pose_cov_diag[0])(0)(0)(0)(0)(0)(0)(pose_cov_diag[1])(0)(0)(0)(0)(0)(0)(pose_cov_diag[2])(0)(0)(0)(0)(0)(0)(pose_cov_diag[3])(0)(0)(0)(0)(0)(0)(pose_cov_diag[4])(0)(0)(0)(0)(0)(0)(pose_cov_diag[5]);
  m_odom_msg.twist.twist.linear.y = 0.0;
  m_odom_msg.twist.twist.linear.z = 0.0;
  m_odom_msg.twist.twist.angular.x = 0.0;
  m_odom_msg.twist.twist.angular.y = 0.0;
  m_odom_msg.twist.covariance = boost::assign::list_of(twist_cov_diag[0])(0)(0)(0)(0)(0)(0)(twist_cov_diag[1])(0)(0)(0)(0)(0)(0)(twist_cov_diag[2])(0)(0)(0)(0)(0)(0)(twist_cov_diag[3])(0)(0)(0)(0)(0)(0)(twist_cov_diag[4])(0)(0)(0)(0)(0)(0)(twist_cov_diag[5]);

  m_pRobot->start();
  m_pRobot->start_2();
}

void odom_node::odomPublish() {
  struct ros::Time now_pub_time;
  now_pub_time = ros::Time::now();

  robot_x = m_pRobot->readOdom(Odom::X);
  robot_y = m_pRobot->readOdom(Odom::Y);
  robot_yaw = m_pRobot->readOdom(Odom::TH);
  robot_vx = m_pRobot->readOdom(Odom::VX);
  robot_vy = m_pRobot->readOdom(Odom::VY);
  robot_yaw_v = m_pRobot->readOdom(Odom::VTH);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_yaw);

  // Transform message
  if (odom_tf_publish_flag) {
    m_odom_trans.header.frame_id = odom_id;
    m_odom_trans.child_frame_id = child_id;
    m_odom_trans.header.stamp = now_pub_time;
    m_odom_trans.transform.translation.x = robot_x;
    m_odom_trans.transform.translation.y = robot_y;
    m_odom_trans.transform.rotation = odom_quat;
    m_odom_broadcaster.sendTransform(m_odom_trans);
  }

  // Odometry message
  m_odom_msg.header.frame_id = odom_id;
  m_odom_msg.child_frame_id = child_id;
  m_odom_msg.header.stamp = now_pub_time;
  m_odom_msg.pose.pose.position.x = robot_x;
  m_odom_msg.pose.pose.position.y = robot_y;
  m_odom_msg.pose.pose.orientation = odom_quat;
  m_odom_msg.twist.twist.linear.x = robot_vx;
  m_odom_msg.twist.twist.linear.y = robot_vy;
  m_odom_msg.twist.twist.angular.z = robot_yaw_v;
  m_odom_pub.publish(m_odom_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hangfa_nq2_odom_node");
  odom_node nd;
  ros::Rate r(nd.pub_rate);

  while (ros::ok()) {
    nd.odomPublish();
    r.sleep();
  }
  return 0;
}
