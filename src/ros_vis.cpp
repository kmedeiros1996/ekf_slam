#include "ekf_slam/ros_vis.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


ROSViz::ROSViz(std::string topic_name)
{

  marker_array_publisher = n.advertise<visualization_msgs::MarkerArray>(topic_name, 1);

}


void ROSViz::publish_state(Eigen::VectorXd state, Eigen::MatrixXd covariance)
{


  ros::Rate r(1);
  std::cout<<"\nVisualization: \n"<<std::endl;


  visualization_msgs::MarkerArray state_markers;

  double pose_x = state(0), pose_y = state(1), pose_theta = state(2);

  std::cout<<"Robot Pose: "<<pose_x<<" "<<pose_y<<" "<<pose_theta<<std::endl;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, pose_theta);
  quat.normalize();

  geometry_msgs::Quaternion quat_msg;

  tf2::convert(quat_msg, quat);


  visualization_msgs::Marker pose_marker;

  pose_marker.header.frame_id = "world";
  pose_marker.header.stamp = ros::Time();
  pose_marker.ns = "robot";
  pose_marker.id = 1;
  pose_marker.type = visualization_msgs::Marker::ARROW;
  pose_marker.action = visualization_msgs::Marker::ADD;

  pose_marker.pose.position.x = pose_x;
  pose_marker.pose.position.y = pose_y;
  pose_marker.pose.position.z = 0;
  pose_marker.pose.orientation = quat_msg;

  pose_marker.scale.x = 0.05;
  pose_marker.scale.y = 0.05;
  pose_marker.scale.z = 0.10;

  pose_marker.color.r = 1.0f;
  pose_marker.color.g = 0.0f;
  pose_marker.color.b = 0.0f;
  pose_marker.color.a = 1.0;
  pose_marker.lifetime = ros::Duration();

  state_markers.markers.push_back(pose_marker);



  std::cout<<pose_marker<<std::endl;

  std::cout<<"Landmarks: "<<std::endl;
  for (int i = 3; i < state.size(); i+=2)
  {

    double lm_x = state(i);
    double lm_y = state(i+1);

    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "landmark";
    marker.id = i-2;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = lm_x;
    marker.pose.position.y = lm_y;
    marker.pose.position.z = 0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    std::cout<<marker<<std::endl;
    state_markers.markers.push_back(marker);
  }

  marker_array_publisher.publish(state_markers);
  std::cout<<"Final Markers: "<<std::endl;
  std::cout<<state_markers<<std::endl;
  r.sleep();

}
