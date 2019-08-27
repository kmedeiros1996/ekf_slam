#include "ekf_slam/ros_vis.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unordered_map>

ROSViz::ROSViz(std::string state_topic_name, std::string true_landmark_topic_name)
{

  marker_array_publisher = n.advertise<visualization_msgs::MarkerArray>(state_topic_name, 1);
  true_landmark_publisher = n.advertise<visualization_msgs::MarkerArray>(true_landmark_topic_name, 2);
}

void ROSViz::publish_true_landmarks(std::unordered_map <int, Eigen::Vector2d> landmark_locations )
{
  ros::Rate r(1);
  visualization_msgs::MarkerArray landmark_markers;
  std::unordered_map<int, Eigen::Vector2d>::iterator lmit = landmark_locations.begin();

  int id = 0;
  while (lmit != landmark_locations.end())
  {
    visualization_msgs::Marker landmark_marker;
    auto lm = lmit->second;
    std::cout<<"Publishing landmark "<<lm<<std::endl;
    double pose_x = lm[0], pose_y = lm[1];
    landmark_marker.header.frame_id = "world";
    landmark_marker.header.stamp = ros::Time();
    landmark_marker.ns = "true_lm";
    landmark_marker.type = visualization_msgs::Marker::CYLINDER;
    landmark_marker.action = visualization_msgs::Marker::ADD;
    landmark_marker.id = id;
    landmark_marker.pose.position.x = pose_x;
    landmark_marker.pose.position.y = pose_y;
    landmark_marker.pose.position.z = 0;

    landmark_marker.scale.x = 0.02;
    landmark_marker.scale.y = 0.08;
    landmark_marker.scale.z = 0.02;

    landmark_marker.color.r = 0.0f;
    landmark_marker.color.g = 1.0f;
    landmark_marker.color.b = 0.0f;
    landmark_marker.color.a = 1.0;
    landmark_marker.lifetime = ros::Duration();

    std::cout<<landmark_marker<<std::endl;
    landmark_markers.markers.push_back(landmark_marker);
    id++;
    lmit++;
  }

  true_landmark_publisher.publish(landmark_markers);
  r.sleep();
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

  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat);


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

  pose_marker.scale.x = 0.09;
  pose_marker.scale.y = 0.05;
  pose_marker.scale.z = 0.05;

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
    state_markers.markers.push_back(marker);
  }

  marker_array_publisher.publish(state_markers);

  r.sleep();

}
