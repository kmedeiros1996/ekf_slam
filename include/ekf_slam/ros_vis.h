#ifndef ROS_VIZ_H
#define ROS_VIZ_H


#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <unordered_map>


class ROSViz
{

private:


  ros::NodeHandle n;
  ros::Publisher marker_array_publisher;
  ros::Publisher true_landmark_publisher;

public:
  ROSViz(std::string state_topic_name, std::string true_landmark_topic_name);
  void publish_true_landmarks(std::unordered_map <int, Eigen::Vector2d> landmark_locations);
  void publish_state(Eigen::VectorXd state, Eigen::MatrixXd covariance);


};

#endif //ROS_VIZ_H
