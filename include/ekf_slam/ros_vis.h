#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <eigen3/Eigen/Dense>
#include <vector>

class ROSViz
{

private:

  ros::Publisher marker_array_publisher;
  ros::NodeHandle n;



public:
  ROSViz(std::string topic_name);
  void publish_state(Eigen::VectorXd state, Eigen::MatrixXd covariance);


};
