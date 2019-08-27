#include <eigen3/Eigen/Dense>
#include <vector>
#include <ekf_slam/ekfslam.h>
#include <ekf_slam/io.h>
#include <math.h>
#include <unordered_map>
#include <map>
#include <stdio.h>
#include "ekf_slam/ros_vis.h"


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "SLAM_markers");



  SlamIO *io = new SlamIO("src/ekf_slam/resources/data.json");

  std::unordered_map <int, Eigen::Vector2d> landmark_locations = io->get_landmark_locations();
  std::vector<Eigen::Vector2d> commands = io->get_commands();
  std::map<double, std::vector<Eigen::Vector3d>> measurements = io->get_measurements();

  delete io;


  std::unordered_map<int, Eigen::Vector2d>::iterator lmit = landmark_locations.begin();

  EKFSlam *system = new EKFSlam(landmark_locations.size());
  ROSViz *viz = new ROSViz("landmark_locations", "true_landmarks");


  viz->publish_true_landmarks(landmark_locations);

  std::map<double, std::vector<Eigen::Vector3d>>::iterator it = measurements.begin();

  size_t command_index = 0;
  double prev_time = 0;
  double dt = 0;
  while (it != measurements.end())
  {

    std::cout<<"Time: "<< it->first<<std::endl<<"Measurements: "<< std::endl;
    for (auto lm : it->second)
      std::cout<<lm.transpose()<<",";

    std::cout<<std::endl<<" Command: ";
    std::cout<<commands[command_index].transpose()<<std::endl;
    command_index++;


    dt = it->first - prev_time;
    prev_time = it->first;
    std::cout<<"Predicting given command..."<<std::endl;
    system->predict_step(commands[command_index], dt);

    std::cout<<"Taking measurements and correcting state..."<<std::endl;
    system->update_state(it->second);


    std::cout<<"STATE: \n"<<system->get_state()<<std::endl<<"COV: \n"<<system->get_state_covariance()<<std::endl;

    viz->publish_state(system->get_state(), system->get_state_covariance());
    it++;

  }

  delete system;
  delete viz;

  return 0;
}
