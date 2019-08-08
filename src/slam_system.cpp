#include <eigen3/Eigen/Dense>
#include <vector>
#include <ekf_slam/ekfslam.h>
#include <ekf_slam/matplotlibcpp.h>
#include <ekf_slam/io.h>
#include <math.h>
#include <unordered_map>
#include <map>
#include <stdio.h>

namespace plt = matplotlibcpp;
int main()
{

  std::unordered_map<int, Eigen::Vector2d> landmark_locations = slamIO::load_landmarks("src/ekf_slam/resources/landmark_locations.txt");
  std::vector<Eigen::Vector2d> commands = slamIO::load_commands("src/ekf_slam/resources/commands.txt");
  std::map<double, std::vector<Eigen::Vector3d>> measurements = slamIO::get_measurements("src/ekf_slam/resources/measurements.json");

  EKFSlam *system = new EKFSlam(landmark_locations.size());

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
    it++;

    dt = it->first - prev_time;
    prev_time = it->first;
    std::cout<<"Predicting given command..."<<std::endl;
    system->predict_step(commands[command_index], dt);

    std::cout<<"Taking measurements and correcting state..."<<std::endl;
    system->update_state(it->second);

    std::cout<<"STATE: \n"<<system->get_state()<<std::endl<<"COV: \n"<<system->get_state_covariance()<<std::endl;
  }

  delete system;
  return 0;
}
