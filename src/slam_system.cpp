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

  SlamIO *io = new SlamIO("src/ekf_slam/resources/data.json");

  std::unordered_map <int, Eigen::Vector2d> landmark_locations = io->get_landmark_locations();
  std::vector<Eigen::Vector2d> commands = io->get_commands();
  std::map<double, std::vector<Eigen::Vector3d>> measurements = io->get_measurements();

  delete io;


  std::unordered_map<int, Eigen::Vector2d>::iterator lmit = landmark_locations.begin();

  std::vector<double> plot_x, plot_y;
  while (lmit != landmark_locations.end())
  {
    auto lm = lmit->second;
    plot_x.push_back(lm[0]);
    plot_y.push_back(lm[1]);
    lmit++;
  }
  plt::plot(plot_x, plot_y, "bx");
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


    dt = it->first - prev_time;
    prev_time = it->first;
    std::cout<<"Predicting given command..."<<std::endl;
    system->predict_step(commands[command_index], dt);

    std::cout<<"Taking measurements and correcting state..."<<std::endl;
    system->update_state(it->second);

    std::cout<<"STATE: \n"<<system->get_state()<<std::endl<<"COV: \n"<<system->get_state_covariance()<<std::endl;

    plt::show();
    it++;
  }

  delete system;
  return 0;
}
