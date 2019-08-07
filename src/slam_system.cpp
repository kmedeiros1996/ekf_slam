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



  return 0;
}
