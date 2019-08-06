#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include <vector>


namespace slamIO
{

  std::unordered_map<int, Eigen::Vector2d> load_landmarks(std::string path)
  {
    std::unordered_map <int, Eigen::Vector2d> landmark_locations;
    std::ifstream in_landmarks;
    in_landmarks.open(path);
    std::string landmark_line;

    while(std::getline(in_landmarks, landmark_line))
    {
      std::istringstream iss(landmark_line);
      int id;
      double xpos, ypos;

      if (!(iss >> id >> xpos >> ypos))
      {
        std::cout<<"Error with reading landmarks."<<std::endl<<landmark_line<<std::endl;
        break;
      }
      Eigen::Vector2d landmark (xpos, ypos);

      landmark_locations[id] = landmark;
    }

    return landmark_locations;
  }


  std::vector<Eigen::Vector2d> load_commands(std::string path)
  {
    std::vector<Eigen::Vector2d> commands;
    std::ifstream in_commands;
    in_commands.open(path);
    std::string command_ln;

    while(std::getline(in_commands, command_ln))
    {
      std::istringstream iss(command_ln);
      double v, w;
      if (!(iss >> v >> w))
      {
        std::cout<<"Error with reading command data."<<std::endl<<command_ln<<std::endl;
        break;
      }
      Eigen::Vector2d command (v, w);
      commands.push_back(command);
    }

    return commands;
  }

}
