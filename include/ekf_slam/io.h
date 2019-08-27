#ifndef IO_H
#define IO_H


#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <map>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <json/json.h>

class SlamIO
{
private:
  std::map<double, std::vector<Eigen::Vector3d>> meas_data;
  std::vector<Eigen::Vector2d> commands;
  std::unordered_map <int, Eigen::Vector2d> landmark_locations;


public:
  SlamIO(std::string path)
  {
    Json::Value root;
    std::ifstream instream(path);
    std::cout<<"Loading json from path "<< path<<std::endl; 
    instream>>root;
    double total_time = 0.0;

    for(auto meas : root["measurements"])
    {
      double dt = meas["dt"].asDouble();
      std::vector<Eigen::Vector3d> mlist;

      for (auto lm : meas["landmarks"])
      {
        Eigen::Vector3d landvec;

        landvec<<lm[0].asDouble(), lm[1].asDouble(), lm[2].asDouble();

        mlist.push_back(landvec);
      }
      total_time+=dt;

      meas_data[total_time] = mlist;

    }

    for (auto comm : root["commands"])
    {
      Eigen::Vector2d command;

      command<<comm[0].asDouble(), comm[1].asDouble();
      commands.push_back(command);
    }

    for (auto true_lm : root["landmark_locations"])
    {
      int id = true_lm[0].asInt();
      int xpos = true_lm[1].asDouble();
      int ypos = true_lm[2].asDouble();

      Eigen::Vector2d landmark;
      landmark<<xpos , ypos;
      landmark_locations[id] = landmark;
    }

    instream.close();

  }



  std::unordered_map<int, Eigen::Vector2d> get_landmark_locations()
  {
    return landmark_locations;
  }
  std::vector<Eigen::Vector2d> get_commands()
  {
    return commands;
  }
  std::map<double, std::vector<Eigen::Vector3d>> get_measurements()
  {
    return meas_data;
  }

};


#endif //IO_H
