#include <ekf_slam/kalmanfilter.h>
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <sstream>
#include <string>
#include <fstream>


int main()
{
  int num_states = 2;
  int num_meas = 2;
  int num_controls = 1;

  Eigen::MatrixXd sys_dyn_A(num_states, num_states);
  Eigen::MatrixXd input_B(num_states, num_controls);
  Eigen::MatrixXd output_C(num_meas, num_states);
  Eigen::MatrixXd proc_noise_cov_Q(num_states, num_states);
  Eigen::MatrixXd meas_noise_cov_R(num_meas, num_meas);
  Eigen::MatrixXd est_error_cov_P(num_states, num_states);


  sys_dyn_A << 1, 0, 0, 1;
  input_B << 1, 0;
  output_C << 1, 0, 0, 1;

  proc_noise_cov_Q << 1, 0, 0, 1;
  meas_noise_cov_R << 1, 0, 0, 1;
  est_error_cov_P << 0.1, 0, 0, 0.1;

  std::cout<< "System Dynamics: \n "<<sys_dyn_A<<std::endl
  <<"Input Control B: \n"<<input_B<<std::endl
  <<"Output C \n"<<output_C<<std::endl
  <<"Process Noise Covariance Q \n"<<proc_noise_cov_Q<<std::endl
  <<"Measurement Noise Covariance R \n"<<meas_noise_cov_R<<std::endl
  <<"Estimated Error Covariance P \n"<<est_error_cov_P<<std::endl;

  KalmanFilter kf(sys_dyn_A, input_B, output_C, proc_noise_cov_Q, meas_noise_cov_R, est_error_cov_P);

  Eigen::VectorXd measurement(num_meas);
  Eigen::VectorXd control_signal(num_controls);
  Eigen::VectorXd init_state(num_states);
  Eigen::VectorXd gt_state(num_states);

  init_state<<0, 0;
  kf.init(init_state);

  double prev_timestamp = 0.0;


  std::ifstream indata;
  indata.open("src/ekf_slam/resources/kalman_test_data.txt");
  std::string line;



  int i = 0;
  while(std::getline(indata, line))
  {
    std::istringstream iss(line);
    double command, meas_x, meas_y, gt_x, gt_y, timestamp;

    if (!(iss >> command >> meas_x >> meas_y >> gt_x >> gt_y >> timestamp))
    {
      std::cout<<"error in reading data file. "<<std::endl;
      std::cout<<line<<std::endl;
      break;
    }

    std::cout<<command<<" "<<meas_x <<  "," << meas_y <<" "<<gt_x <<  "," << gt_y <<" "<<timestamp<<std::endl;
    double dt = timestamp - prev_timestamp;
    gt_state<<gt_x, gt_y;
    control_signal<<command;
    measurement<<meas_x, meas_y;

    kf.predict_step(control_signal);
    kf.update_state(measurement);

    std::cout << "t = " << timestamp << ", " << "meas[" << i << "] = " << measurement.transpose()
        << ", state[" << i << "] = " << kf.get_state().transpose() << std::endl;


    std::cout<<"Diff from ground truth: \n"<<(kf.get_state() - gt_state).transpose()<<std::endl;

    ++i;
  }

  return 0;
}
