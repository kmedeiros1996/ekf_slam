#ifndef KALMAN_H
#define KALMAN_H

#include <eigen3/Eigen/Dense>
class KalmanFilter
{

private:

int num_states, num_meas, num_controls;
Eigen::MatrixXd sys_dyn_A,
                input_B,
                output_C,
                proc_noise_cov_Q,
                meas_noise_cov_R,
                est_error_cov_P,
                kalman_gain_K,
                init_error_cov_P0,
                identity;

Eigen::VectorXd est_state_xhat;

bool is_init = false;


public:
  KalmanFilter(
    const Eigen::MatrixXd &inA,
    const Eigen::MatrixXd &inB,
    const Eigen::MatrixXd &inC,
    const Eigen::MatrixXd &inQ,
    const Eigen::MatrixXd &inR,
    const Eigen::MatrixXd &inP
    );


  void init();  //States = 0
  void init(const Eigen::VectorXd &state_x0);


  void predict_step(const Eigen::VectorXd &controls_u);
  void update_state(const Eigen::VectorXd &meas_Y);
  void update_dynamics_matrix(const Eigen::MatrixXd new_sys_dyn_A);
  void update_output_matrix(const Eigen::MatrixXd new_output_C);

  Eigen::VectorXd get_state();

};


#endif //KALMAN_H
