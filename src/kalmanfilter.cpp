#include <eigen3/Eigen/Dense>
#include <ekf_slam/kalmanfilter.h>
#include <iostream>

KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd &inA,
  const Eigen::MatrixXd &inB,
  const Eigen::MatrixXd &inC,
  const Eigen::MatrixXd &inQ,
  const Eigen::MatrixXd &inR,
  const Eigen::MatrixXd &inP
  )
  : sys_dyn_A(inA),
    input_B(inB),
    output_C(inC),
    proc_noise_cov_Q(inQ),
    meas_noise_cov_R(inR),
    init_error_cov_P0(inP)

    {

      num_states = output_C.rows();
      num_meas = sys_dyn_A.cols();
      num_controls = input_B.cols();
      is_init = false;
      std::cout<<num_meas<<", "<<num_meas<<std::endl;
      identity.resize(num_meas, num_meas);
      std::cout<<num_meas<<std::endl;
      est_state_xhat.resize(num_meas);
      identity.setIdentity();
    }

  void KalmanFilter::init()
  {
    est_state_xhat.setZero();
    est_error_cov_P = init_error_cov_P0;
    is_init = true;
  }

  void KalmanFilter::init(const Eigen::VectorXd &state_x0)
  {
    est_state_xhat = state_x0;
    est_error_cov_P = init_error_cov_P0;
    is_init = true;
  }

  void KalmanFilter::predict_step(const Eigen::VectorXd &controls_u)
  {
    if (!is_init)
    {
      std::cout<<"Initializing filter with estimated state zero."<<std::endl;
      init();
    }

    est_state_xhat = sys_dyn_A * est_state_xhat + input_B * controls_u;
    est_error_cov_P = sys_dyn_A * est_error_cov_P * sys_dyn_A.transpose() + proc_noise_cov_Q;
  }

  void KalmanFilter::update_state(const Eigen::VectorXd &meas_Y)
  {
    kalman_gain_K = est_error_cov_P * output_C.transpose() * (output_C * est_error_cov_P * output_C.transpose()).inverse();
    est_state_xhat = est_state_xhat + kalman_gain_K * (meas_Y - output_C * est_state_xhat);
    est_error_cov_P = (identity - kalman_gain_K * output_C) * est_error_cov_P;

  }

  void KalmanFilter::update_dynamics_matrix(const Eigen::MatrixXd new_sys_dyn_A)
  {
    sys_dyn_A = new_sys_dyn_A;
  }

  void KalmanFilter::update_output_matrix(const Eigen::MatrixXd new_output_C)
  {
    output_C = new_output_C;
  }

  Eigen::VectorXd KalmanFilter::get_state()
  {
    return est_state_xhat;
  }
