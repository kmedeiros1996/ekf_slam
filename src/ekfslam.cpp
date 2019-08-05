#include <eigen3/Eigen/Dense>
#include <vector>
#include  <ekf_slam/ekfslam.h>
#include <math.h>




EKFSlam::EKFSlam(int num_landmarks, Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(3, 1))
{
  int r_state_size = 3;
  float motion_noise = 0.001;

  this->num_landmarks = num_landmarks;
  state = Eigen::VectorXd::Zero(2*num_landmarks + r_state_size, 1);

  state(0) = initial_state(0);
  state(1) = initial_state(1);
  state(2) = initial_state(2);


  full_state_cov = MatrixXd::Zero(2*num_landmarks+r_state_size, 2*num_landmarks+r_state_size);
  robot_cov = MatrixXd::Zero(r_state_size, r_state_size);
  robot_map_cov = MatrixXd::Zero(r_state_size, 2*num_landmarks);
  map_cov = MatrixXd::Zero(2*num_landmarks, 2*num_landmarks);


  full_state_cov.topLeftCorner(r_state_size, r_state_size) = robot_cov;
  full_state_cov.topRightCorner(r_state_size, 2*num_landmarks) = robot_map_cov;
  full_state_cov.bottomLeftCorner(2*num_landmarks, r_state_size) = robot_map_cov.transpose();
  full_state_cov.bottomRightCorner(2*num_landmarks, 2*num_landmarks) = map_cov;

  proc_noise_cov_Q = MatrixXd::Zero(2*num_landmarks + r_state_size, 2*num_landmarks + r_state_size);

  proc_noise_cov_Q.topLeftCorner(3, 3) <<

    motion_noise, 0, 0,
    0, motion_noise, 0,
    0, 0, motion_noise/10;


    observed_landmarks.rehash(num_landmarks);


}

EKFSlam::~EKFSlam(){}

Eigen::VectorXd EKFSlam::velocity_motion_model_g(Eigen::Vector2d &command, double delta_t)
{
  double lin_vel = command(0);
  double ang_vel = command(1);

  Eigen::VectorXd pose_shift(3, 1);
  double lin_by_ang = lin_vel/ang_vel;
  double prev_theta = state(2);

  double new_theta = ang_vel * delta_t;

  pose_shift << -lin_by_ang * sin(prev_theta) + lin_by_ang * sin(prev_theta + new_theta),
  -lin_by_ang * cos(prev_theta) + lin_by_ang * cos(prev_theta + new_theta),
  new_theta;

  return pose_shift;

}

Eigen::MatrixXd EKFSlam::motion_model_jacobian_G(Eigen::Vector2d &command, double delta_t)
{
  Eigen::MatrixXd jacobian_G = MatrixXd::Zero(3, 3);
  double lin_vel = command(0);
  double ang_vel = command(1);
  double lin_by_ang = lin_vel/ang_vel;
  double prev_theta = state(2);
  double new_theta = ang_vel * delta_t;


  jacobian_G <<
  1, 0, -lin_by_ang * cos(prev_theta) + lin_by_ang * cos(prev_theta +new_theta ),
  0, 1, -lin_by_ang * sin(prev_theta) + lin_by_ang * sin(prev_theta + new_theta),
  0, 0, 0;

  return jacobian_G;
}

Eigen::Vector2d EKFSlam::observation_model_h(Eigen::Vector2d landmark)
{
  Eigen::Vector2d expected_Z_t;
  Eigen::Vector2d delta(landmark(0) - state(0), landmark(1) - state(1));

  double expected_range_q = delta.dot(delta);
  double expected_bearing_q = atan2(delta(1), delta(0)) - state(2);
  expected_Z_t << sqrt(expected_range), expected_bearing;

  return expected_Z_t;
}

Eigen::MatrixXd obs_model_jacobian_H(Eigen::Vector2d landmark)
{
  Eigen::MatrixXd jacobian_H = Eigen::MatrixXd::Zero(2, 5);
  Eigen::Vector2d delta(landmark(0) - state(0), landmark(1) - state(1));

  double q = delta.dot(delta);

  jacobian_H<<
  -sqrt(q)*delta(0), -sqrt(q)* delta(1), 0, sqrt(q) *delta(0), sqrt(q)*delta(1),
  delta(1), -delta(0), -q, -delta(1), delta(0);

  jacobian_H = jacobian_H/q;

  return jacobian_H;

}



void EKFSlam::predict_step(const Eigen::VectorXd &command_u, double delta_t)
{

  Eigen::VectorXd pose_shift_g = velocity_motion_model_g(command_u, delta_t);
  Eigen::MatrixXd jacobian_Gt = motion_model_jacobian_G(command_u, delta_t);


  state(0) += pose_shift_g(0);
  state(1) += pose_shift_g(1);
  state(2) += pose_shift_g(2);

  int csize = full_state_cov.cols();

  full_state_cov.topLeftCorner(3, 3) = jacobian_Gt * full_state_cov.topLeftCorner(3, 3) * jacobian_Gt.transpose();
  full_state_cov.topRightCorner(3, csize-3) = jacobian_Gt * full_state_cov.topRightCorner(3, csize-3);
  full_state_cov.bottomLeftCorner(csize-3, 3) = full_state_cov.topRightCorner(3, csize-3).transpose();
  full_state_cov = full_state_cov + proc_noise_cov_Q;

}


void EKFSlam::update_state(std::vector<Eigen::Vector3d> &input_measurement)
{
  int meas_size = meas_y.size();

  Eigen::VectorXd expected_Z = VectorXd::Zero(2*meas_size);
  Eigen::VectorXd measured_Z = VectorXd::Zero(2*meas_size);

  Eigen::MatrixXd jacobian_H = Eigen::MatrixXd::Zero(2*meas_size, 2*observed_landmarks.size() + 3);
  for (int j = 0; i < meas_size(); j++)
  {
    Eigen::Vector3d measurement = input_measurement.at(j);

    int id = measurement(0);
    double dist = measurement(1);
    double bearing = measurement(2);

    //If the landmark isn't in our hashmap of landmarks, we'll initialize it here
    if (observed_landmarks.find(id) == observed_landmarks.end())
    {
      Eigen::Vector2d landmark;
      landmark <<
      state(0) + dist * cos(state(2) + bearing),
      state(1) + dist * sin(state(2) + bearing);

      observed_landmarks[id] = landmark;
    }
    measured_Z(2*j) = dist;
    measured_Z(2*j + 1) = bearing;


    Eigen::Vector2d landmark = observed_landmarks[id];
    Eigen::Vector2d expected_meas_Z_t = observation_model_h(landmark);

    expected_Z(2*j) = expected_meas_Z_t(0);
    expected_Z(2*j+1) = expected_meas_Z_t(1);


    Eigen::MatrixXd H_t = obs_model_jacobian_H(landmark);

    jacobian_H.block<2,3> (2*j, 0) <<
      H_t(0, 0), H_t(0, 1), H_t(0, 2),
      H_t(1, 0), H_t(1, 1), H_t(1, 2);

    jacobian_H.block<2,2> (2*i, 2*id+1) <<
      H_t(0, 3), H_t(0, 4),
      H_t(1, 3), H_t(1, 4);

  }

  Eigen::MatrixXd meas_noise_cov_R = Eigen::MatrixXd::Identity(2*meas_size, 2*meas_size);
  kalman_gain_K = full_state_cov * jacobian_H.transpose() * (jacobian_H * full_state_cov * jacobian_H.transpose() + meas_noise_cov_R).inverse();
  Eigen::Vector2d meas_diff = measured_Z - expected_Z;

  state = state + kalman_gain_K * meas_diff;
  full_state_cov = full_state_cov - kalman_gain_K*jacobian_H*full_state_cov;
  
}
