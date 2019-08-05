#ifndef EKFSLAM_H
#define EKFSLAM_H
#include <eigen3/Eigen/Dense>
#include <vector>
#include <unordered_map>

/*This EKF Slam implementation assumes the following:
  -movement on a 2D plane (x, y, theta)
  -velocity based motion model given u = [linear_velocity, angular_velocity]
  -range bearing observation model given z = [landmark_id, distance_to_landmark_r, bearing_to_landmark_phi]
*/


class EKFSlam
{

private:

  int num_landmarks;
  Eigen::MatrixXd proc_noise_cov_Q;
  Eigen::MatrixXd robot_cov;
  Eigen::MatrixXd robot_map_cov;
  Eigen::MatrixXd map_cov;
  Eigen::MatrixXd full_state_cov;
  Eigen::VectorXd state;

  long double timestamp;

  std::unordered_map<int, Eigen::Vector2d> observed_landmarks;

  Eigen::VectorXd velocity_motion_model_g(Eigen::Vector2d &command, double delta_t);
  Eigen::MatrixXd motion_model_jacobian_G(Eigen::Vector2d &command, double delta_t);

  Eigen::Vector2d observation_model_h(Eigen::Vector2d landmark);
  Eigen::MatrixXd obs_model_jacobian_H();

public:

  EKFSlam(int num_landmarks, Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(3, 1));
  ~EKFSlam();


  void predict_step(const Eigen::VectorXd &command_u, double delta_t);
  void update_state(std::vector<Eigen::Vector3d> &input_measurement);

  Eigen::VectorXd get_state();
  Eigen::MatrixXd get_state_covariance();

};






#endif
