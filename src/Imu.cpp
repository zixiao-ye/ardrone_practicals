/*
 * Imu.cpp
 *
 *  Created on: 8 Feb 2017
 *      Author: sleutene
 */

#include <arp/kinematics/Imu.hpp>
#include <iostream>

namespace arp {
namespace kinematics {

bool Imu::stateTransition(const RobotState & state_k_minus_1,
                          const ImuMeasurement & z_k_minus_1,
                          const ImuMeasurement & z_k, RobotState & state_k,
                          ImuKinematicsJacobian* jacobian)
{
  // get the time delta
  const double dt = double(
      z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;
  if (dt < 1.0e-12 || dt > 0.05) {
    // for safety, we assign reasonable values here.
    state_k = state_k_minus_1;
    if(jacobian) {
      jacobian->setIdentity();
    }
    return false;  // negative, no or too large time increments not permitted
  }

  // TODO: implement trapezoidal integration

  Eigen::Vector3d d_alpha1 = dt * state_k_minus_1.q_WS.toRotationMatrix() * (z_k_minus_1.omega_S - state_k_minus_1.b_g);
  RobotState dx1;
  dx1.t_WS = dt * state_k_minus_1.v_W;
  dx1.q_WS = (deltaQ(d_alpha1)*state_k_minus_1.q_WS);
  dx1.v_W = dt * (state_k_minus_1.q_WS.toRotationMatrix() * (z_k_minus_1.acc_S - state_k_minus_1.b_a) + Eigen::Vector3d(0,0,-9.81));
  dx1.b_g = Eigen::Vector3d::Zero();
  dx1.b_a = Eigen::Vector3d::Zero();

  Eigen::Vector3d d_alpha2 = dt * dx1.q_WS.toRotationMatrix() * (z_k.omega_S - state_k_minus_1.b_g);
  RobotState dx2;
  dx2.t_WS = dt * (state_k_minus_1.v_W + dx1.v_W);
  dx2.q_WS = (deltaQ(d_alpha2) * state_k_minus_1.q_WS);
  dx2.v_W = dt * (dx1.q_WS.toRotationMatrix() * (z_k.acc_S - state_k_minus_1.b_a) + Eigen::Vector3d(0,0,-9.81));
  dx2.b_g = Eigen::Vector3d::Zero();
  dx2.b_a = Eigen::Vector3d::Zero();

  state_k.t_WS = state_k_minus_1.t_WS + (dx1.t_WS+dx2.t_WS)/2;
  state_k.q_WS = (deltaQ((d_alpha1 + d_alpha2)/2) * state_k_minus_1.q_WS);
  state_k.v_W = state_k_minus_1.v_W + (dx1.v_W + dx2.v_W)/2;
  state_k.b_g = state_k_minus_1.b_g;
  state_k.b_a = state_k_minus_1.b_a;


  if (jacobian) {
    // TODO: if requested, impement jacobian of trapezoidal integration with chain rule
    ImuKinematicsJacobian I_15,F_c1,F_c2;
    I_15.setIdentity();

    F_c1.setZero();
    F_c1.block<3,3>(0,6).setIdentity();
    F_c1.block<3,3>(3,3) = -crossMx(state_k_minus_1.q_WS.toRotationMatrix() * (z_k_minus_1.omega_S - state_k_minus_1.b_g));
    F_c1.block<3,3>(6,3) = -crossMx(state_k_minus_1.q_WS.toRotationMatrix() * (z_k_minus_1.acc_S - state_k_minus_1.b_a));
    F_c1.block<3,3>(3,9) = -state_k_minus_1.q_WS.toRotationMatrix();
    F_c1.block<3,3>(6,12) = -state_k_minus_1.q_WS.toRotationMatrix();

    F_c2.setZero();
    F_c2.block<3,3>(0,6).setIdentity();
    F_c2.block<3,3>(3,3) = -crossMx(dx1.q_WS.toRotationMatrix() * (z_k.omega_S - state_k.b_g));
    F_c2.block<3,3>(6,3) = -crossMx(dx1.q_WS.toRotationMatrix() * (z_k.acc_S - state_k.b_a));
    F_c2.block<3,3>(3,9) = -dx1.q_WS.toRotationMatrix();
    F_c2.block<3,3>(6,12) = -dx1.q_WS.toRotationMatrix();


    *jacobian = I_15 + dt/2*F_c1 + dt/2 * F_c2 * (I_15 + dt*F_c1);
    //jacobian->setIdentity();
    jacobian->block<3,3>(3,3).setIdentity();
    jacobian->block<3,3>(3,9) = -dt/2*(state_k_minus_1.q_WS.toRotationMatrix() + state_k.q_WS.toRotationMatrix());
    jacobian->block<3,3>(6,3) = -dt/2*crossMx(state_k_minus_1.q_WS.toRotationMatrix()*(z_k_minus_1.acc_S-state_k_minus_1.b_a) + state_k.q_WS.toRotationMatrix()*(z_k.acc_S-state_k.b_a));
  }
  return true;
}

}
}  // namespace arp

