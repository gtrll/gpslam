/**
 *  @file  GPutils.h
 *  @brief GP utils, calculation of Qc, Q, Lamda matrices etc.
 *  @author Xinyan Yan, Jing Dong
 *  @date Qct 26, 2015
 **/

#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

#include <cmath>


namespace gpslam {

/// get Qc covariance matrix from noise model
gtsam::Matrix getQc(const gtsam::SharedNoiseModel& Qc_model);


/// calculate Q
template <int Dim>
Eigen::Matrix<double, 2*Dim, 2*Dim> calcQ(const Eigen::Matrix<double, Dim, Dim>& Qc, double tau) {
  Eigen::Matrix<double, 2*Dim, 2*Dim> Q = (Eigen::Matrix<double, 2*Dim, 2*Dim>() <<
      1.0 / 3 * pow(tau, 3.0) * Qc, 1.0 / 2 * pow(tau, 2.0) * Qc,
      1.0 / 2 * pow(tau, 2.0) * Qc, tau * Qc).finished();
  return Q;
}

/// calculate Q_inv
template <int Dim>
Eigen::Matrix<double, 2*Dim, 2*Dim> calcQ_inv(const Eigen::Matrix<double, Dim, Dim>& Qc, double tau) {
  Eigen::Matrix<double, Dim, Dim> Qc_inv = Qc.inverse();
  Eigen::Matrix<double, 2*Dim, 2*Dim> Q_inv =
      (Eigen::Matrix<double, 2*Dim, 2*Dim>() <<
          12.0 * pow(tau, -3.0) * Qc_inv, (-6.0) * pow(tau, -2.0) * Qc_inv,
          (-6.0) * pow(tau, -2.0) * Qc_inv, 4.0 * pow(tau, -1.0) * Qc_inv).finished();
  return Q_inv;
}

/// calculate Phi
template <int Dim>
Eigen::Matrix<double, 2*Dim, 2*Dim> calcPhi(double tau) {

  Eigen::Matrix<double, 2*Dim, 2*Dim> Phi = (Eigen::Matrix<double, 2*Dim, 2*Dim>() <<
      Eigen::Matrix<double, Dim, Dim>::Identity(), tau * Eigen::Matrix<double, Dim, Dim>::Identity(),
      Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Identity()).finished();
  return Phi;
}

/// calculate Lambda
template <int Dim>
Eigen::Matrix<double, 2*Dim, 2*Dim> calcLambda(const Eigen::Matrix<double, Dim, Dim>& Qc,
    double delta_t, const double tau) {

  Eigen::Matrix<double, 2*Dim, 2*Dim> Lambda = calcPhi<Dim>(tau) - calcQ(Qc, tau) * (calcPhi<Dim>(delta_t - tau).transpose())
    * calcQ_inv(Qc, delta_t) * calcPhi<Dim>(delta_t);
  return Lambda;
}

/// calculate Psi
template <int Dim>
Eigen::Matrix<double, 2*Dim, 2*Dim> calcPsi(const Eigen::Matrix<double, Dim, Dim>& Qc,
    double delta_t, double tau) {

  Eigen::Matrix<double, 2*Dim, 2*Dim> Psi = calcQ(Qc, tau) * (calcPhi<Dim>(delta_t - tau).transpose())
    * calcQ_inv(Qc, delta_t);
  return Psi;
}

} // namespace gtsam

