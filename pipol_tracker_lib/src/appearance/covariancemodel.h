#pragma once
#ifndef COVARIANCEMODEL_H__
#define COVARIANCEMODEL_H__

#include <Eigen/Eigenvalues>
#include <opencv2/core/eigen.hpp>
#include "appearance.h"

/**
 * @brief The CovarianceModel class uses an edge-based covariance matrix as representation.
 *        Functions documented in AbstractAppearanceModel are not documented here.
 */
class CovarianceModel : public AbstractAppearanceModel
{
protected:
  std::vector<cv::Mat> _covMatrices;
  cv::Mat _mean;
  double _varianceSqr;
  cv::Mat _distances;

  /**
   * @brief Update the inner Gaussian of the model: _mean and _varianceSqr.
   */
  void _updateGaussian();

public:
  CovarianceModel();
  virtual void addAppearance(cv::Mat& img, const cv::Rect& detBox = cv::Rect());
  virtual void merge(const AbstractAppearanceModel* app);
  virtual double match(const AbstractAppearanceModel* app) const;

protected:
  /**
   * @brief Compute the distance of two covariance matrices.
   * @param a
   * @param b
   * @return \sum ln^2 \lambda_k (C_i, C_j) where Ax = \lambda Bx  where  x_k != 0, lambdas are generalized eigenvalues.
   * @bug the math is not able to handle matrices with nonindependent columns. Returns nan in these cases. See code for more.
   */
  static double covMatrixDistance(const cv::Mat& a, const cv::Mat& b);

  /**
   * @brief computeProbFromGaussian
   * @param mean
   * @param varianceSqr
   * @param x
   * @return exp(-pow(covMatrixDistance(mean, x),2) / (2*varianceSqr))/sqrt(2.0*M_PI*varianceSqr)
   */
  static double computeProbFromGaussian(const cv::Mat& mean, double varianceSqr, const cv::Mat& x);

  cv::Mat _Ix, _Iy, _Ixx, _Iyy, _Ixy; // variables to store temporary derived images
  cv::Mat _kernelX, _kernelY, _kernelXX, _kernelYY; // variables to store derivation kernels

};
#endif
