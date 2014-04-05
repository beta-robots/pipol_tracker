#include "covariancemodel.h"

CovarianceModel::CovarianceModel()
{
  _kernelX.create(1, 3, CV_64FC1);
  _kernelX.at<double>(0,0) = -1.0;
  _kernelX.at<double>(0,1) = 0.0;
  _kernelX.at<double>(0,2) = 1.0;

  _kernelY.create(3, 1, CV_64FC1);
  _kernelY.at<double>(0,0) = -1.0;
  _kernelY.at<double>(1,0) = 0.0;
  _kernelY.at<double>(2,0) = 1.0;

  _kernelXX.create(1, 3, CV_64FC1);
  _kernelXX.at<double>(0,0) = 1.0;
  _kernelXX.at<double>(0,1) = -2.0;
  _kernelXX.at<double>(0,2) = 1.0;

  _kernelYY.create(3, 1, CV_64FC1);
  _kernelYY.at<double>(0,0) = 1.0;
  _kernelYY.at<double>(1,0) = -2.0;
  _kernelYY.at<double>(2,0) = 1.0;
}


void CovarianceModel::addAppearance(cv::Mat & img, const cv::Rect &detBox)
{
  cv::Mat newCovMat = cv::Mat::zeros(5,5, CV_64FC1);

  cv::Mat target = (detBox == cv::Rect()) ? img : img(detBox);
  cv::cvtColor(target, target, CV_RGB2GRAY);

  const int totalPoints = target.rows * target.cols;

  _Ix.create(target.rows, target.cols, CV_64FC1);
  _Iy.create(target.rows, target.cols, CV_64FC1);
  _Ixx.create(target.rows, target.cols, CV_64FC1);
  _Iyy.create(target.rows, target.cols, CV_64FC1);
  _Ixy.create(target.rows, target.cols, CV_64FC1);

  cv::filter2D(target, _Ix, CV_64FC1, _kernelX);
  cv::filter2D(target, _Iy, CV_64FC1, _kernelY);
  cv::filter2D(target, _Ixx, CV_64FC1, _kernelXX);
  cv::filter2D(target, _Iyy, CV_64FC1, _kernelYY);
  cv::Sobel(target, _Ixy, CV_64FC1, 1, 1);

  cv::Mat meanVec(5, 1, CV_64FC1);
  meanVec.at<double>(0,0) = cv::mean(_Ix)(0); // cv::Scalar s; s(0) will give the first element in the scalar
  meanVec.at<double>(1,0) = cv::mean(_Iy)(0);
  meanVec.at<double>(2,0) = cv::mean(_Ixx)(0);
  meanVec.at<double>(3,0) = cv::mean(_Iyy)(0);
  meanVec.at<double>(4,0) = cv::mean(_Ixy)(0);

  cv::Mat pix(5, 1, CV_64FC1);

  for(int i=0; i<target.rows; ++i)
  {
    for(int j=0; j<target.cols; ++j)
    {
      pix.at<double>(0,0) = _Ix.at<double>(i,j);
      pix.at<double>(1,0) = _Iy.at<double>(i,j);
      pix.at<double>(2,0) = _Ixx.at<double>(i,j);
      pix.at<double>(3,0) = _Iyy.at<double>(i,j);
      pix.at<double>(4,0) = _Ixy.at<double>(i,j);

      const cv::Mat pixMinMeanVec = pix-meanVec;
      const cv::Mat currCovMat = (pixMinMeanVec * pixMinMeanVec.t());

      newCovMat += currCovMat;
    }
  }
  newCovMat /= (totalPoints-1);

  _covMatrices.push_back(newCovMat);

  if(_covMatrices.size() > 2)
    _updateGaussian();
}

void CovarianceModel::_updateGaussian()
{
  // update distances between elemen covMatrices
  _distances.create(_covMatrices.size(), _covMatrices.size(), CV_64FC1);
  // not the best implementation since this matrix is a symmetric one
  // should only need to compute the upper or lower diagonal
  for(unsigned int i=0; i<_covMatrices.size(); ++i)
    for(unsigned int j=0; j<_covMatrices.size(); ++j)
      _distances.at<double>(i,j) = covMatrixDistance(_covMatrices[i], _covMatrices[j]);

  // update mean
  std::vector<double> combined;
  for(int i=0; i<_distances.rows; ++i)
  {
    double sum = 0.0;
    for(int j=0; j<_distances.cols; ++j)
    {
      sum += _distances.at<double>(i,j);
    }
    combined.push_back(sum);
  }
  // the mean is the matrice which has the least distance to all others
  const int meanIdx = std::distance(combined.begin(), std::min_element(combined.begin(), combined.end()));
  _mean = _covMatrices[meanIdx];

  // update variance
  _varianceSqr = 0.0;
  for(int i=0; i<_distances.rows; ++i)
    // since I already have the distances computed let's use them
    _varianceSqr += pow(_distances.at<double>(i, meanIdx),2);
  _varianceSqr /= _covMatrices.size();

  //  if(isnan(_varianceSqr))
  //  {
  //    for(int i=0; i<_distances.rows; ++i)
  //      std::cerr << _distances.at<double>(i, meanIdx) << ",";
  //    std::cerr << std::endl;
  //    std::cerr << _distances << std::endl;
  //    throw std::runtime_error("variance turned nan");
  //  }
}

void CovarianceModel::merge(const AbstractAppearanceModel* app)
{
  const CovarianceModel* other = dynamic_cast<const CovarianceModel*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in CovarianceModel::merge: the given model is not of this type");
  if(other->_covMatrices.empty())
    throw std::runtime_error("Error in CovarianceModel::merge: the given model is empty");

  _covMatrices.push_back(*(other->_covMatrices.end()));
  _updateGaussian();
}

double CovarianceModel::covMatrixDistance(const cv::Mat& a, const cv::Mat& b)
{
  // if the values are the same in each matrix, save the trouble of eigenvalues
  // the behaviour of the math is the same anyway
  if(cv::countNonZero(a == b) == a.cols*a.rows)
  {
    return 0.0;
  }
  else
  {
    double result = 0.0;
    // use the Eigen library to compute the generalized eigenvalues of two covariance matrices
    // Ax = \lambda Bx  where  x_k != 0
    Eigen::MatrixXd C_i, C_j;
    cv::cv2eigen(a, C_i);
    cv::cv2eigen(b, C_j);

    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es;
    es.compute(C_i, C_j, Eigen::ComputeEigenvectors|Eigen::Ax_lBx);

    cv::Mat eigenValues;
    cv::eigen2cv(Eigen::MatrixXd(es.eigenvalues().real()), eigenValues);

    for(int i=0; i<eigenValues.rows; ++i) // eigenValues.rows == 5
      result += pow(log(eigenValues.at<double>(i,0)), 2);  // ln^2 \lambda_k (C_i, C_j)
    //result = sqrt(result);

    // http://www.developerstation.org/2012/04/general-eigen-values-and-eigen-vectors.html
    // http://eigen.tuxfamily.org/dox/classEigen_1_1GeneralizedSelfAdjointEigenSolver.html#aaa204ef15aaefac270c0376269083ed6

    //    if(isnan(result))
    //    {
    //      /* Problem: the generalized eigenvalues of these matrices cannot be computed because
    //       * each column should be independent and zero vectors are clearly not like that.
    //       *Example:
    //        A:[0, 0, 0, 0, 0;
    //           0, 1482.289314516129, 0, 4.623991935483871, 0;
    //           0, 0, 0, 0, 0;
    //           0, 4.623991935483871, 0, 1138.168346774193, 0;
    //           0, 0, 0, 0, 0]
    //        B:[0, 0, 0, 0, 0;
    //           0, 21.51992753623188, 0, -1.005434782608696, 0;
    //           0, 0, 0, 0, 0;
    //           0, -1.005434782608696, 0, 11.07065217391304, 0;
    //           0, 0, 0, 0, 0]
    //      */
    //      std::cerr << "A:" << a << std::endl
    //                << "B:" << b << std::endl
    //                << "eigenvalues: " << eigenValues << std::endl
    //                << "distance: " << result << std::endl;
    //      throw std::runtime_error("distance is nan, possibly eigenvalues are also nan");
    //    }
    return result;
  }
}

double CovarianceModel::computeProbFromGaussian(const cv::Mat& mean, double varianceSqr, const cv::Mat& x)
{
  const double numerator = exp(- pow(covMatrixDistance(mean, x),2) / (2*varianceSqr));
  const double denominator = sqrt(2.0*M_PI*varianceSqr);
  //  if(isnan(numerator/denominator))
  //  {
  //    std::cerr << numerator << "/" << denominator << std::endl
  //              << "varianceSqr: " << varianceSqr << std::endl;
  //    throw std::runtime_error("nan as matching result.");
  //  }
  const double result = numerator/denominator;
  // HACK: when there is a nan, usually coming from covMatrixDistance, it returns 0.
  return isnan(result) ? 0.0 : result;
}

double CovarianceModel::match(const AbstractAppearanceModel* app) const
{
  const CovarianceModel* other = dynamic_cast<const CovarianceModel*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in CovarianceModel::match: the given model is not of this type");
  if(other->_covMatrices.empty())
    throw std::runtime_error("Error in CovarianceModel::match: the given model is empty");

  if(_covMatrices.size() < 3)
  {
    std::cerr << "Error in CovarianceModel::match: missing enough data to compute Gaussian" << std::endl;
    return 0.0;
  }

  return computeProbFromGaussian(_mean, _varianceSqr, other->_covMatrices[0]);
}
