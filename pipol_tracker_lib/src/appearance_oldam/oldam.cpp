#include "oldam.h"

const double Oldam::w[] = {0.33, 0.33, 0.33};

void Oldam::addAppearance(cv::Mat &img, const cv::Rect &detBox)
{
  _hsHist.addAppearance(img, detBox);
  _hog.addAppearance(img, detBox);
  _cov.addAppearance(img, detBox);
}

double Oldam::match(const AbstractAppearanceModel* app) const
{
  const Oldam* other = dynamic_cast<const Oldam*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in Oldam::match: the given model is not of this type");

  const double hsResult = _hsHist.match(&other->_hsHist);
  const double hogResult = _hog.match(&other->_hog);
  const double covResult = _cov.match(&other->_cov);

  //return (hsResult*w[0]) + (hogResult*w[1]) + (covResult*w[2]);
  return hsResult*hogResult*covResult;
}

void Oldam::merge(const AbstractAppearanceModel *app)
{
  const Oldam* other = dynamic_cast<const Oldam*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in Oldam::match: the given model is not of this type");

  _hsHist.merge(&other->_hsHist);
  _hog.merge(&other->_hog);
  _cov.merge(&other->_cov);
}
