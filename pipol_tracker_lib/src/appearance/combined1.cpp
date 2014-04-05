#include "combined1.h"

void Combined1::addAppearance(cv::Mat &img, const cv::Rect &detBox)
{
  _hsHist.addAppearance(img, detBox);
  _covModel.addAppearance(img, detBox);
  _rgbHist.addAppearance(img, detBox);
}

double Combined1::match(const AbstractAppearanceModel *app) const
{
  const Combined1* other = dynamic_cast<const Combined1*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in Combined1::merge: the given model is not of this type");

  return _hsHist.match(&other->_hsHist) * _covModel.match(&other->_covModel) * _rgbHist.match(&other->_rgbHist);
}

void Combined1::merge(const AbstractAppearanceModel *app)
{
  const Combined1* other = dynamic_cast<const Combined1*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in Combined1::merge: the given model is not of this type");

  _hsHist.merge(&other->_hsHist);
  _covModel.merge(&other->_covModel);
  _rgbHist.merge(&other->_rgbHist);
}
