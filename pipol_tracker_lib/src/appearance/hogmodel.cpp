#include "hogmodel.h"

void HOGModel::addAppearance(cv::Mat & img, const cv::Rect &detBox)
{
  cv::Mat target = (detBox == cv::Rect()) ? img : img(detBox);
  cv::cvtColor(target, target, CV_RGB2GRAY); // image to grayscale

  // !!!!! assumes that the person is in the middle of the image
  //target = cropBy(target, 60, 40); // crop the image to the center 60% height and 40% width

  cv::HOGDescriptor hog; // by default: winSize(64,128), blockSize(16,16), blockStride(8,8),
  hog.cellSize = cv::Size(8,8);
  hog.blockSize = cv::Size(16,16);
  std::vector<float> descriptors;

  cv::resize(target, target, cv::Size(96, 192)); // 1 / 2
  hog.compute(target, descriptors);

  _hogHist.create(descriptors.size(), 1, CV_32FC1);
  for(unsigned int i=0;i<descriptors.size();i++)
    _hogHist.at<float>(i,0)=descriptors.at(i);
}

void HOGModel::merge(const AbstractAppearanceModel* app)
{
  const HOGModel* other = dynamic_cast<const HOGModel*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in HOGModel::merge: the given model is not of this type");
  if(other->_hogHist.empty())
    throw std::runtime_error("Error in HOGModel::merge: the given model is empty");

  _hogHist = other->_hogHist;
}

double HOGModel::match(const AbstractAppearanceModel* app) const
{
  const HOGModel* other = dynamic_cast<const HOGModel*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in HOGModel::match: the given model is not of this type");
  if(other->_hogHist.empty())
    throw std::runtime_error("Error in HOGModel::match: the given model is empty");

  const double result = cv::compareHist(this->_hogHist, other->_hogHist, CV_COMP_CORREL);

  return (result < 0 ? 0 : result);
}
