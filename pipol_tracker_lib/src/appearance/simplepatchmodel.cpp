#include "simplepatchmodel.h"

SimplePatchModel::SimplePatchModel(int maxHistorySize): _maxHistorySize(maxHistorySize)
{
}

void SimplePatchModel::addAppearance(cv::Mat& img, const cv::Rect &detBox)
{
  cv::Mat target = (detBox == cv::Rect()) ? img : img(detBox);
  _appearances.push_back(target);
  if(_appearances.size() > _maxHistorySize)
      _appearances.pop_front();
}

void SimplePatchModel::merge(const AbstractAppearanceModel* app)
{
    const SimplePatchModel* other = dynamic_cast<const SimplePatchModel*>(app);
    if(other == NULL)
        throw std::runtime_error("Error in SimplePatchModel::merge: the given model is not of this type");
    if(other->_appearances.size() != 1)
        throw std::runtime_error("Error in SimplePatchModel::merge: the given model contains more appearances");

    _appearances.push_back(other->_appearances[0]);
    if(_appearances.size() > _maxHistorySize)
        _appearances.pop_front();
}

double SimplePatchModel::match(const AbstractAppearanceModel* app) const 
{
  const SimplePatchModel* other = dynamic_cast<const SimplePatchModel*>(app);
  if(other == NULL)
      throw std::runtime_error("Error in SimplePatchModel::match: the given model is not of this type");
  if(other->_appearances.empty())
      throw std::runtime_error("Error in SimplePatchModel::match: the given model is empty");

  std::vector<double> confidences;

  double tmpMax, tmpMin;
  cv::Point tmpMaxIdx, tmpMinIdx;
  cv::Mat convResult;

  for(unsigned int i=0; i<this->_appearances.size(); ++i)
  {
    // if the template if bigger than the image we are searching in, rescale it
    if(other->_appearances[0].rows < this->_appearances[i].rows || other->_appearances[0].cols < this->_appearances[i].cols)
    {
      cv::Mat resized;
      cv::resize(this->_appearances[i], resized, other->_appearances[0].size());
      cv::matchTemplate(other->_appearances[0], resized, convResult, CV_TM_SQDIFF_NORMED); // do the image correlation step
    }
    else
    {
      cv::matchTemplate(other->_appearances[0], this->_appearances[i], convResult, CV_TM_SQDIFF_NORMED); // do the image correlation step
    }

    cv::minMaxLoc(convResult, &tmpMin, &tmpMax, &tmpMinIdx, &tmpMaxIdx); // look for the max/min
    confidences.push_back(tmpMax); // administrate the significant value as matching result for appearance
  }
  return 1.0-(*(std::min_element(confidences.begin(), confidences.end())));
}

void SimplePatchModel::print() const
{
  std::cout << "Model contains " << _appearances.size() << " appearances of target." << std::endl;
}
