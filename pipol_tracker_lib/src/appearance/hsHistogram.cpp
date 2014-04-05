#include "hsHistogram.h"

HsHistogram::HsHistogram(int maxHistorySize): _maxHistorySize(maxHistorySize)
{
}

void HsHistogram::addAppearance(cv::Mat & img, const cv::Rect &detBox)
{
  const cv::Mat target = (detBox == cv::Rect()) ? img : img(detBox);

  cv::Mat currHist;
  pal_vision_util::calcHSVHist(target, currHist);

  //store the current histogram while keeping the last _max_history_size histograms
  _appearances.push_back(currHist);
  if(_appearances.size() > _maxHistorySize)
    _appearances.pop_front();
}

void HsHistogram::merge(const AbstractAppearanceModel* app)
{
  const HsHistogram* other = dynamic_cast<const HsHistogram*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in HsHistogram::merge: the given model is not of this type");
  if(other->_appearances.size() != 1)
    throw std::runtime_error("Error in HsHistogram::merge: the given model contains more appearances");

  _appearances.push_back(other->_appearances[0]);
  if(_appearances.size() > _maxHistorySize)
    _appearances.pop_front();
}

double HsHistogram::match(const AbstractAppearanceModel* app) const
{
  const HsHistogram* other = dynamic_cast<const HsHistogram*>(app);
  //"if" below added by ACM
  if ( (other->_appearances.empty()) || (this->_appearances.empty()) )
  {
        std::cout << "Error in HsHistogram::match: the given or this model are empties" << std::endl;
        return -1;
  }

  if(other == NULL)
    throw std::runtime_error("Error in HsHistogram::match: the given model is not of this type");
  if(other->_appearances.empty())
  {
    throw std::runtime_error("Error in HsHistogram::match: the given model is empty");
  }

  std::vector<double> results;
  for(unsigned int i=0; i<_appearances.size(); ++i)
  {
    // not the most elegant thing to do but solves the problem of very small negative values
    const double histSim = (cv::compareHist(this->_appearances[i], other->_appearances[0], CV_COMP_CORREL));  // assuming that the other model only has 1 appearance
    results.push_back( (histSim < 0) ? 0.0 : histSim );
  }
  return *std::max_element(results.begin(), results.end());
}

cv::Mat & HsHistogram::getHsHist()
{
  return _appearances.back();
}
