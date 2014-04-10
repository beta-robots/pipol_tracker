#include "rgbHistogram.h"

RGBHistogram::RGBHistogram(int maxHistorySize): _maxHistorySize(maxHistorySize)
{
}

void RGBHistogram::addAppearance(cv::Mat & img, const cv::Rect &detBox)
{
    const cv::Mat target = (detBox == cv::Rect()) ? img : img(detBox);

    //compute a simple 3channel rgb histogram
    int histSize[] = {32, 32, 32};
    float range[] = {0, 255};
    const float* ranges[] = {range, range, range};
    int channels[] = {0, 1, 2};

    cv::Mat currHist;
    cv::calcHist(&target, 1, channels, cv::Mat(),
                 currHist, 3, histSize, ranges);

    _appearances.push_back(currHist);
    if(_appearances.size() > _maxHistorySize)
        _appearances.pop_front();

    //cv::normalize(hist, hist);
}

void RGBHistogram::merge(const AbstractAppearanceModel* app)
{
    const RGBHistogram* other = dynamic_cast<const RGBHistogram*>(app);
    if(other == NULL)
        throw std::runtime_error("Error in RGBHistogram::merge: the given model is not of this type");
    if(other->_appearances.size() != 1)
        throw std::runtime_error("Error in RGBHistogram::merge: the given model contains more appearances");

    _appearances.push_back(other->_appearances[0]);
    if(_appearances.size() > _maxHistorySize)
        _appearances.pop_front();
}

double RGBHistogram::match(const AbstractAppearanceModel* app) const
{
    const RGBHistogram* other = dynamic_cast<const RGBHistogram*>(app);
    if(other == NULL)
        throw std::runtime_error("Error in RGBHistogram::match: the given model is not of this type");
    if(other->_appearances.empty())
        throw std::runtime_error("Error in RGBHistogram::match: the given model is empty");

    std::vector<double> results;
    for(unsigned int i=0; i<_appearances.size(); ++i)
    {
      // not the most elegant thing to do but solves the problem of very small negative values
      const double histSim = (cv::compareHist(this->_appearances[i], other->_appearances[0], CV_COMP_CORREL));  // assuming that the other model only has 1 appearance
      results.push_back( (histSim < 0) ? 0.0 : histSim );
    }
    return *std::max_element(results.begin(), results.end());
}
