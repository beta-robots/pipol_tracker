#include "simplepatchmodel.h"
#include <iostream>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>

SimplePatchModel::SimplePatchModel()
{
}

void SimplePatchModel::extract(cv::Mat& img, const cv::Rect_<int>& detBox)
{
  if(detBox == cv::Rect())
    this->appearances.push_back(img);
  else
    this->appearances.push_back(img(detBox));
}

void SimplePatchModel::update()
{
}

double SimplePatchModel::match(const AbstractAppearanceModel* app) const 
{
  const SimplePatchModel* other = (SimplePatchModel*)app; // const_cast<SimplePatchModel*>(app);
  std::vector<double> confidences;

  double tmpMax, tmpMin;
  cv::Point tmpMaxIdx, tmpMinIdx;
  cv::Mat convResult;

  for(unsigned int i=0; i<this->appearances.size(); ++i)
  {
//    std::cout << other->ID << " " << tmpMin << std::endl;
//    std::cout << "[" << other->appearances[0].rows << ", " << other->appearances[0].cols << "]"
//              << " --> "
//              << "[" << this->appearances[i].rows << ", " << this->appearances[i].cols << "]"
//              << std::endl;
    // if the template if bigger than the image we are searching in, rescale it to 2/3rd
    if(other->appearances[0].rows < this->appearances[i].rows || other->appearances[0].cols < this->appearances[i].cols)
    {
      cv::Mat resized;
//      double sizeX = other->appearances[0].rows*(2.0/3.0)/(this->appearances[i].rows);
//      double sizeY = sizeX*(this->appearances[i].cols/this->appearances[i].rows);
//      cv::resize(this->appearances[i], resized, cv::Size(sizeX, sizeY)); // resize pattern to 2/3 of image width
      cv::resize(this->appearances[i], resized, other->appearances[0].size());
//      std::cout << "Resized:"
//                << "[" << resized.rows << ", " << resized.cols << "]"
//                << std::endl;
      cv::matchTemplate(other->appearances[0], resized, convResult, CV_TM_SQDIFF_NORMED); // do the image correlation step
    }
    else
    {
      cv::matchTemplate(other->appearances[0], this->appearances[i], convResult, CV_TM_SQDIFF_NORMED); // do the image correlation step
    }

    cv::minMaxLoc(convResult, &tmpMin, &tmpMax, &tmpMinIdx, &tmpMaxIdx); // look for the max/min
    confidences.push_back(tmpMax); // administrate the significant value as matching result for appearance
  }

  // search for the best match in the confidences vector
  // *(std::min_element(confidences.begin(), confidences.end())) gives the smallest element in the vector
  //
  return 1.0-(*(std::min_element(confidences.begin(), confidences.end())));
}

void SimplePatchModel::print() const
{
  std::cout << "Model contains " << appearances.size() << " appearances of target." << std::endl;
}
