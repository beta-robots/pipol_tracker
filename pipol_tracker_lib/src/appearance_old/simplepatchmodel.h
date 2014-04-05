#pragma once
#ifndef __SIMPLEPATCHMODEL__H__
#define __SIMPLEPATCHMODEL__H__

#include "appearance.h"
#include <vector>
#include <opencv2/core/core.hpp>

class SimplePatchModel : public AbstractAppearanceModel
{
public:
  SimplePatchModel();
  void extract(cv::Mat& img, const cv::Rect_<int>& detBox = cv::Rect());
  void update();
  double match(const AbstractAppearanceModel* app) const;
  void print() const;

private:
  std::vector<cv::Mat> appearances;
};

#endif
