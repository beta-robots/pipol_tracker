#pragma once
#ifndef __SIMPLEPATCHMODEL__H__
#define __SIMPLEPATCHMODEL__H__

#include "appearance.h"
#include <deque>

class SimplePatchModel : public AbstractAppearanceModel
{
public:
  SimplePatchModel(int maxHistorySize = 5);
  void addAppearance(cv::Mat& img, const cv::Rect& detBox = cv::Rect());
  virtual void merge(const AbstractAppearanceModel* app);
  double match(const AbstractAppearanceModel* app) const;
  virtual void print() const;

protected:
  std::deque<cv::Mat> _appearances;
  unsigned int _maxHistorySize;
};

#endif
