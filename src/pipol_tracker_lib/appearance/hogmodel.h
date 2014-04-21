#pragma once
#ifndef HOGMODEL_H__
#define HOGMODEL_H__

#include "appearance.h"
#include <opencv2/objdetect/objdetect.hpp>

class HOGModel : public AbstractAppearanceModel
{
protected:
  cv::Mat _hogHist;

public:
  virtual void addAppearance(cv::Mat& img, const cv::Rect &detBox = cv::Rect());
  virtual void merge(const AbstractAppearanceModel* app);
  virtual double match(const AbstractAppearanceModel *app) const;
};
#endif
