#pragma once
#ifndef OLDAM_H__
#define OLDAM_H__

#include "appearance.h"
#include "hsHistogram.h"
#include "hogmodel.h"
#include "covariancemodel.h"

class Oldam : public AbstractAppearanceModel
{
protected:
  HsHistogram _hsHist;
  HOGModel _hog;
  CovarianceModel _cov;

  static const double w[];

public:
  virtual void addAppearance(cv::Mat& img, const cv::Rect& detBox = cv::Rect());
  virtual double match(const AbstractAppearanceModel* app) const;
  virtual void merge(const AbstractAppearanceModel* app);
};
#endif
