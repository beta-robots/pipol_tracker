#pragma once
#ifndef COMBINED_1_H__
#define COMBINED_1_H__

#include "appearance.h"
#include "hsHistogram.h"
#include "covariancemodel.h"
#include "rgbHistogram.h"

class Combined1 : public AbstractAppearanceModel
{
protected:
  HsHistogram _hsHist;
  CovarianceModel _covModel;
  RGBHistogram _rgbHist;

public:
  virtual void addAppearance(cv::Mat& img, const cv::Rect& detBox = cv::Rect());
  virtual double match(const AbstractAppearanceModel* app) const;
  virtual void merge(const AbstractAppearanceModel* app);
};

#endif
