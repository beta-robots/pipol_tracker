#pragma once
#ifndef hsHistogram_H
#define hsHistogram_H

#include <deque>
#include "appearance.h"
#include "histogram.h"

/** \brief Class for hsHistogram appearance model
*
* Class for hsHistogram appearance model.
* The model is based on a computing the Hue-Saturation histogram of the bounding box, which is weighted by a mask,
* giving more importance to pixels close to central vertical line
* 
**/
class HsHistogram : public AbstractAppearanceModel
{
protected:
  /** \brief Collection of histograms
  **/
  std::deque<cv::Mat> _appearances;

  /** \brief Max history size
   *
   * The maximum length of histograms to be stored in the collection
   *
  **/
  unsigned int _maxHistorySize;

public:

  HsHistogram(int maxHistorySize = 10);
  virtual void addAppearance(cv::Mat & img, const cv::Rect &detBox = cv::Rect());
  virtual void merge(const AbstractAppearanceModel* app);
  virtual double match(const AbstractAppearanceModel *app) const;

  /** \brief Gets the HS histogram image
   *
   * Gets the HS histogram image
   *
  **/
  cv::Mat & getHsHist();

};
#endif
