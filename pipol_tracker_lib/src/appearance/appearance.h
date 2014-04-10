#pragma once
#ifndef appearance_H
#define appearance_H

#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <assert.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

/** @brief Base class for appearance models
*
* Base class for appearance models.
* This class only defines pure virtual methods
* 
**/
class AbstractAppearanceModel
{
public:
  int ID;
public:
  virtual ~AbstractAppearanceModel(){}

  /** @brief Extract features from an image then update the model with it.
   */
  virtual void addAppearance(cv::Mat& img, const cv::Rect& detBox = cv::Rect()) = 0;

  /**
   * @brief Crop an image to it's inner subimage with rowPerc% of rowsize and colsPerc% of column size of img.
   * @param img
   * @param rowPerc
   * @param colsPerc
   * @return the cropped image
   */
  cv::Mat cropBy(cv::Mat& img, int rowPerc, int colsPerc);

  /**
   * @brief Match the parameter appearance model to the current appearance model.
   * @param app
   * @return probability of matching should be in the interval [0,1]
   */
  virtual double match(const AbstractAppearanceModel* app) const = 0;

  /**
   * @brief Merge the parameter appearance model into the current one.
   *        Only single-appearance model is accepted, no history copy is done here.
   * @param app
   */
  virtual void merge(const AbstractAppearanceModel* app) = 0;

  /** @brief Print a text representation of the actual model.
  **/
  virtual void print() const {}
};
#endif
