#pragma once
#ifndef appearance_H
#define appearance_H

#include <iostream>
#include <cmath>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

/** \brief Base class for appearance models
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
  //AbstractAppearanceModel();
  //virtual ~AbstractAppearanceModel();

  /** \brief Extract features from an image.
  **/
  virtual void extract(cv::Mat& img, const cv::Rect_<int>& detBox = cv::Rect()) = 0;

  /** \brief
  **/
  virtual void update() = 0; // I think this should be part of extract.

  /** \brief Match two appearances.
   *  The problem with this is that we will have polymorphic attach while we shouldn't.
   * "like Current" in Eiffel: http://tecomp.sourceforge.net/index.php?file=doc/papers/lang/catcall_solution.txt#chapter_2
  **/
  virtual double match(const AbstractAppearanceModel* app) const = 0;

  /** \brief Print a text representation of the actual model.
  **/
  virtual void print() const = 0;
};
#endif
