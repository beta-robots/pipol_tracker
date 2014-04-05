#pragma once
#ifndef hsHistogram_H
#define hsHistogram_H

#include "appearance.h"

const unsigned int NUM_BINS_PER_CHANNEL_DEFAULT = 200;

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
  /** \brief Number of bins per channel
   *
   * Number of bins per channel
   *
  **/
  unsigned int numBins;

  /** \brief Hue-Saturation histogram
   *
   * Hue-Saturation histogram
   *
  **/
  cv::Mat hsHist;

  /** \brief Hue-Saturation histogram accumulated
   *
   * Hue-Saturation histogram modelling the person appearance as a long-term "learnt" model
   *
  **/
  cv::Mat hsHistLT;

  /** \brief Weighting mask
   *
   * mask of values in [0,1] weighting the more rellevant areas of a bounding box to be considered by histogram calculation
   *
  **/
  cv::Mat wMask;

  /** \brief hsv image of the current frame
   *
   * hsv image of the current frame
   *
  **/
  cv::Mat imgHsv;

  /** \brief separated channels HSV
   *
   * separated channels HSV
   *
  **/
  cv::Mat imgHsvSplit[3];

public:
  /** \brief Default constructor
   *
   * Default constructor
   *
  **/
  HsHistogram();

  /** \brief Constructor with arguments
   *
   * Constructor with arguments. Argument is the number of boxes
   *
  **/
  HsHistogram(unsigned int nB);

  /** \brief Default destructor
   *
   * Default destructor
   *
  **/
  virtual ~HsHistogram();

  /** \brief Extracts an appearance model
   *
   * Extracts an appearance model.
   * The resulting model is set to this->hsHist
   *
  **/
  virtual void extract(cv::Mat & img, const cv::Rect_<int> & detBox);

  /** \brief Updates the appearance model
   *
   * Updates the appearance model by integrating this->hsHist to long-term model
   * The resulting model is set to this->hsHistLT
   *
  **/
  virtual void update();

  /** \brief Match this with app
   *
   * Matching function between "this" model and app
   * Returns a value in [0,1]
   *
  **/
  virtual double match(const AbstractAppearanceModel *app) const;

  /** \brief Prints debugging data to stdout
   *
   * Prints debugging data to stdout
   *
  **/
  virtual void print() const;

  /** \brief Gets the mask image
   *
   * Gets the mask image
   *
  **/
  cv::Mat & getwMask();

  /** \brief Gets the HSV image
   *
   * Gets the HSV image
   *
  **/
  cv::Mat & getImgHsv();

  /** \brief Gets a single channel of HSV image
   *
   * Gets a single channel of HSV image
   *
  **/
  cv::Mat & getImgHsv(int chId);

  /** \brief Gets the HS histogram image
   *
   * Gets the HS histogram image
   *
  **/
  cv::Mat & getHsHist();
};
#endif
