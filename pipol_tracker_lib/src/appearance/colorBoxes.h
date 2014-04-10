#pragma once
#ifndef colorBoxes_H
#define colorBoxes_H

#include "appearance.h"

const unsigned int NUM_OF_BOXES_DEFAULT = 5;

/** \brief A single color box 
 *
 * A single color box:
 *      RGB color
 *      Bounding box
 *
 **/
struct coloredBox
{
  cv::Vec3b color;
  cv::Rect_<int> box;
};

/** \brief Class for colorBoxes appearance model
*
* Class for colorBoxes appearance model.
* The model is based on a "stack" of boxes representing main colors of the human bounding box.
* 
**/
class ColorBoxes : public AbstractAppearanceModel
{
protected:
  /** \brief Number of boxes
   *
   * Number of boxes
   *
  **/
  unsigned int numBoxes;

  /** \brief Vector of boxes for a single frame
   *
   * Vector of boxes modelling the person appearance just for a single frame
   *
  **/
  std::vector<coloredBox> boxSet;

  /** \brief Vector of boxes accumulated
   *
   * Vector of boxes modelling the person appearance as a long-term "learnt" model
   *
  **/
  std::vector<coloredBox> boxSetLT;

  /** \brief Flooded image
   *
   * Original image with flood regions overpainted
   *
  **/
  cv::Mat floodImage;

  /** \brief Image of box stack
   *
   * Image of the appearance color box model
   *
   **/
  cv::Mat boxAppImage;

public:
  /** \brief Default constructor
   *
   * Default constructor
   *
  **/
  ColorBoxes();

  /** \brief Constructor with arguments
   *
   * Constructor with arguments. Argument is the number of boxes
   *
  **/
  ColorBoxes(unsigned int nB);

  /** \brief Default destructor
   *
   * Default destructor
   *
   **/
  virtual ~ColorBoxes();

  /** \brief Extracts an appearance model
   *
   * Extracts an appearance model.
   * The resulting model is set to this->boxSet
   *
  **/
  virtual void addAppearance(cv::Mat & img, const cv::Rect &detBox);


  virtual void merge(const AbstractAppearanceModel* app);

  /** \brief Match this with app
   *
   * Matching function between "this" model and app
   * Returns a value in [0,1]
   *
  **/
  virtual double match(const AbstractAppearanceModel* app) const;

  /** \brief Prints debugging data to stdout
   *
   * Prints debugging data to stdout
   *
  **/
  virtual void print() const;

  /** \brief Gets flood image
   *
   * Gets flood image
   *
   **/
  cv::Mat & getFloodImage();

  /** \brief Gets box image
   *
   * Gets box image
   *
  **/
  cv::Mat & getBoxImage();

};
#endif
