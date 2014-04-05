#pragma once
#ifndef EQUAL_BLOCK_APPEARANCE_H__
#define EQUAL_BLOCK_APPEARANCE_H__

#include <pal_appearance_models/appearance.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <iostream>
#include <numeric>
#include <stdexcept>

template <class T>
/**
 * @brief The EqualBlockAppearance class accepts an class derived from
 * AbstractAppearanceModel as it's template parameter and uses that model
 * on blocks created by splitting up the image into equal sized blocks.
 */
class EqualBlockAppearance : public AbstractAppearanceModel
{
  // T extends/implements AbstractAppearanceModel
  BOOST_STATIC_ASSERT((boost::is_base_of<AbstractAppearanceModel, T>::value));

protected:
  std::vector<T> _blockDescriptors;
  const int _horizontalRegions;
  const int _verticalRegions;

public:
  EqualBlockAppearance(int horizontalRegions = 3, int verticalRegions = 5);
  virtual void addAppearance(cv::Mat & img, const cv::Rect& detBox = cv::Rect());
  /**
   * @brief match
   * @param app
   * @return the mean of the block matches
   */
  virtual double match(const AbstractAppearanceModel *app) const;
  virtual void merge(const AbstractAppearanceModel* app);

public:
  /**
   * @brief produceRegions
   * @param input
   * @param horizontalRegions
   * @param verticalRegions
   * @return a vector containing regions of the input image
   */
  static std::vector<cv::Mat> produceRegions(const cv::Mat& input, int horizontalRegions, int verticalRegions);
};

template <class T>
EqualBlockAppearance<T>::EqualBlockAppearance(int horizontalRegions, int verticalRegions)
  :_horizontalRegions(horizontalRegions), _verticalRegions(verticalRegions)
{
  for(int i=0; i<_horizontalRegions*_verticalRegions; ++i)
    _blockDescriptors.push_back(T());
}

template <class T>
void EqualBlockAppearance<T>::addAppearance(cv::Mat &img, const cv::Rect &detBox)
{
  cv::Mat target = (detBox == cv::Rect()) ? img : img(detBox);
  std::vector<cv::Mat> regions = produceRegions(target, _horizontalRegions, _verticalRegions);

  for(unsigned int i=0; i<_blockDescriptors.size(); ++i)
    _blockDescriptors[i].addAppearance(regions[i]);
}

template <class T>
double EqualBlockAppearance<T>::match(const AbstractAppearanceModel* app) const
{
  const EqualBlockAppearance<T>* other = dynamic_cast<const EqualBlockAppearance<T>*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in BlockAppearance::match: the given model is not of this type");
  if(_blockDescriptors.size() != other->_blockDescriptors.size())
    throw std::runtime_error("Error in BlockAppearance::match: the given model is empty");

  std::vector<double> results;
  for(unsigned int i=0; i<_blockDescriptors.size(); ++i)
    results.push_back(_blockDescriptors[i].match(&(other->_blockDescriptors[i])));

  // returned the combined probability (should be quite small)
  //return std::accumulate(results.begin(), results.end(), double(1.0), std::multiplies<double>());
  return std::accumulate(results.begin(), results.end(), double(1.0))/results.size();
}

template <class T>
void EqualBlockAppearance<T>::merge(const AbstractAppearanceModel *app)
{
  const EqualBlockAppearance<T>* other = dynamic_cast<const EqualBlockAppearance<T>*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in BlockAppearance::merge: the given model is not of this type");
  if(_blockDescriptors.size() != other->_blockDescriptors.size())
    throw std::runtime_error("Error in BlockAppearance::merge: the given model doesn't contain enough data or not using the same parameters");

  // merge block by block
  for(unsigned int i=0; i<_blockDescriptors.size(); ++i)
    _blockDescriptors[i].merge(&(other->_blockDescriptors[i]));
}

template <class T>
std::vector<cv::Mat> EqualBlockAppearance<T>::produceRegions(const cv::Mat &input, int horizontalRegions, int verticalRegions)
{
  std::vector<cv::Mat> result;

  int rowStep = input.rows / verticalRegions;
  int colsStep = input.cols / horizontalRegions;

  for(int i=0; i<input.rows; i+=rowStep)
    for(int j=0; j<input.cols; j+=colsStep)
      result.push_back(input(cv::Range(i, std::min(i + rowStep, input.rows)),
                             cv::Range(j, std::min(j + colsStep, input.cols))));
  return result;
}
#endif
