#pragma once
#ifndef COMMON_BLOCK_APPEARANCE_H__
#define COMMON_BLOCK_APPEARANCE_H__

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
 * @brief The CommonBlockAppearance class accepts an class derived from
 * AbstractAppearanceModel as it's template parameter and uses that model
 * on blocks created using a commonly used image splitting setup.
 * The blocks are as follows:
 *  | x x |  | x 0 |  | 0 x |  | x x |  | 0 0 |
 *  | x x |, | x 0 |, | 0 x |, | 0 0 |, | x x |
 */
class CommonBlockAppearance : public AbstractAppearanceModel
{
  // T extends/implements AbstractAppearanceModel
  BOOST_STATIC_ASSERT((boost::is_base_of<AbstractAppearanceModel, T>::value));

protected:
  std::vector<T> _blockDescriptors;

public:
  virtual void addAppearance(cv::Mat& img, const cv::Rect &detBox = cv::Rect());
  /**
   * @brief match
   * @param app
   * @return the product of the match results of individual blocks
   */
  virtual double match(const AbstractAppearanceModel *app) const;
  virtual void merge(const AbstractAppearanceModel* app);

public:
  /**
   * @brief produceRegions
   * @param input
   * @return a vector containing regions of the input image
   */
  static std::vector<cv::Mat> produceRegions(const cv::Mat& input);
};

template <class T>
void CommonBlockAppearance<T>::addAppearance(cv::Mat &img, const cv::Rect &detBox)
{
  cv::Mat target = (detBox == cv::Rect()) ? img : img(detBox);
  std::vector<cv::Mat> regions = produceRegions(target);

  // if the object is not initialized yet with model objects, do so
  if(_blockDescriptors.size() == 0)
  {
    for(unsigned int i=0; i<regions.size(); ++i)
      _blockDescriptors.push_back(T());
  }

  for(unsigned int i=0; i<_blockDescriptors.size(); ++i)
    _blockDescriptors[i].addAppearance(regions[i]);
}

template <class T>
double CommonBlockAppearance<T>::match(const AbstractAppearanceModel* app) const
{
  const CommonBlockAppearance<T>* other = dynamic_cast<const CommonBlockAppearance<T>*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in BlockAppearance::match: the given model is not of this type");
  if(_blockDescriptors.size() != other->_blockDescriptors.size())
    throw std::runtime_error("Error in BlockAppearance::match: the given model is empty");

  std::vector<double> results;
  for(unsigned int i=0; i<_blockDescriptors.size(); ++i)
    results.push_back(_blockDescriptors[i].match(&(other->_blockDescriptors[i])));

  // returned the combined probability (should be quite small)
  return std::accumulate(results.begin(), results.end(), double(1.0), std::multiplies<double>());
}

template <class T>
void CommonBlockAppearance<T>::merge(const AbstractAppearanceModel *app)
{
  const CommonBlockAppearance<T>* other = dynamic_cast<const CommonBlockAppearance<T>*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in BlockAppearance::merge: the given model is not of this type");
  if(_blockDescriptors.size() != other->_blockDescriptors.size())
    throw std::runtime_error("Error in BlockAppearance::merge: the given model doesn't contain enough data or not using the same parameters");

  // merge block by block
  for(unsigned int i=0; i<_blockDescriptors.size(); ++i)
    _blockDescriptors[i].merge(&(other->_blockDescriptors[i]));
}

template <class T>
std::vector<cv::Mat> CommonBlockAppearance<T>::produceRegions(const cv::Mat &input)
{
  std::vector<cv::Mat> result;

  const int rows = input.rows;
  const int cols = input.cols;
  // | x x | full image
  // | x x |
  result.push_back(input);
  // | x 0 |
  // | x 0 |
  result.push_back(input(cv::Range(0, rows),
                         cv::Range(0, cols/2)));
  // | 0 x |
  // | 0 x |
  result.push_back(input(cv::Range(0, rows),
                         cv::Range(cols/2, cols)));
  // | x x |
  // | 0 0 |
  result.push_back(input(cv::Range(0, rows/2),
                         cv::Range(0, cols)));
  // | 0 0 |
  // | x x |
  result.push_back(input(cv::Range(rows/2, rows),
                         cv::Range(0, cols)));
  return result;
}
#endif
