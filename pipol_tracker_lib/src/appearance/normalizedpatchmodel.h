#pragma once
#ifndef __NORMALIZED_PATCH_MODEL__H__
#define __NORMALIZED_PATCH_MODEL__H__

#include <algorithm>
#include <numeric>
#include <functional>
#include "appearance.h"

class NormalizedPatchModel : public AbstractAppearanceModel
{
public:

  typedef std::vector<float> patchVector;

  NormalizedPatchModel();
  void addAppearance(cv::Mat& img, const cv::Rect& detBox = cv::Rect());
  virtual void merge(const AbstractAppearanceModel* app);
  double match(const AbstractAppearanceModel* app) const;
  void print() const;

protected:

  void imgToVector(const cv::Mat& img, patchVector& v);
  float mean(const std::vector<float>& v);
  void meanAndStdDev(const std::vector<float>& v, float& m, float& s);
  float stdDev(const std::vector<float>& v, float mean);
  void normalize(const std::vector<float>& v, std::vector<float>& normalizedV);
  static float patchSimilarity(const patchVector& v1, const patchVector& v2);

private:  

  std::vector<patchVector> _appearances;
  int _normalizedWidth, _normalizedHeight;

};

#endif
