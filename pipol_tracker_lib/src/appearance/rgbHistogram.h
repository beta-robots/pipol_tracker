#pragma once
#ifndef rgbHistogram__H_
#define rgbHistogram__H_

#include "appearance.h"
#include <deque>

class RGBHistogram : public AbstractAppearanceModel
{
protected:
    /** \brief Collection of Hue-Saturation histograms
    **/
    std::deque<cv::Mat> _appearances;

    /** \brief Max history size
     *
     * The maximum length of histograms to be stored in the collection
     *
    **/
    unsigned int _maxHistorySize;

public:
    RGBHistogram(int maxHistorySize = 10);

    virtual void addAppearance(cv::Mat & img, const cv::Rect &detBox = cv::Rect());

    virtual void merge(const AbstractAppearanceModel* app);

    virtual double match(const AbstractAppearanceModel *app) const;
};
#endif
