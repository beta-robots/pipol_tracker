/*
* Software License Agreement (Modified BSD License)
*
* Copyright (c) 2012, PAL Robotics, S.L.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of PAL Robotics, S.L. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/** \author Bence Magyar */

#ifndef _PAL_HISTOGRAM_UTIL_H_
#define _PAL_HISTOGRAM_UTIL_H_

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace pal_vision_util
{
    /**
* @brief calcHSVHist compute and return a Hue-Saturation histogram of a given image
* @param src
* @param histogram
*/
    void calcHSVHist(const cv::Mat& src, cv::MatND& histogram);

    /**
* @brief calcHSVHist compute a Hue-Saturation for each given image
* @param src
* @param histogram
*/
    void calcHSVHist(const std::vector<cv::Mat>& src, std::vector<cv::MatND>& histograms);

    /**
* @brief calcHSVHist compute and return Hue-Saturation histogram of a given set of images
* @param src
* @param histograms
*/
    void calcHSVHist(const std::vector<cv::Mat>& src, cv::MatND& histogram);

    //transform a histogram into an image
    cv::Mat histogramImage(const cv::MatND& hist);

    /**
* @brief showHist visualize a Hue-Saturation histogram
* @param hist
* @param waitKey if true, the user has to press a key on the image to continue
*/
    void showHist(const cv::MatND& hist, bool waitKey = true);

    /**
* @brief backProject generate a binary image resulting of thresholding the per-pixel backprojection of a H-S histogram
* to a color image
* @param image the input BGR image
* @param hist the input H-S histogram
* @param result the resulting 1 channel 8 bit image corresponding so that a pixel is 255 if the result of its back-projection is > threshold,
* otherwise the pixel is set to 0.
*/
    void backProject(const cv::Mat& image,
                     const cv::MatND& hist,
                     int threshold,
                     cv::Mat& result,
                     int dilateIterations = 0,
                     int dilateSize = 3,
                     int erodeIterations = 0,
                     int erodeSize = 3);
}

#endif // _PAL_HISTOGRAM_UTIL_H_


