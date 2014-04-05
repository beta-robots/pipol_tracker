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


// #include <ros/ros.h>

#include "histogram.h"

namespace pal_vision_util
{
    void calcHSVHist(const cv::Mat& src, cv::MatND& histogram)
    {
        cv::Mat hsv;
        cv::cvtColor(src, hsv, CV_BGR2HSV);
        // Quantize the hue to 30 levels
        // and the saturation to 32 levels
        int hbins = 30, sbins = 32;
        int histSize[] = {hbins, sbins};
        // hue varies from 0 to 179, see cvtColor
        float hranges[] = { 0, 180 };
        // saturation varies from 0 (black-gray-white) to
        // 255 (pure spectrum color)
        float sranges[] = { 0, 256 };
        const float* ranges[] = { hranges, sranges };

        // we compute the histogram from the 0-th and 1-st channels
        int channels[] = {0, 1};

        cv::calcHist( &hsv, 1, channels, cv::Mat(), // do not use mask
                  histogram, 2, histSize, ranges,
                  true, // the histogram is uniform
                  false );
    }

    void calcHSVHist(const std::vector<cv::Mat>& src, std::vector<cv::MatND>& histograms)
    {
      histograms.clear();

      BOOST_FOREACH(const cv::Mat& mat, src)
      {
        cv::MatND hist;
        calcHSVHist(mat, hist);
        histograms.push_back(hist.clone());
      }
    }

    void calcHSVHist(const std::vector<cv::Mat>& src, cv::MatND& histogram)
    {
        std::vector<cv::Mat> hsv;
        BOOST_FOREACH(const cv::Mat& mat, src)
        {
          cv::Mat hsvMat;
          cv::cvtColor(mat, hsvMat, CV_BGR2HSV);
          hsv.push_back(hsvMat);
        }
        // Quantize the hue to 30 levels
        // and the saturation to 32 levels
        int hbins = 30, sbins = 32;
        int histSize[] = {hbins, sbins};
        // hue varies from 0 to 179, see cvtColor
        float hranges[] = { 0, 180 };
        // saturation varies from 0 (black-gray-white) to
        // 255 (pure spectrum color)
        float sranges[] = { 0, 256 };
        const float* ranges[] = { hranges, sranges };

        // we compute the histogram from the 0-th and 1-st channels
        int channels[] = {0, 1};
        cv::calcHist( hsv.data(), hsv.size(), channels, cv::Mat(), // do not use mask
                  histogram, 2, histSize, ranges,
                  true, // the histogram is uniform
                  false );
    }


    cv::Mat histogramImage(const cv::MatND& hist)
    {
        // Quantize the hue to 30 levels
        // and the saturation to 32 levels
        int hbins = 30, sbins = 32;
        double maxVal;
        cv::minMaxLoc(hist, 0, &maxVal, 0, 0);

        int scale = 10;
        cv::Mat histImg = cv::Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

        for( int h = 0; h < hbins; h++ )
            for( int s = 0; s < sbins; s++ )
            {
            float binVal = hist.at<float>(h, s);
            int intensity = cvRound(binVal*255/maxVal);
            cv::rectangle( histImg, cv::Point(h*scale, s*scale),
                           cv::Point( (h+1)*scale - 1, (s+1)*scale - 1),
                           cv::Scalar::all(intensity),
                           CV_FILLED );
        }
        return histImg;
    }

    void showHist(const cv::MatND& hist, bool waitKey)
    {
        cv::Mat histImg = histogramImage(hist);

        cv::namedWindow( "H-S Histogram", 1 );
        cv::imshow( "H-S Histogram", histImg );
        if ( waitKey )
          cv::waitKey();
    }

    void backProject(const cv::Mat& image,
                     const cv::MatND& hist,
                     int threshold,
                     cv::Mat& result,
                     int dilateIterations,
                     int dilateSize,
                     int erodeIterations,
                     int erodeSize)
    {
      cv::Mat hsv;
      cv::cvtColor(image, hsv, CV_BGR2HSV);
      // Quantize the hue to 30 levels
      // and the saturation to 32 levels
      // hue varies from 0 to 179, see cvtColor
      float hranges[] = { 0, 180 };
      // saturation varies from 0 (black-gray-white) to
      // 255 (pure spectrum color)
      float sranges[] = { 0, 256 };
      const float* ranges[] = { hranges, sranges };
      // we compute the histogram from the 0-th and 1-st channels
      int channels[] = {0, 1};

      //back-projection takes every pixel in hsv and checks in what bin of target_hist belongs to. Then, the value of
      //the bin is stored in backProject in the same coordinates as the pixel
      cv::calcBackProject(&hsv, 1, channels, hist, result, ranges, 1, true);

      //normalization is not necessary as the histogram target_hist was already normalized
      //cv::normalize(backProject, backProject, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
      cv::threshold(result, result, threshold, 255, CV_THRESH_BINARY);

      cv::Mat tmp1, tmp2;

      if ( dilateIterations == 0 && erodeIterations == 0 )
          return;

      tmp1 = result;

      if ( dilateIterations > 0 )
      {
          cv::dilate(tmp1, erodeIterations == 0 ? result: tmp2,
                     cv::Mat::ones(dilateSize, dilateSize, CV_8UC1),
                     cv::Point(-1, -1), dilateIterations);
      }

      if ( erodeIterations > 0 )
      {
          cv::erode(dilateIterations == 0 ? tmp1 : tmp2, result,
                    cv::Mat::ones(erodeSize, erodeSize, CV_8UC1),
                    cv::Point(-1, -1), erodeIterations);
      }

    }

}


