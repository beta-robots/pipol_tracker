
#ifndef personAppearance_H
#define personAppearance_H

#include <cmath>
#include <iostream>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

const unsigned int NUM_OF_BOXES_DEFAULT = 2;
const unsigned int NUM_BINS_PER_CHANNEL_DEFAULT = 100;

struct coloredBox
{
        cv::Vec3b color;
        cv::Rect_<int> box;
};

class CpersonAppearance
{
        protected:
               /**
                * 
                * Vector of boxes modelling the person appearance
                * 
                **/
               std::vector<coloredBox> boxSet;
               
               /**
                * 
                * mask of values in [0,1] weighting the more rellevant areas of a bounding box to be considered by histogram calculation
                * 
                **/
               cv::Mat wMask;
               
               /**
                * 
                * Original image with flood regions overpainted
                * 
                **/
               cv::Mat floodImage;
               
               /**
                * 
                * Image of the appearance color box model
                * 
                **/
               cv::Mat boxAppImage;
               
               /**
                * 
                * hsv image of the current frame
                * 
                **/
               cv::Mat imgHsv;
               
               /**
                * 
                * sparated channels HSV
                * 
                **/
               cv::Mat imgHsvSplit[3];
               
               /**
                * 
                * Hue-Saturation histogram
                * 
                **/
               cv::Mat hsHist;
               
        public:
                CpersonAppearance();
                ~CpersonAppearance();
                void extractAppearanceBoxes(cv::Mat & img, const cv::Rect_<int> & detBox, unsigned int numBoxes = NUM_OF_BOXES_DEFAULT);
                void extractAppearanceHSVhist(cv::Mat & img, const cv::Rect_<int> & detBox, unsigned int numBins = NUM_BINS_PER_CHANNEL_DEFAULT);
                double matchAppearance(CpersonAppearance & app) const;
                void updateAppearance();
                cv::Mat & getFloodImage();
                cv::Mat & getBoxImage();
                cv::Mat & getwMask(); 
                cv::Mat & getImgHsv();
                cv::Mat & getImgHsv(int chId);
                cv::Mat & getHsHist();
                void printAppearance() const;
};

#endif
