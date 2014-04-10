#include "personAppearance.h"

CpersonAppearance::CpersonAppearance()
{

}

CpersonAppearance::~CpersonAppearance()
{
        
}

void CpersonAppearance::extractAppearanceBoxes(cv::Mat & img, const cv::Rect_<int> & detBox, unsigned int numBoxes)
{
        cv::Vec3b pixel3u;
        cv::Scalar_<uchar> pixel4u;
        cv::Rect_<int> subBox,floodBox;
        unsigned int ii;
 
        //clones original image
        floodImage = img.clone();
        
        //draws the bounding box on the original image
        cv::rectangle(floodImage, detBox, cv::Scalar_<uchar>(255,0,0), 5);
        
        //initialize subBox
        subBox.x = detBox.x;
        subBox.width = detBox.width;
        subBox.height = detBox.height/numBoxes;
        
        //initialize boxSet
        boxSet.resize(numBoxes);
        
        //main loop: for each box
        for (ii=0; ii<numBoxes; ii++)
        {
                //computes y coordinate of the subBox
                subBox.y = detBox.y+ii*detBox.height/numBoxes;
                
                //draw the associated bounding box
                cv::rectangle(floodImage, subBox, cv::Scalar(255,0,0), 5);
                
                //pick central pixel color
                pixel3u = floodImage.at<cv::Vec3b>(cv::Point(subBox.x+subBox.width/2,subBox.y+subBox.height/2));
                
                //flood
                pixel4u = cv::Scalar_<uchar>(pixel3u[0],pixel3u[1],pixel3u[2],0);
                cv::floodFill(floodImage, cv::Point(subBox.x+subBox.width/2,subBox.y+subBox.height/2), pixel4u, &floodBox, cv::Scalar_<uchar>(5, 5, 5, 1), cv::Scalar_<uchar>(5, 5, 5, 1)); 
                boxSet[ii].box = floodBox;
                boxSet[ii].color[0] = pixel4u[0];
                boxSet[ii].color[1] = pixel4u[1];
                boxSet[ii].color[2] = pixel4u[2];
                
                //debugging: draw subBox central rectangle
                cv::rectangle(floodImage,cv::Rect_<int>(subBox.x+subBox.width/2-15,subBox.y+subBox.height/2-15,30,30),cv::Scalar(0,0,255), 5);                
        }        
        
        //draw appearance model
        boxAppImage = cv::Mat(floodImage.rows,floodImage.cols,CV_8UC3,cv::Scalar(255,255,255));
        cv::rectangle(boxAppImage, detBox, cv::Scalar(255,0,0), 5);
        for (ii=0; ii<numBoxes; ii++)        
        {
                pixel4u = cv::Scalar_<uchar>(boxSet[ii].color[0],boxSet[ii].color[1],boxSet[ii].color[2],1);
                boxAppImage(boxSet[ii].box) = pixel4u;
        }
}

void CpersonAppearance::extractAppearanceHSVhist(cv::Mat & img, const cv::Rect_<int> & detBox, unsigned int numBins)
{
        int ii,jj;
        float mValue, maskTotal;
        float hValue, sValue;
        int hBin, sBin;
        //cv::Scalar_<uchar> hsvValue;
        cv::Vec3b hsvValue;
        
        //create a weighting mask given the box size
        maskTotal = 0;
        wMask = cv::Mat::zeros(detBox.height,detBox.width, CV_32FC1);
        for (ii=0; ii<detBox.width; ii++)
        {
                //sinus pulse
                mValue = sin(M_PI*(float)(ii)/(float)(detBox.width));
                
                //trianglular pulse
                //if (ii<detBox.width/2) mValue = (float)ii/(detBox.width/2.0);
                //else mValue = 1-(float)(ii-detBox.width/2.0)/(detBox.width/2.0);
                
                //quadratic pulse
                //if (ii<detBox.width/2) mValue = pow((float)ii/(detBox.width/2.0),2);
                //else mValue = pow(1-(float)(ii-detBox.width/2.0)/(detBox.width/2.0),2);
                
                for (jj=0; jj<detBox.height; jj++)
                {
                        wMask.at<float>(jj,ii) = mValue;
                        maskTotal += mValue;
                }
        }
        
        //convert current frame to hsv
        cvtColor(img, imgHsv, CV_BGR2HSV);
        
        //split channels
        cv::split(imgHsv,imgHsvSplit); //split RGB channels, just for GUI purposes
        
        //clears HS histogram
        hsHist = cv::Mat::zeros(numBins,numBins,CV_32FC1);
        
        //fills histogram taking into account mask. Horizontal axis (width, cols) holds Hue. Vertical axis (height, rows) holds saturation
        for (ii=detBox.x; ii<(detBox.x+detBox.width); ii++)
        {
                for (jj=detBox.y; jj<(detBox.y+detBox.height); jj++)
                {
                        //hsvValue = imgHsv.at< cv::Scalar_<uchar> >(jj,ii);
                        hsvValue = imgHsv.at<cv::Vec3b>(jj,ii);
                        hValue = (float)hsvValue[0];
                        sValue = (float)hsvValue[1];
                        hBin = (int)(hValue*numBins/256.0);
                        sBin = (int)(sValue*numBins/256.0);
                        hsHist.at<float>(sBin,hBin) = hsHist.at<float>(sBin,hBin) + wMask.at<float>(jj-detBox.y,ii-detBox.x);
                        //std::cout << "hBin: " << hBin << "; sBin: " << sBin << std::endl;
                }
        }               
        
        //normalize histogram with the total "energy" of the box
        //double maxVal=0; minMaxLoc(hsHist, 0, &maxVal, 0, 0); std::cout << "maxVal: " << maxVal << std::endl;
        for (ii=0; ii<(int)numBins; ii++)
        {
                for (jj=0; jj<(int)numBins; jj++)
                {
                        hsHist.at<float>(ii,jj) = 255*hsHist.at<float>(ii,jj)/maskTotal;
                }
        }
}


double CpersonAppearance::matchAppearance(CpersonAppearance & app) const
{
        return 0;
}

void CpersonAppearance::updateAppearance()
{
        
}

cv::Mat & CpersonAppearance::getFloodImage()
{
        return floodImage;
}

cv::Mat & CpersonAppearance::getBoxImage()
{
        return boxAppImage;
}

cv::Mat & CpersonAppearance::getwMask()
{
        return wMask;
}

cv::Mat & CpersonAppearance::getImgHsv()
{
        return imgHsv;
}

cv::Mat & CpersonAppearance::getImgHsv(int chId)
{
        return imgHsvSplit[chId];
}

cv::Mat & CpersonAppearance::getHsHist()
{
        return hsHist;
}

void CpersonAppearance::printAppearance() const
{
        
}

