
#include "hsHistogram.h"

HsHistogram::HsHistogram()
{
  numBins = NUM_BINS_PER_CHANNEL_DEFAULT;
}

HsHistogram::~HsHistogram()
{

}

void HsHistogram::extract(cv::Mat & img, const cv::Rect_<int> & detBox)
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

void HsHistogram::update()
{
  //questions:
  // * how much sense does it make to aggregate histograms in a reinforced learning fashion?
  // * is it better to use a queue with fixed length storing the last N histograms
  // and matching can be done by matching on each and evaluating results
  // using a fixed discount coefficient
}

double HsHistogram::match(const AbstractAppearanceModel* app) const
{
  //to do

  return 0;
}

void HsHistogram::print() const
{
  //to do, debugging purposes
}

cv::Mat & HsHistogram::getwMask()
{
  return wMask;
}

cv::Mat & HsHistogram::getImgHsv()
{
  return imgHsv;
}

cv::Mat & HsHistogram::getImgHsv(int chId)
{
  return imgHsvSplit[chId];
}

cv::Mat & HsHistogram::getHsHist()
{
  return hsHist;
}
