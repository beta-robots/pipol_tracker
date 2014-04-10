#include "colorBoxes.h"

ColorBoxes::ColorBoxes()
{
  numBoxes = NUM_OF_BOXES_DEFAULT;
}

ColorBoxes::~ColorBoxes()
{

}

// dctNormalize?
void ColorBoxes::addAppearance(cv::Mat & img, const cv::Rect &detBox)
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

void ColorBoxes::merge(const AbstractAppearanceModel* app)
{
  const ColorBoxes* other = dynamic_cast<const ColorBoxes*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in ColorBoxes::merge: the given model is not of this type");
  if(other->boxSet.empty())
    throw std::runtime_error("Error in ColorBoxes::merge: the given model is empty");

  // not implemented yet
}

double ColorBoxes::match(const AbstractAppearanceModel* app) const
{
  const ColorBoxes* other = dynamic_cast<const ColorBoxes*>(app);
  if(other == NULL)
    throw std::runtime_error("Error in ColorBoxes::match: the given model is not of this type");
  if(other->boxSet.empty())
    throw std::runtime_error("Error in ColorBoxes::match: the given model is empty");
  // not implemented yet

  return 0;
}

void ColorBoxes::print() const
{
  //to do, debugging purposes
}

cv::Mat & ColorBoxes::getFloodImage()
{
  return floodImage;
}

cv::Mat & ColorBoxes::getBoxImage()
{
  return boxAppImage;
}
