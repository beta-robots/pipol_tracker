#include "normalizedpatchmodel.h"

NormalizedPatchModel::NormalizedPatchModel():
  _normalizedWidth(20),
  _normalizedHeight(20)
{
}

void NormalizedPatchModel::imgToVector(const cv::Mat& img, patchVector& v)
{
  v.clear();
  for (int row = 0; row < img.rows; ++row)
    for (int col = 0; col < img.cols; ++col)
    {
      v.push_back( static_cast<float>(img.at<uchar>(row, col)) );
    }
}

float NormalizedPatchModel::mean(const std::vector<float>& v)
{
  unsigned int length = v.size();
  if ( length == 0 )
    throw std::runtime_error("Error in NormalizedPatchModel::mean function: empty vector");
  else if ( length == 1 )
    return v[0];

  return std::accumulate(v.begin(), v.end(), static_cast<float>(0)) / static_cast<float>(length);
}

void NormalizedPatchModel::meanAndStdDev(const std::vector<float>& v, float& m, float& s)
{
  m = mean(v);
  s = stdDev(v, m);
}

float NormalizedPatchModel::stdDev(const std::vector<float>& v, float mean)
{
  std::vector<float> zeroMean = v;

  std::transform( zeroMean.begin(), zeroMean.end(), zeroMean.begin(), std::bind2nd( std::minus<float>(), mean ) );

  float deviation = std::inner_product( zeroMean.begin(), zeroMean.end(), zeroMean.begin(), static_cast<float>(0) );
  return static_cast<float>( sqrt( deviation / v.size() ) );
}

struct Normalize : public std::unary_function<float, float>
{
  Normalize( const float& mean, const float& dev )
    : _mean(mean), _dev(dev) {
  }

  float operator()(const float& arg1)
  {
    return (arg1 - _mean)/_dev;
  }

  float _mean;
  float _dev;
};


void NormalizedPatchModel::normalize(const std::vector<float>& v, std::vector<float>& normalizedV)
{
  float m, s;
  meanAndStdDev(v, m, s);
  normalizedV = v;
  std::transform( v.begin(), v.end(), normalizedV.begin(), Normalize(m, s) );
}



void NormalizedPatchModel::addAppearance(cv::Mat& img, const cv::Rect &detBox)
{
  cv::Mat imgGray;
  if ( img.channels() == 3 ) //BGR image?
    cv::cvtColor( img, imgGray, CV_BGR2GRAY );
  else if ( img.channels() == 1 )
    imgGray = img.clone();
  else
    throw std::runtime_error("Error in NormalizedPatchModel::extract: image type not supported");

  cv::Mat roi;
  if ( detBox == cv::Rect() )
    roi = imgGray;
  else
    roi = imgGray(detBox);

  cv::Mat patch;

//  std::cout << "input image size w: " << img.size().width << " h: " << img.size().height << std::endl;
//  std::cout << "roi         size w: " << detBox.width << " h: " << detBox.height << std::endl;
//  std::cout << "image crop  size w: " << roi.size().width << " h: " << roi.size().height << std::endl;

  cv::resize(roi, patch, cv::Size(_normalizedWidth, _normalizedHeight));

//  cv::namedWindow("patch");
//  cv::imshow("patch", patch);
//  cv::waitKey(0);

  //std::cout << "patch  size w: " << patch.size().width << " h: " << patch.size().height << std::endl;

  patchVector patchVector;
  imgToVector(patch, patchVector);

  //float m = mean(patchVector);
  //float s = stdDev(patchVector, m);

  normalize(patchVector, patchVector);

  _appearances.push_back(patchVector);

}

float NormalizedPatchModel::patchSimilarity(const patchVector& v1, const patchVector& v2)
{
  float dist = 0;

  //compute normalized cross correlation of v1 and v2.
  for (unsigned int i = 0; i < v1.size(); ++i)
  {
    //As the vectors are already normalized (0 mean, 1 stdDev) the computation is just the product:
    dist += v1[i]*v2[i];
  }

  dist /= v1.size();

  return dist;
}

void NormalizedPatchModel::merge(const AbstractAppearanceModel* app)
{
    const NormalizedPatchModel* other = dynamic_cast<const NormalizedPatchModel*>(app);
    if(other == NULL)
        throw std::runtime_error("Error in NormalizedPatchModel::merge: the given model is not of this type");
    if(other->_appearances.size() != 1)
        throw std::runtime_error("Error in NormalizedPatchModel::merge: the given model contains more appearances");

    _appearances.push_back(other->_appearances[0]);
}

double NormalizedPatchModel::match(const AbstractAppearanceModel* app) const
{
  const NormalizedPatchModel* other = dynamic_cast<const NormalizedPatchModel*>(app);
  if ( other == NULL )
    throw std::runtime_error("Error in NormalizedPatchModel::match: the given model is not of this type");

  if ( other->_appearances.empty() )
    throw std::runtime_error("Error in NormalizedPatchModel::match: the given model is empty");

  float maxSimilarity = 0;

  for (unsigned int i = 0; i < _appearances.size(); ++i)
  {
    float s = patchSimilarity(_appearances[i], other->_appearances[0]);
    if ( s > maxSimilarity )
      maxSimilarity = s;
  }

  return static_cast<double>(maxSimilarity);
}

void NormalizedPatchModel::print() const
{
  std::cout << "Model contains " << _appearances.size() << " appearances of target." << std::endl;
}
