#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <numeric>

#include <pal_appearance_models/appearance.h>
#include <pal_appearance_models/oldam.h>
#include <pal_appearance_models/colorBoxes.h>
#include <pal_appearance_models/hogmodel.h>
#include <pal_appearance_models/hsHistogram.h>
#include <pal_appearance_models/rgbHistogram.h>
#include <pal_appearance_models/simplepatchmodel.h>
#include <pal_appearance_models/covariancemodel.h>
#include <pal_appearance_models/normalizedpatchmodel.h>
#include <pal_appearance_models/combined1.h>
#include <pal_appearance_models/equalblockappearance.hpp>
#include <pal_appearance_models/commonblockappearance.hpp>

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define VERBOSE

#define THRESHOLD 0.55 // might be a good idea to move it to command line argument

class TestCase
{
public:
  int personId;
  cv::Mat image;
  cv::Rect bbox;

  TestCase()
  {
    personId = -1;
  }

  TestCase(const std::string& data, const std::string& prefix)
  {
    std::istringstream is(data);

    is >> personId;

    std::string file_name;
    is >> file_name;
    image = cv::imread(prefix + file_name);

    is >> bbox.x;
    is >> bbox.y;
    is >> bbox.width;
    is >> bbox.height;
  }

  void print() const
  {
    std::cout << "ID: " << personId << ", rows: "
              << image.rows << ", cols: " << image.cols
              << ", bbox x: " << bbox.x << ", y: " << bbox.y
              << ", height: " << bbox.height << ", width: " << bbox.width
              << std::endl;
  }

};

struct evalStatistics
{
public:
  int truePos;
  int falsePos;
  int trueNeg;
  int falseNeg;

  evalStatistics()
  {
    truePos = falsePos = trueNeg = falseNeg = 0;
  }

  evalStatistics& operator+=(const evalStatistics& rhs)
  {
    this->truePos += rhs.truePos;
    this->falsePos += rhs.falsePos;
    this->trueNeg += rhs.trueNeg;
    this->falseNeg += rhs.falseNeg;
    return *this;
  }
};

struct confPlusID
{
public:
  int trackID;
  int detectionID;
  double confidence;

  friend bool operator< (confPlusID a, confPlusID b);
};

bool operator< (confPlusID a, confPlusID b)
{
  return a.confidence < b.confidence;
}

std::vector< std::vector< boost::shared_ptr<TestCase> > > loadDataset(int argc, char* argv[]);

void splitDataset(const std::string& modelArg, const int trainingPercentage,
                  const std::vector< std::vector< boost::shared_ptr<TestCase> > > &dataset,
                  std::vector< boost::shared_ptr<AbstractAppearanceModel> > &training,
                  std::vector< boost::shared_ptr<AbstractAppearanceModel> > &test,
                  std::vector<double> &runtimesExtract, std::vector<int> &testToDataset);


int main(int argc, char* argv[])
{
  if(argc<4)
  {
    std::cerr << "Not enough arguments given!" << std::endl;
    std::cout << "Way to call: " << argv[0]
              << " <model name> <training percentage> <dataset1 root> ... <datasetK root>"
              << std::endl;
    std::cout << "Acceptable model names: simplepatch, oldam, colorboxes, hshistogram, hog, covariance, normalizedpatch, blockcovariance" << std::endl << std::endl
              << "<training percentage>: the way to split up dataset for training and test sets is the following: training: first 20% of curr dataset, test: leftover 80%."
              << std::endl << std::endl
              << "<dataset root>: absolute path to root of one dataset. A valid dataset contains a valid info.txt file plus images of the same person."
              << std::endl;
    return 1;
  }

  const std::string modelArg(argv[1]);
  const int trainingPercentage = atoi(argv[2]);

  // load dataset
  std::vector< std::vector< boost::shared_ptr<TestCase> > > dataset = loadDataset(argc, argv);

  // vectors for keeping track of speed
  std::vector<double> runtimesMatch;
  std::vector<double> runtimesExtract;

  // divide the dataset into a training and a test set for each person
  std::vector< boost::shared_ptr<AbstractAppearanceModel> > tracks;
  std::vector< boost::shared_ptr<AbstractAppearanceModel> > detections;
  std::vector<int> numTestForDataset;
  splitDataset(modelArg, trainingPercentage, dataset, tracks, detections, runtimesExtract, numTestForDataset);

#ifdef VERBOSE
  std::cout << tracks.size() << " appearances matched against " << detections.size() << " test cases." << std::endl;
#endif

  //match all with all and store the results in a "multitable"
  std::vector< confPlusID > matches; // the result {trackID, detectionID, conf}
  cv::Mat matchesMat(detections.size(), tracks.size(), CV_64FC1);
  confPlusID result;
  for(unsigned int i=0; i<tracks.size(); ++i)
  {
    for(unsigned int j=0; j<detections.size(); ++j)
    {
#ifdef VERBOSE
      std::cout << "Matching track " << i+1 << " to detections " << j+1 ;
#endif

      //prepare struct
      result.trackID = tracks[i]->ID;
      result.detectionID = detections[j]->ID;

      double duration; // used for time measuring
      duration = static_cast<double>(cv::getTickCount()); // used for time measuring

      result.confidence = tracks[i]->match(detections[j].get()); // MATCHING DONE HERE
      matchesMat.at<double>(j,i) = result.confidence; // also fill a matrix with the results a different way

      duration = (static_cast<double>(cv::getTickCount())-duration)*1000.0; // used for time measuring
      duration /= cv::getTickFrequency(); // used for time measuring
      runtimesMatch.push_back(duration);// register the elapsed time in ms

      // store struct
      matches.push_back(result);

#ifdef VERBOSE
      std::cout << " with confidence: " << result.confidence << std::endl;
#endif
    }
  }

  //***PREC/RECALL section
  evalStatistics stats;
  for(unsigned int i=0; i<matches.size(); ++i)
  {
    //evaluate results
    if(matches[i].trackID == matches[i].detectionID)
    {
      if(matches[i].confidence > THRESHOLD) // matching
        ++(stats.truePos);
      else
        ++(stats.falseNeg);
    } else
    {
      if(matches[i].confidence > THRESHOLD) // matching
        ++(stats.falsePos);
      else
        ++(stats.trueNeg);
    }
  }

  // classic precision/recall statistics computed when each model is matched against each training sample
  //    std::cout << "Results: [truePos, falsePos, trueNeg, falseNeg]" << std::endl;

  //    std::cout << "[" << stats.truePos << ", "
  //              << stats.falsePos << ", "
  //              << stats.trueNeg << ", "
  //              << stats.falseNeg << "]"
  //              << std::endl;

  if(stats.truePos != 0)
    std::cout << "Precision: " << static_cast<double>(stats.truePos)/
                 (stats.truePos+stats.falsePos) << std::endl
              << "Recall: " << static_cast<double>(stats.truePos)/
                 (stats.truePos+stats.falseNeg) << std::endl;
  else // if truePos is 0,
    std::cout << "Precision: 0" << std::endl
              << "Recall: 0" << std::endl;
  //***end of PREC/RECALL section


  //***SCORE section
  int score = 0;
  std::vector<double> confidences;

  for(unsigned int i=0; i<detections.size(); ++i)
  {
    for(unsigned int j=0; j<tracks.size(); ++j)
    {
      confidences.push_back(tracks[j]->match(detections[i].get())); // MATCHING DONE HERE
    }
    // get the index of the maximum element
    int idx = std::distance(confidences.begin(), std::max_element(confidences.begin(), confidences.end()));
    confidences.clear();

    // if the best match for a detections sample was the corresponding trained model, increase the score of the method
    if(detections[i]->ID == tracks[idx]->ID)
      ++score;
  }
  std::cout << "Pick score of method: " << score << " / " << detections.size() << " --> "
            << score/(static_cast<double>(detections.size())) << std::endl;

  //***end of SCORE section


  //***TOTAL PROBABILITY section

  // compute full table with each detection to each track
  cv::Mat probMatrix = cv::Mat::ones(detections.size(), tracks.size(), CV_64FC1);
  for(unsigned int i=0; i<detections.size(); ++i)
  {
    for(unsigned int j=0; j<tracks.size(); ++j)
    {
      // compute P(d_1,t_1) = m(d_1, t_1) * ¬m(d_1, t_2) * ... * ¬m(d_1, t_n)
      for(unsigned int k=0; k<tracks.size(); ++k)
      {
        if(k == j)
          probMatrix.at<double>(i,j) *= matchesMat.at<double>(i,k);
        else
          probMatrix.at<double>(i,j) *= (1.0 - matchesMat.at<double>(i,k));
      }
    }
  }

  int lastRow = 0;
  cv::Mat meanProbTable = cv::Mat::zeros(tracks.size(), tracks.size(), CV_64FC1);
  for(int j=0; j<probMatrix.cols; ++j)
  {
    // compute sum
    lastRow = 0;
    for(unsigned int meanRow = 0; meanRow < numTestForDataset.size(); ++meanRow)
    {
      for(int k=0; k < numTestForDataset[meanRow]; ++k)
        meanProbTable.at<double>(meanRow, j) += probMatrix.at<double>(lastRow+k,j);
      lastRow += numTestForDataset[meanRow];
    }

    // divide by the total nums
    for(unsigned int meanRow = 0; meanRow < numTestForDataset.size(); ++meanRow)
      meanProbTable.at<double>(meanRow, j) /= static_cast<double>(numTestForDataset[meanRow]);
  }

  // normalizing the mean probability table row by row gives nicer values
  // + the elements of the diagonal should appear a lot higher than others

  for(int i=0; i<meanProbTable.rows; ++i)
    cv::normalize(meanProbTable.row(i), meanProbTable.row(i));

  std::cout << "| ";
  for(unsigned int i=0; i<tracks.size(); ++i)
    std::cout << "t" << i << " ";
  std::cout << "|" << std::endl
            << "P: " << std::endl << meanProbTable << std::endl
            << "mean(tr(P)): "  << cv::trace(meanProbTable)(0)/meanProbTable.rows << std::endl;

  //***end of TOTAL PROBABILITY section

  const double avgMatchTime = std::accumulate(runtimesMatch.begin(), runtimesMatch.end(), 0.0) / runtimesMatch.size();
  const double avgExtractTime = std::accumulate(runtimesExtract.begin(), runtimesExtract.end(), 0.0) / runtimesExtract.size();
  std::cout << "Average runtime for matching: " << avgMatchTime
            << "ms, extract: " << avgExtractTime << " ms. "
            << "Computed from " << runtimesMatch.size() << " runs." << std::endl;

  return 0;
}


std::vector< std::vector< boost::shared_ptr<TestCase> > > loadDataset(int argc, char* argv[])
{
  // iterate through the directories received in command line arguments argv[i] i>1
  // { read info file, create TestCase objects }
  std::vector< std::vector< boost::shared_ptr<TestCase> > > result;
  for(int i=3; i<argc; ++i)
  {
    std::stringstream ss;
    ss << argv[i] << "/info.txt";
#ifdef VERBOSE
    std::cout << "Reading files from " << ss.str() << std::endl;
#endif

    // open the actual info file
    std::ifstream info;
    info.open(ss.str().c_str());
    if(!info.is_open())
    {
      std::cerr << "info.txt not present at " << ss.str() << std::endl;
      continue; // skip to the next dataset folder
    }
    std::string line;
    std::string prefix = std::string(argv[i]) + "/";

    std::vector< boost::shared_ptr<TestCase> > currDataset;
    // read info file line by line and construct test case objects

    while(std::getline(info, line))
    {
      if(line.length() == 0)
        continue; // in case there are a few new lines hanging around, skip them
#ifdef VERBOSE
      std::cout << line << std::endl;
#endif
      boost::shared_ptr<TestCase> test(new TestCase(line, prefix));
      test->print();
      currDataset.push_back(test);
    }
    result.push_back(currDataset);
  }
  return result;
}

void splitDataset(const std::string& modelArg, const int trainingPercentage,
                  const std::vector< std::vector< boost::shared_ptr<TestCase> > > &dataset,
                  std::vector< boost::shared_ptr<AbstractAppearanceModel> > &training,
                  std::vector< boost::shared_ptr<AbstractAppearanceModel> > &test,
                  std::vector<double> &runtimesExtract,
                  std::vector<int> &numTestForDataset)
{


  boost::shared_ptr<AbstractAppearanceModel> model;
  int numTests = 0;

  for(unsigned int i=0; i<dataset.size(); ++i)
  {
    if(modelArg == "oldam")
    {
      model = boost::shared_ptr<AbstractAppearanceModel>(new Oldam());
    } else if(modelArg == "colorboxes"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new ColorBoxes());
    } else if(modelArg == "hshistogram"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new HsHistogram());
    } else if(modelArg == "simplepatch"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new SimplePatchModel());
    } else if(modelArg == "hog"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new HOGModel());
    } else if(modelArg == "covariance"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new CovarianceModel());
    } else if(modelArg == "rgbhistogram"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new RGBHistogram());
    } else if(modelArg == "normalizedpatch"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new NormalizedPatchModel());
    } else if(modelArg == "blockcovariance"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new EqualBlockAppearance<CovarianceModel>());
    } else if(modelArg == "combined1"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new Combined1());
    } else if(modelArg == "commonblockcombined1"){
      model = boost::shared_ptr<AbstractAppearanceModel>(new CommonBlockAppearance<Combined1>());
    }

#ifdef VERBOSE
    std::cout << "Dataset "<< i
              << " number of training examples: "
              << ceil(dataset[i].size()*((trainingPercentage)/100.0))
              << std::endl;
#endif

    // aggregate all training samples of one person into one model

    // dataset : [train1 train2 train3 train4 test1 test2 test3 ... ]
    // dataset(borderIdx) = test1
    const unsigned int borderIdx = ceil(dataset[i].size()*(trainingPercentage/100.0));
    for(unsigned int j=0; j<borderIdx; ++j)
    {
      boost::shared_ptr<TestCase> curr = dataset[i][j];
      std::cout << j << std::endl;

      double duration; // used for time measuring
      duration = static_cast<double>(cv::getTickCount()); // used for time measuring

      model->addAppearance(curr->image, curr->bbox);

      duration = (static_cast<double>(cv::getTickCount())-duration)*1000.0; // used for time measuring
      duration /= cv::getTickFrequency(); // used for time measuring
      runtimesExtract.push_back(duration);// register the elapsed time in ms

      model->ID = curr->personId;
    }
    training.push_back(model);

#ifdef VERBOSE
    std::cout << "Dataset "<< i
              << " number of test appearances: "
              << floor(dataset[i].size()*((100.0-trainingPercentage)/100.0))
              << std::endl;
#endif

    // test models are represented by one image each
    for(unsigned int j=borderIdx; j<dataset[i].size(); ++j)
    {
      if(modelArg == "oldam")
      {
        model = boost::shared_ptr<AbstractAppearanceModel>(new Oldam());
      } else if(modelArg == "colorboxes"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new ColorBoxes());
      } else if(modelArg == "hshistogram"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new HsHistogram());
      } else if(modelArg == "simplepatch"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new SimplePatchModel());
      } else if(modelArg == "hog"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new HOGModel());
      } else if(modelArg == "covariance"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new CovarianceModel());
      } else if(modelArg == "rgbhistogram"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new RGBHistogram());
      } else if(modelArg == "normalizedpatch"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new NormalizedPatchModel());
      } else if(modelArg == "blockcovariance"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new EqualBlockAppearance<CovarianceModel>());
      } else if(modelArg == "combined1"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new Combined1());
      } else if(modelArg == "commonblockcombined1"){
        model = boost::shared_ptr<AbstractAppearanceModel>(new CommonBlockAppearance<Combined1>());
      }

      boost::shared_ptr<TestCase> curr = dataset[i][j];

      double duration; // used for time measuring
      duration = static_cast<double>(cv::getTickCount()); // used for time measuring

      model->addAppearance(curr->image);

      duration = (static_cast<double>(cv::getTickCount())-duration)*1000.0; // used for time measuring
      duration /= cv::getTickFrequency(); // used for time measuring
      runtimesExtract.push_back(duration);// register the elapsed time in ms

      model->ID = curr->personId;
      test.push_back(model);
    }
    numTestForDataset.push_back(test.size() - numTests);
    numTests = test.size();
  }
}
