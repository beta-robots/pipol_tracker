#include "appearance.h"

cv::Mat AbstractAppearanceModel::cropBy(cv::Mat& img, int rowsPerc, int colsPerc)
{
    // make sure that the correct input is provided
    assert(rowsPerc >= 0 && rowsPerc <100
           && colsPerc >=0 && colsPerc < 100
           && img.rows > 0 && img.cols > 0);

    return img(cv::Rect(int((100-colsPerc)*img.cols/200.),
                        int((100-rowsPerc)*img.rows/200.),
                        int(colsPerc*img.cols/100.),
                        int(rowsPerc*img.rows/100.)));
}
