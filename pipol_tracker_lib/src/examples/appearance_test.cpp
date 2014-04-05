
#include "appearance/hsHistogram.h"
#include <cstdlib>

int main(int argc, char** argv)
{
        cv::Mat rawImage;
        cv::VideoCapture video;
        HsHistogram appHS[2] = HsHistogram(1);
        bool appIdx = true;         
        double matchValue;
        unsigned int count = 0;
        bool run = true;
        cv::Rect_<int> box;
        bool isDevice = false;
        bool isFile = false;
        std::string fileName;
        int deviceId;
        
        //check input params
        if (argc!=3)
        {
                std::cout << "\nCalling sequence is: $ ./appearance_test d|f deviceNum|fileName\n" << std::endl;
                return -1;
        }

        //check if video or image collection input
        if ( *argv[1] == 'd' ) 
        {
                isDevice = true;
                deviceId = atoi(argv[2]);
        }
        if ( *argv[1] == 'f' )
        {
                isFile = true;
                fileName = std::string(argv[2]);
        }
        if ( (!isDevice)&&(!isFile) )
        {
                std::cout << "\nUnknown Input format. Only 'd' (device) or 'f' (video file) allowed.\n" << std::endl;
                return -1;                      
        }
       
        //initializes windows
        cv::namedWindow("rawImage",CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
        cv::moveWindow("rawImage",20,20);
        cv::namedWindow("HS-histogram",CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
        cv::moveWindow("HS-histogram",320,20);
        
        //opens video device
        if ( isDevice )
        {
                video.open(deviceId);
        
                if(video.isOpened())  // check if we succeeded
                {
                        std::cout << "Getting video from video device /dev/video" << deviceId << std::endl;
                }
                else
                {
                        std::cout << "Unable to get video from video device /dev/video" << deviceId << ". EXIT" << std::endl;
                        return -1;
                }
        }
        
        //opens video file
        if ( isFile )
        {
                video.open(fileName);
                        
                if(video.isOpened())  // check if we succeeded
                {
                        std::cout << "Getting video from " << fileName << std::endl;
                }
                else
                {
                        std::cout << "Unable to get video from "<< fileName << ". EXIT" << std::endl;
                        return -1;
                }
        }
        
        //gets a frame and initializes the box with image size
        video >> rawImage; 
        box.x = rawImage.cols/2 - rawImage.cols/10;
        box.y = 30;
        box.width = rawImage.cols/5;
        box.height = rawImage.rows - 60;
        
        //main loop
        while(run)
        {
                //get video frame and check it
                video >> rawImage; 
                if(!rawImage.data) 
                {
                        run = false;
                        std::cout << "No Image from input device" << std::endl;
                        break;
                }
                                
                //compute appearance based on color histogram
                appHS[(int)appIdx].addAppearance(rawImage,box);
                
                //check matching between current & last appearances
                if ( count > 1 )
                {
                        matchValue = appHS[(int)appIdx].match(&appHS[(int)(!appIdx)]);
                        std::cout << "matchValue: " << matchValue << std::endl;
                }
                
                //displays images
                cv::rectangle(rawImage, box, cv::Scalar(255,0,0), 5);
                cv::imshow("rawImage", rawImage);
                cv::imshow("HS-histogram", appHS[(int)appIdx].getHsHist());
                
                //wait for a key 10 ms (in some computer 1 ms is not enough to refresh)
                cv::waitKey(10); 
                
                //switch even indicator
                appIdx = !appIdx;
                
                //increment counter
                count ++;
        }

        //end
        return 0;
}
