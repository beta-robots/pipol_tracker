
#include "appearance/colorBoxes.h"
#include "appearance/hsHistogram.h"
#include <cstdlib>

int main(int argc, char** argv)
{
        cv::Mat rawImage;
        cv::VideoCapture video;
        ColorBoxes appCB;
        HsHistogram appHS;
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
        cv::namedWindow("floodImage",CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
        cv::moveWindow("floodImage",320,20);        
        cv::namedWindow("Appearance Model",CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
        cv::moveWindow("Appearance Model",620,20);
        cv::namedWindow("wMask",CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
        cv::moveWindow("wMask",20,320);
        cv::namedWindow("hsv",CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
        cv::moveWindow("hsv",320,320);
        cv::namedWindow("hue",CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
        cv::moveWindow("hue",620,320);
        cv::namedWindow("saturation",CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
        cv::moveWindow("saturation",620,520);
        cv::namedWindow("HS-histogram",CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
        cv::moveWindow("HS-histogram",920,320);
        
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
                                
                //compute appearance based on a set of colored boxes
                appCB.extract(rawImage, box);
                                          
                //compute appearance based on color histogram
                appHS.extract(rawImage,box);
                
                //displays images
                cv::imshow("rawImage", rawImage);
                cv::imshow("floodImage", appCB.getFloodImage());
                cv::imshow("Appearance Model", appCB.getBoxImage());
                cv::imshow("wMask", appHS.getwMask());
                cv::imshow("hsv", appHS.getImgHsv());
                cv::imshow("hue", appHS.getImgHsv(0));
                cv::imshow("saturation", appHS.getImgHsv(1));
                cv::imshow("HS-histogram", appHS.getHsHist());
                cv::waitKey(10); // wait for a key 10 ms, in some computer 1 ms is not enough to refresh the windows
        }

        //end
        return 0;
}
