
//std dependencies
#include <iostream>
#include <vector>
#include <string>
#include <fstream>

//ros dependencies
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <rosbag/player.h>
#include <rosbag/bag_player.h>

//node main
int main(int argc, char **argv)
{
    //user inputs
    char user_command = 0x0;
    int user_int;
    double user_double1, user_double2;
    
    //welcome
    std::cout << "=======================================================" << std::endl;
    std::cout << "                 GROUND TRUTH ANNOTATION               " << std::endl;
    std::cout << "e-> exit program" << std::endl;
    std::cout << "t-> new target" << std::endl;
    std::cout << "i-> new iteration" << std::endl;
    std::cout << "=======================================================" << std::endl;
    
    //init ros
    ros::init(argc, argv, "gt_annotation_node");

    //create the Bag and the BagPlayer objects
    rosbag::Bag bag;
    bag.open("/home/andreu/dataSets/peopleTracking/pal_detectors/20140925_twoPeople.bag", rosbag::bagmode::Read);
    rosbag::BagPlayer player(bag.getFileName());
    rosbag::View view(bag); //set the view including all topics of the bag
    ros::Time ts_i = view.getBeginTime();//get first ts
    ros::Time ts_e = view.getEndTime();//get end ts
    
    //create the file where ground truth will be saved
    std::ofstream gt_file;
    gt_file.open("/home/andreu/Desktop/gt.txt");
    
    //loop
    std::cout << "Starting bag play at " << ts_i << std::endl;
    while ( (ts_i.sec + 1  < ts_e.sec ) && (user_command != 'e') )
    {
        player.set_start(ts_i);
        ts_i.sec = ts_i.sec + 1;
        player.set_end(ts_i);
        player.start_play();
        std::cout << std::endl << "New iteration at " << ts_i << std::endl;
        gt_file << ts_i << " ";
        std::cout << "e,i,t ?: ";
        std::cin >> user_command;
        while(user_command == 't') 
        {
            switch(user_command)
            {
                case 'e': 
                    break;

                case 'i':
                    break;
                                        
                case 't':
                    std::cout << "target id: ";
                    std::cin >> user_int;
                    std::cout << "x: ";
                    std::cin >> user_double1;
                    std::cout << "y: ";
                    std::cin >> user_double2;
                    gt_file << user_int << " " << user_double1 << " " << user_double2 << " ";                    
                    std::cout << "e,i,t ?: ";
                    std::cin >> user_command;
                    break; 
                    
                default: 
                    std::cout << "e,i,t ?: ";
                    std::cin >> user_command;
                    break; 
            }
        }
        gt_file << std::endl; //next line to text file
            
    }

    //close the bag and the file
    bag.close();    
    gt_file.close();
    
    //exit program
    return 0;
}


//Some API findings ...

//     //create the bag object
//     rosbag::Bag bag;
//     bag.open("/home/andreu/dataSets/peopleTracking/pal_detectors/20140925_twoPeople.bag", rosbag::bagmode::Read);
//     
//     //set the view, including all topics
//     rosbag::View view(bag);
//     
//     //run over bag messages
//     rosbag::View::iterator bag_it;
//     for(bag_it = view.begin(); bag_it != view.end(); bag_it++)
//     {
//         std::cout << bag_it->getTime() << ": " << bag_it->getTopic() << std::endl;
//     }
//     close the bag
//     bag.close();    
        
//     //create a player
//     rosbag::PlayerOptions options;
//     options.start_paused = true;
//     options.bags.push_back("/home/andreu/dataSets/peopleTracking/pal_detectors/20140925_twoPeople.bag");
//     options.duration = 1;//1 second
//     options.time = 1411645985;
//     rosbag::Player player(options);
//     std::cout << "Start publishing from " << options.bags.at(0) << std::endl;
//     player.publish();

//----------------------------------------------

//     std::vector<std::string> topics;
//     topics.push_back(std::string("/beta/pipol_tracker_node/image_out"));
//     topics.push_back(std::string("/fullbody_2d_detector/detections"));
//     topics.push_back(std::string("/tf"));
//     topics.push_back(std::string("/clock"));
//     rosbag::View view(bag, rosbag::TopicQuery(topics));
