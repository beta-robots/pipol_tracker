
//ros dependencies
#include "pipol_tracker_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "btr_point_tracker_node");
      
      //create ros wrapper object
      CpointTrackerNode tracker;
      
      //set node loop rate
      ros::Rate loop_rate(20);
      
      //node loop 
      while ( ros::ok() )
      {
            //if new image , do things
            if ( tracker.newImage() )
            {
                  tracker.process();
                  tracker.publishImage();
                  tracker.publishTracks();
            }
            
            //execute pending callbacks
            ros::spinOnce(); 
      }
            
      //exit program
      return 0;
}