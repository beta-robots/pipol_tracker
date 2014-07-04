
//ros dependencies
#include "pipol_tracker_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "pipol_tracker_node");
      
      //create ros wrapper object
      CpipolTrackerNode tracker;
      
      //set node loop rate
      ros::Rate loopRate(tracker.rate_);
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 

            //do things
            tracker.process();
                        
            //relax to fit output rate
            loopRate.sleep();            
      }
            
      //exit program
      return 0;
}