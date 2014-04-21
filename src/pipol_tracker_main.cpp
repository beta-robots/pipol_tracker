
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
      ros::Rate loopRate(10);
      
      //node loop 
      while ( ros::ok() )
      {
            //do things
            tracker.process();
            
            //execute pending callbacks
            ros::spinOnce(); 
            
            //relax to fit output rate
            loopRate.sleep();            
      }
            
      //exit program
      return 0;
}