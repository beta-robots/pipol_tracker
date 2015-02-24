/*
 * Interactive markers for ground truth annotation
 */

//std dependencies
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <fstream>
#include <sstream>

//ROS dependencies
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>


//class PeopleMarker
class PeopleMarker
{
    protected:
        static unsigned int id_count_;
        unsigned int id_;
        visualization_msgs::InteractiveMarker imarker_;
        visualization_msgs::Marker box_marker_;
        visualization_msgs::InteractiveMarkerControl box_control_;        
        visualization_msgs::InteractiveMarkerControl move_control_x_;
        visualization_msgs::InteractiveMarkerControl move_control_y_;
        
    public:
        //PeopleMarker(interactive_markers::InteractiveMarkerServer & _server) :
        PeopleMarker() :
            id_(++id_count_)
        {
            //imarker init
            imarker_.header.frame_id = "/base_link";
            imarker_.name = "target_" + std::to_string(id_);
            imarker_.description = "Ground truth position for target " + std::to_string(id_);
            
            //marker init
            box_marker_.type = visualization_msgs::Marker::CUBE;
            box_marker_.scale.x = 0.1;
            box_marker_.scale.y = 0.1;
            box_marker_.scale.z = 0.1;
            box_marker_.color.r = 1.0;
            box_marker_.color.g = 0.0;
            box_marker_.color.b = 0.0;
            box_marker_.color.a = 0.4;
            
            //box control init
            box_control_.always_visible = true;
            box_control_.markers.push_back( box_marker_ );
            
            // add the box control to the interactive marker
            imarker_.controls.push_back( box_control_ );
            
            //move control init
            move_control_x_.name = "move_x";
            move_control_y_.name = "move_y";
            move_control_x_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            move_control_y_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            
            //Get Y axis by rotating pi/2 around z axis (0,0,1)'
            move_control_y_.orientation.w = 0.7071068; //cos(pi/4)
            move_control_y_.orientation.x = 0.; //0*sin(pi/4)
            move_control_y_.orientation.y = 0.; //0*sin(pi/4)
            move_control_y_.orientation.z = 0.7071068; //1*sin(pi/4)
            
            // add the move control to the interactive marker
            imarker_.controls.push_back( move_control_x_ );
            imarker_.controls.push_back( move_control_y_ );
            
            // add the marker to the server collection
            //_server.insert(imarker_);
        };

        virtual ~PeopleMarker()
        {
            //
        };
        
        visualization_msgs::InteractiveMarker & getImarker()
        {
            return imarker_;
        }
            
};

class GTannotation
{
    protected: 
        //lists
        static std::list<PeopleMarker> imarkers_last_;
        std::list<PeopleMarker> imarkers_all_;
        
        //interactive markers server
        interactive_markers::InteractiveMarkerServer server_;
        
        //interactive buttons
        visualization_msgs::InteractiveMarker newT_button_;
        visualization_msgs::Marker newT_marker_;
        visualization_msgs::InteractiveMarkerControl newT_control_;
        
        //file where ground truth is saved
        std::ofstream gt_file;

    public:
        GTannotation() : 
            server_("people_marker"),
            gt_file("/home/andreu/Desktop/gt.txt")
        {
            //imarker init
            newT_button_.header.frame_id = "/base_link";
            newT_button_.name = "new_target_button";
            newT_button_.description = "Button to create a new target";            
            newT_button_.pose.position.x = 5;
            newT_button_.pose.position.y = 2;
            
            //marker init
            newT_marker_.header.frame_id = "/base_link";
            newT_marker_.type = visualization_msgs::Marker::SPHERE;            
            newT_marker_.pose.position.x = 5;
            newT_marker_.pose.position.y = 2;
            newT_marker_.scale.x = 0.6;
            newT_marker_.scale.y = 0.6;
            newT_marker_.scale.z = 0.6;
            newT_marker_.color.r = 0.0;
            newT_marker_.color.g = 1.0;
            newT_marker_.color.b = 0.0;
            newT_marker_.color.a = 1.0;
            
            //control init
            newT_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
            newT_control_.name = "button_control";
            newT_control_.always_visible = true;
            newT_control_.markers.push_back( newT_marker_ );
            
            // add the newT control to the interactive marker
            newT_button_.controls.push_back( newT_control_ );
            
            // set to server and commit
            server_.insert(newT_button_, &GTannotation::newTargetButton);
            server_.applyChanges();
        };
        
        virtual ~GTannotation()
        {
            //
        };

        void update()
        {
            std::list<PeopleMarker>::iterator it;
            
            //manage new created targets
            for( it = imarkers_last_.begin(); it != imarkers_last_.end(); it ++ )
            {
                // set to server and commit
                server_.insert( it->getImarker() );
            }
            
            //append last created markers to the global list
            imarkers_all_.splice(imarkers_all_.end(), imarkers_last_); 
            
            //commit changes to server
            server_.applyChanges();
        }
        
        static void newTargetButton( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
        {
            //cout message
            std::cout << "Button pressed!" << std::endl;
            
            // create and push_back a new target marker
            imarkers_last_.push_back( PeopleMarker() );            
        };
        
        void annotateButton( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
        {
            //TODO
        };

};

//static members init
unsigned int PeopleMarker::id_count_ = 1;
std::list<PeopleMarker> GTannotation::imarkers_last_;

int main(int argc, char** argv)
{
    //init ros
    ros::init(argc, argv, "gt_annotation_imarker");

    //Create a Ground Truth annotation
    GTannotation gt;
    
    //set node loop rate
    ros::Rate loopRate(10);

    //main loop
    while ( ros::ok() )
    {
        //execute pending callbacks
        ros::spinOnce(); 

        //do things
        gt.update(); 

        //relax to fit output rate
        loopRate.sleep();            
    }
}
