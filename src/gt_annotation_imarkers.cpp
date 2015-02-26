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
#include <rosgraph_msgs/Clock.h>

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
            imarker_.name = "T" + std::to_string(id_);
            imarker_.description = "T" + std::to_string(id_);
            
            //marker init
            //box_marker_.header.frame_id = "/base_link";
            box_marker_.type = visualization_msgs::Marker::CUBE;
            box_marker_.scale.x = 0.2;
            box_marker_.scale.y = 0.2;
            box_marker_.scale.z = 0.2;
            box_marker_.color.r = 1.0;
            box_marker_.color.g = 0.0;
            box_marker_.color.b = 0.0;
            box_marker_.color.a = 0.6;
            
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
        
        unsigned int getId()
        {
            return id_;
        }
        
        double getRangeSq()
        {
            return getX()*getX() + getY()*getY();
        }
        
        double getX()
        {
            return imarker_.controls[0].markers[0].pose.position.x;
        }

        double getY()
        {
            return imarker_.controls[0].markers[0].pose.position.x;
        }
        
};

struct TargetPosition
{
    unsigned int id_;
    double x_;
    double y_;
    
    TargetPosition( unsigned int _id, double _x, double _y ) :
        id_(_id),
        x_(_x),
        y_(_y)
    {
        //
    }

    
    double getRangeSq()
    {
        return x_*x_+y_*y_;
    }
};

class GTannotation
{
    protected: 
        //ros node handle
        ros::NodeHandle nh_;        
        
        //file where ground truth is saved
        std::ofstream gt_file_;        

        //clock subscriber
        ros::Subscriber clock_subscriber_;
        
        //current time stamp
        ros::Time current_ts_;
                
        //lists
        static std::list<PeopleMarker> imarkers_last_;
        std::list<PeopleMarker> imarkers_all_;
        
        //list of target target positions
        static std::list<TargetPosition> target_positions_;
        
        //flag to indicate annotate button pressed
        static bool annotate_flag_; 
        
        //interactive markers server
        interactive_markers::InteractiveMarkerServer server_;
        
        //interactive button to create a new target
        visualization_msgs::InteractiveMarker newT_button_;
        visualization_msgs::Marker newT_marker_;
        visualization_msgs::InteractiveMarkerControl newT_control_;
        
        //interactive button to annotate current iteration
        visualization_msgs::InteractiveMarker annotate_button_;
        visualization_msgs::Marker annotate_marker_;
        visualization_msgs::InteractiveMarkerControl annotate_control_;

    public:
        GTannotation() : 
            nh_(ros::this_node::getName()),
            server_("people_marker"),
            gt_file_("/home/andreu/Desktop/gt.txt")
        {
            //newT imarker init
            newT_button_.header.frame_id = "/base_link";
            newT_button_.name = "new_target_button";
            newT_button_.description = "Press to create a new target";            
            newT_button_.scale = 2;
            newT_button_.pose.position.x = 3.2;
            newT_button_.pose.position.y = 0.3;
            
            //newT marker init
            newT_marker_.header.frame_id = "/base_link";
            newT_marker_.type = visualization_msgs::Marker::SPHERE;            
            newT_marker_.pose.position.x = 6;
            newT_marker_.pose.position.y = 3;
            newT_marker_.scale.x = 0.6;
            newT_marker_.scale.y = 0.6;
            newT_marker_.scale.z = 0.6;
            newT_marker_.color.r = 0.0;
            newT_marker_.color.g = 1.0;
            newT_marker_.color.b = 0.0;
            newT_marker_.color.a = 1.0;
            
            //newT control init
            newT_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
            newT_control_.name = "button_control";
            newT_control_.always_visible = true;
            newT_control_.markers.push_back( newT_marker_ );
            
            // add the newT control to the newT interactive marker
            newT_button_.controls.push_back( newT_control_ );
            
            //annotate imarker init
            annotate_button_.header.frame_id = "/base_link";
            annotate_button_.name = "annotate_button";
            annotate_button_.description = "Press to annotate iteration";            
            annotate_button_.scale = 2;
            annotate_button_.pose.position.x = 2.4;
            annotate_button_.pose.position.y = 0.65;
            
            //newT marker init
            annotate_marker_.header.frame_id = "/base_link";
            annotate_marker_.type = visualization_msgs::Marker::SPHERE;            
            annotate_marker_.pose.position.x = 5.2;
            annotate_marker_.pose.position.y = 3;
            annotate_marker_.scale.x = 0.6;
            annotate_marker_.scale.y = 0.6;
            annotate_marker_.scale.z = 0.6;
            annotate_marker_.color.r = 1.0;
            annotate_marker_.color.g = 0.0;
            annotate_marker_.color.b = 0.0;
            annotate_marker_.color.a = 1.0;
            
            //newT control init
            annotate_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
            annotate_control_.name = "button_control";
            annotate_control_.always_visible = true;
            annotate_control_.markers.push_back( annotate_marker_ );
            
            // add the newT control to the newT interactive marker
            annotate_button_.controls.push_back( annotate_control_ );
                        
            // set to server and commit
            server_.insert(newT_button_, &GTannotation::newTargetButton);
            server_.insert(annotate_button_, &GTannotation::annotateButton);
            server_.applyChanges();          
            
            //set clock subscriber. Just queue the last message
            clock_subscriber_ = nh_.subscribe("/clock", 1, &GTannotation::clockCallback, this);
        };
        
        virtual ~GTannotation()
        {
            //close ground truth file
            gt_file_.close(); 
        };

        void update()
        {
            std::list<PeopleMarker>::iterator m_it;
            std::list<TargetPosition>::iterator t_it;
            
            //if new created targets, add them to the global list
            if ( imarkers_last_.size() != 0 )
            {
                for( m_it = imarkers_last_.begin(); m_it != imarkers_last_.end(); m_it ++ )
                {
                    // set to server and commit
                    server_.insert( m_it->getImarker(), &GTannotation::targetPositionUpdate );
                }
                
                //append last created markers to the global list
                imarkers_all_.splice(imarkers_all_.end(), imarkers_last_); 
                
                //commit changes to server
                server_.applyChanges();
            }
            
            //if annotate button has been pressed, do annotate
            if ( annotate_flag_ )
            {
                annotate_flag_ = false;
                gt_file_ << current_ts_ << " ";
                for( t_it = target_positions_.begin(); t_it != target_positions_.end(); t_it ++ )
                {
                    //check if nearest than 5m
                    if ( t_it->getRangeSq() < 25 )
                    {
                        gt_file_ << t_it->id_ << " " << t_it->x_ << " " << t_it->y_ << " ";
                    }
                }
                gt_file_ << std::endl;
            }
        }
        
        static void newTargetButton( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
        {
            if ( feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK )
                imarkers_last_.push_back( PeopleMarker() ); // create and push_back a new target marker
        };
        
        static void annotateButton( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
        {
            if ( feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK )
                annotate_flag_ = true;
        };
        
        static void targetPositionUpdate( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
        {
            std::string im_name;
            unsigned int im_id;
            std::list<TargetPosition>::iterator t_it;
            bool t_found = false;
            
            //get the target id
            im_name = feedback->marker_name;
            im_id = std::stoul( im_name.substr(1,im_name.size()-1) ); //removes initial "T" and gets the unsigned integer
            
            //update position in the target_positions_ list
            for( t_it = target_positions_.begin(); t_it != target_positions_.end(); t_it ++ )
            {
                if ( im_id == t_it->id_ ) //already in the list
                {
                    t_it->x_ = feedback->pose.position.x;
                    t_it->y_ = feedback->pose.position.y;
                    t_found = true;
                    break;
                }
            }
            
            if ( !t_found )
            {
                target_positions_.push_back( TargetPosition(im_id, feedback->pose.position.x, feedback->pose.position.y) );
            }
        }
        
        void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
        {
            current_ts_ = msg->clock;
            //std::cout << "Time: " << msg->clock << std::endl;
        };

};

//static members init. (Because imarkers feedback callbacks should be static)
unsigned int PeopleMarker::id_count_ = 0;
bool GTannotation::annotate_flag_ = false;
std::list<PeopleMarker> GTannotation::imarkers_last_;
std::list<TargetPosition> GTannotation::target_positions_;

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
