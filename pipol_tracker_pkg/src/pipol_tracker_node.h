#ifndef pipol_tracker_node_H
#define pipol_tracker_node_H

//std
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

//pipol_tracker LIBRARY
#include "peopleTracker.h"

//ROS headers for image I/O
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

//ROS std messages
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h> //odometry (input)
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

//ROS messages belonging to this package
#include <pipol_tracker_pkg/LegDetections.h> //leg detections (input)
#include <pipol_tracker_pkg/HogDetections.h> //body detections (input)
#include <pipol_tracker_pkg/FaceDetections.h> //face detections (input)
#include <pipol_tracker_pkg/peopleTrackingArray.h> //target list (output)

//ROS dynamic configure
#include <pipol_tracker_pkg/pipolTrackerParamsConfig.h>

//open TLD
// #include <tld_msgs/BoundingBox.h>
// #include <tld_msgs/Target.h>

//visualization constants 
const double MARKER_SIZE = 0.5;
const double MARKER_Z = 0.2;
const double MARKER_DURATION = 0.1;
const double MARKER_TEXT_SIZE = 0.3;
const double MARKER_TRANSPARENCY = 0.9;

//node execution mode
enum executionModes {MULTI_TRACKING=0, SHOOT_TLD, FOLLOW_ME};

/** \brief Wrapper class of the CpeopleTracker class from pipol_tracker library
 *
 */
class pipolTrackerNode
{
      protected: 
            //ros node handle
            ros::NodeHandle nh;
            
            //image transport
            image_transport::ImageTransport it;
            
            // subscribers 
            ros::Subscriber odometrySubs;
            ros::Subscriber legDetectionsSubs;
            ros::Subscriber bodyDetectionsSubs;
            ros::Subscriber faceDetectionsSubs;
            ros::Subscriber followMeSubs;
            ros::Subscriber tldDetectionsSubs;
            image_transport::Subscriber imageSubs;
            cv_bridge::CvImagePtr cvImgPtrSubs;            
                  
            //publishers & messages
            ros::Publisher particleSetPub;      
            visualization_msgs::MarkerArray MarkerArrayMsg;
            ros::Publisher peopleSetPub;
            pipol_tracker_pkg::personArray personArrayMsg;
            image_transport::Publisher imagePub;      
            cv_bridge::CvImage cvImgPub;
            // ros::Publisher tldBoxPub;
            // tld_msgs::Target tldBoxMsg;
            
            //management variables
            unsigned int exeMode; //execution mode
            //bool tldMessageFilled; //indicates when the TLD request message has been filled
                  
            //pipol_tracker library objects
            CpeopleTracker tracker;//The tracker object !!!
            pFilterParameters filterParams; // particle filter params
            trackerParameters trackerParams; // tracker params
            CodometryObservation platformOdometry; //keeps last odometry data
            
            //debugging
            bool verboseMode;
            bool viewBodyDetections;
            double ratioParticlesDisplayed;
            //unsigned int frameCount, hogDetCount;
            
            // initialize tf from base_link to camera, and get camera matrix
            //void initCamera();      
            
            //useful to initialize base_link to camera transform
            //tf::TransformListener tfListener;      
      
      protected: 
            // subscriber callbacks
            void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
            void legDetections_callback(const pipol_tracker_pkg::LegDetections::ConstPtr& msg);
            void bodyDetections_callback(const pipol_tracker_pkg::HogDetections::ConstPtr& msg);
            void faceDetections_callback(const pipol_tracker_pkg::FaceDetections::ConstPtr& msg);
            void followMe_callback(const std_msgs::Int32::ConstPtr& msg);
            void tldDetections_callback(const tld_msgs::BoundingBox::ConstPtr& msg);
            void image_callback(const sensor_msgs::ImageConstPtr& msg);


      public:
            /** \brief Constructor
            * 
            * This constructor initializes specific class attributes and all ROS
            * communications variables to enable message exchange.
            */
            pipolTrackerNode();

            /** \brief Destructor
            * 
            * This destructor frees all necessary dynamic memory allocated within this
            * this class.
            */
            ~pipolTrackerNode();

            /** \brief Main process 
            * 
            * Main process flow
            * 
            **/
            void process();
                        
            /** \brief Fill output messages
            * 
            * Fills main output and debug messages
            */
            void fillMessages();
                                    
};
#endif
