#ifndef pipol_tracker_node_H
#define pipol_tracker_node_H

//std C++
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

//pipol_tracker LIBRARY
#include "pipol_tracker_lib/peopleTracker.h"

//Eigen (algebra)
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"


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

//ROS messages belonging to pal_vision/detection_msgs pkg. 
#include <pal_vision_msgs/LegDetections.h> //leg detections (input)
#include <pal_vision_msgs/HogDetections.h> //body detections (input)
#include <pal_vision_msgs/FaceDetections.h> //face detections (input)
#include <pal_detection_msgs/LegDetections.h> //leg detections (input)
#include <pal_detection_msgs/Detections2d.h> //body detections (input)
#include <pal_detection_msgs/FaceDetections.h> //face detections (input)
#include <pal_detection_msgs/PersonDetections.h> //3d body detections (input)


//ROS messages belonging to open-TLD pkg
// #include <tld_msgs/BoundingBox.h>
// #include <tld_msgs/Target.h>

//ROS messages belonging to this pkg
#include <pipol_tracker_pkg/personArray.h> //target list (output)

//ROS dynamic configure
#include <pipol_tracker_pkg/pipol_tracker_configConfig.h>

//visualization constants
const double MARKER_SIZE = 0.5;
const double MARKER_Z = 0.2;
const double MARKER_DURATION = 0.1;
const double MARKER_TEXT_SIZE = 0.3;
const double MARKER_TRANSPARENCY = 0.9;

//Put 2013 to use rosbags of 2013. Messages were from pal_vision_msgs, quite different than current pal_detection_msgs
const int YEAR = 2014; 

//node execution mode
enum executionModes {MULTI_TRACKING=0, SHOOT_TLD, FOLLOW_ME};

/** \brief Wrapper class of the CpeopleTracker class from pipol_tracker library
 *
 */
class CpipolTrackerNode
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
        ros::Subscriber body3dDetectionsSubs;
        ros::Subscriber followMeSubs;
        //ros::Subscriber tldDetectionsSubs;
        image_transport::Subscriber imageSubs;
        cv_bridge::CvImagePtr cvImgPtrSubs;            
        ros::Subscriber cameraInfoSubs;
        
                
        //publishers
        ros::Publisher particleSetPub;      
        ros::Publisher peopleSetPub;
        image_transport::Publisher imagePub;      
        // ros::Publisher tldBoxPub;
        
        // output messages
        visualization_msgs::MarkerArray MarkerArrayMsg;
        pipol_tracker_pkg::personArray personArrayMsg;
        cv_bridge::CvImage cvImgPub;
        // tld_msgs::Target tldBoxMsg;
        
        //sensor parameters (laser pose, camera calibration, ...)
        Eigen::Matrix4d laserH_base;//3D pose of laser device used by leg detector, wrt base_link
        Eigen::Matrix3d camK; //camera intrinsic parameters
        
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
        bool viewParticles;
        double odoTrans; 
        //double ratioParticlesDisplayed;
        //unsigned int frameCount, hogDetCount;
        
        // initialize tf from base_link to camera, and get camera matrix
        //void initCamera();      
        
        //useful to initialize base_link to camera transform
        //tf::TransformListener tfListener;
        
    public:
        double rate_;//wished process rate, [hz]
        
    protected: 
        // subscriber callbacks
        void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void legDetections_callback_2013(const pal_vision_msgs::LegDetections::ConstPtr& msg);
        void bodyDetections_callback_2013(const pal_vision_msgs::HogDetections::ConstPtr& msg);
        void faceDetections_callback_2013(const pal_vision_msgs::FaceDetections::ConstPtr& msg);
        void legDetections_callback(const pal_detection_msgs::LegDetections::ConstPtr& msg);
        void bodyDetections_callback(const pal_detection_msgs::Detections2d::ConstPtr& msg);
        void faceDetections_callback(const pal_detection_msgs::FaceDetections::ConstPtr& msg);            
        void body3dDetections_callback(const pal_detection_msgs::PersonDetections::ConstPtr& msg);
        void followMe_callback(const std_msgs::Int32::ConstPtr& msg);
        void image_callback(const sensor_msgs::ImageConstPtr& msg);
        void cameraInfo_callback(const sensor_msgs::CameraInfo & msg);
        //void tldDetections_callback(const tld_msgs::BoundingBox::ConstPtr& msg);

    public:
        /** \brief Constructor
        * 
        * This constructor initializes specific class attributes and all ROS
        * communications variables to enable message exchange.
        */
        CpipolTrackerNode();

        /** \brief Destructor
        * 
        * This destructor frees all necessary dynamic memory allocated within this
        * this class.
        */
        ~CpipolTrackerNode();

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
