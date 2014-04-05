// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _people_tracking_rai_alg_node_h_
#define _people_tracking_rai_alg_node_h_

//std
#include <sstream>
#include <string>
#include <fstream>

//library
//#include "position3d.h"
#include "peopleTracker.h"

//iri-ros
#include <iri_base_algorithm/iri_base_algorithm.h>
#include "people_tracking_rai_alg.h"

//required headers for image I/O
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// [publisher subscriber headers]
#include <std_msgs/Int32.h>
#include <tld_msgs/BoundingBox.h>
#include <tld_msgs/Target.h>
#include <pal_vision_msgs/FaceDetections.h>
#include <pal_vision_msgs/HogDetections.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <iri_perception_msgs/peopleTrackingArray.h>
#include <pal_vision_msgs/LegDetections.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// [service client headers]

// [action server client headers]

//visualization constants 
const double MARKER_SIZE = 0.5;
const double MARKER_Z = 0.2;
const double MARKER_DURATION = 0.1;
const double MARKER_TEXT_SIZE = 0.3;
const double MARKER_TRANSPARENCY = 0.9;

//node execution mode
enum executionModes {MULTI_TRACKING=0, SHOOT_TLD, FOLLOW_ME};

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PeopleTrackingRaiAlgNode : public algorithm_base::IriBaseAlgorithm<PeopleTrackingRaiAlgorithm>
{
  private:
      //execution mode
      unsigned int exeMode;
        
      // initialize tf from base_link to camera, and get camera matrix
      void initCamera();
      
      //indicates when the TLD request message has been filled
      bool tldMessageFilled;
      
      //useful to initialize base_link to camera transform
      //tf::TransformListener tfListener;
        
      // image transport 
	image_transport::ImageTransport it_;
	cv_bridge::CvImagePtr cv_ptr;
	  
    // [publisher attributes]
      ros::Publisher tldBB_publisher_;
      tld_msgs::Target tldBB_msg_;
	ros::Publisher particleSet_publisher_;
	visualization_msgs::MarkerArray MarkerArray_msg_;
	ros::Publisher peopleSet_publisher_;
	iri_perception_msgs::peopleTrackingArray peopleTrackingArray_msg_;
	image_transport::Publisher image_pub_;

    // [subscriber attributes]
      ros::Subscriber followMe_subscriber_;
      void followMe_callback(const std_msgs::Int32::ConstPtr& msg);
      CMutex followMe_mutex_;
      ros::Subscriber tldDetections_subscriber_;
      void tldDetections_callback(const tld_msgs::BoundingBox::ConstPtr& msg);
      CMutex tldDetections_mutex_;            
      ros::Subscriber faceDetections_subscriber_;
      void faceDetections_callback(const pal_vision_msgs::FaceDetections::ConstPtr& msg);
      CMutex faceDetections_mutex_;      
	ros::Subscriber bodyDetections_subscriber_;
	void bodyDetections_callback(const pal_vision_msgs::HogDetections::ConstPtr& msg);
	CMutex bodyDetections_mutex_;
	ros::Subscriber odometry_subscriber_;
	void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
	CMutex odometry_mutex_;
	ros::Subscriber legDetections_subscriber_;
	void legDetections_callback(const pal_vision_msgs::LegDetections::ConstPtr& msg);
	CMutex legDetections_mutex_;
	image_transport::Subscriber image_sub_;
	void image_callback(const sensor_msgs::ImageConstPtr& msg);
	CMutex image_mutex_;
	
    //dynamic reconfigure mutex
	CMutex config_mutex;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]
    
    //library objects
	pFilterParameters filterParams;
	trackerParameters trackerParams;
	CpeopleTracker tracker;
	CodometryObservation platformOdometry;    
	
    //debugging
	std::ofstream hogFile;
	bool verboseMode;
	bool viewBodyDetections;
	double ratioParticlesDisplayed;
      unsigned int frameCount, hogDetCount;

  public:
	/**
	* \brief Constructor
	* 
	* This constructor initializes specific class attributes and all ROS
	* communications variables to enable message exchange.
	*/
	PeopleTrackingRaiAlgNode(void);

	/**
	* \brief Destructor
	* 
	* This destructor frees all necessary dynamic memory allocated within this
	* this class.
	*/
	~PeopleTrackingRaiAlgNode(void);
	
	/**
	* \brief Fill output messages
	* 
	* Fills main output and debug messages
	*/
	void fillMessages();

  protected:
	/**
	* \brief main node thread
	*
	* This is the main thread node function. Code written here will be executed
	* in every node loop while the algorithm is on running state. Loop frequency 
	* can be tuned by modifying loop_rate attribute.
	*
	* Here data related to the process loop or to ROS topics (mainly data structs
	* related to the MSG and SRV files) must be updated. ROS publisher objects 
	* must publish their data in this process. ROS client servers may also
	* request data to the corresponding server topics.
	*/
	void mainNodeThread(void);

	/**
	* \brief dynamic reconfigure server callback
	* 
	* This method is called whenever a new configuration is received through
	* the dynamic reconfigure. The derivated generic algorithm class must 
	* implement it.
	*
	* \param config an object with new configuration from all algorithm 
	*               parameters defined in the config file.
	* \param level  integer referring the level in which the configuration
	*               has been changed.
	*/
	void node_config_update(Config &config, uint32_t level);

	/**
	* \brief node add diagnostics
	*
	* In this abstract function additional ROS diagnostics applied to the 
	* specific algorithms may be added.
	*/
	void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
