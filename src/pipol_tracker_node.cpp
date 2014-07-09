#include "pipol_tracker_node.h"

CpipolTrackerNode::CpipolTrackerNode() : nh(ros::this_node::getName()) , it(this->nh)
{
      int intParam, year;
           
      //general prupose variables
      //tldMessageFilled = false;
      exeMode = MULTI_TRACKING;
      std::cout << "TRACKER EXE MODE INIT TO: " << exeMode << std::endl;
      tracker.setFollowMeTargetId(-1);//indicates no follow me target yet
	
      //init user parameters. general running parameters
      nh.getParam("verbose_mode", intParam); this->verboseMode = (bool)intParam;
      
      //init user parameters. filter parameters
      nh.getParam("num_particles", intParam); this->filterParams.numParticles = (unsigned int)intParam;
      nh.getParam("init_delta_xy", this->filterParams.initDeltaXY);
      nh.getParam("init_delta_vxy", this->filterParams.initDeltaVxy);
      nh.getParam("sigma_resampling_xy", this->filterParams.sigmaResamplingXY);
      nh.getParam("sigma_ratio_resampling_vxy", this->filterParams.sigmaRatioResamplingVxy);
      nh.getParam("sigma_min_resampling_vxy", this->filterParams.sigmaMinResamplingVxy);
      nh.getParam("person_radius_legs", this->filterParams.personRadiusLegs);
      nh.getParam("person_radius_body", this->filterParams.personRadiusBody);
      nh.getParam("matching_legs_alpha", this->filterParams.matchingLegsAlpha);
      nh.getParam("matching_legs_beta", this->filterParams.matchingLegsBeta);
      nh.getParam("matching_bearing_alpha", this->filterParams.matchingBearingAlpha);
      nh.getParam("matching_bearing_beta", this->filterParams.matchingBearingBeta);
      nh.getParam("matching_body3d_alpha", this->filterParams.matchingBody3dAlpha);
      nh.getParam("matching_body3d_beta", this->filterParams.matchingBody3dBeta);      
//    nh.getParam("power_rgb_cos", intParam); this->filterParams.powerRGBcos = (unsigned int)intParam;

      //init user parameters. tracker parameters
      nh.getParam("minimum_distance_between_people", this->trackerParams.minDistanceBetweenPeople);
      nh.getParam("minimum_association_prob", this->trackerParams.minAssociationProb);
      nh.getParam("max_detection_distance_accepted", this->trackerParams.maxDetectionDistance);
      nh.getParam("min_detection_distance_accepted", this->trackerParams.minDetectionDistance);
      nh.getParam("max_detection_azimut_accepted", this->trackerParams.maxDetectionAzimut);      
      this->trackerParams.maxDetectionAzimut = this->trackerParams.maxDetectionAzimut*M_PI/180.;
      nh.getParam("max_consecutive_uncorrected_iterations", intParam); this->trackerParams.maxConsecutiveUncorrected = (unsigned int)intParam;
      nh.getParam("minimum_iterations_to_be_target", intParam); this->trackerParams.minIterationsToBeTarget = (unsigned int)intParam;
      nh.getParam("minimum_iterations_to_be_visually_confirmed", intParam); this->trackerParams.iterationsToBeVisuallyConfirmed = (unsigned int)intParam;
      nh.getParam("minimum_iterations_to_be_friend", intParam); this->trackerParams.iterationsToBeFriend = (unsigned int)intParam;                
      nh.getParam("minimum_appearance_region_size", intParam); this->trackerParams.minAppearanceRegionSize = (unsigned int)intParam;
      nh.getParam("tracker_rate", this->rate_);

      //visualization parameters
      nh.getParam("view_body_detections", intParam); this->viewBodyDetections = (bool)intParam;
      nh.getParam("view_particles", intParam); this->viewParticles = (bool)intParam;
      //nh.getParam("ratio_particles_displayed", this->ratioParticlesDisplayed);
      
      //set parameters to tracker
      this->tracker.setParameters(trackerParams);
      this->tracker.setFilterParameters(filterParams);

      //initializes random generator, for visualization purposes (particle visualization ratio);
      srand ( time(NULL) );
      
      //debug counters
      //frameCount = 0;
      //hogDetCount = 0;
	
      // init publishers
	this->particleSetPub = nh.advertise<visualization_msgs::MarkerArray>("debug", 100);
	this->peopleSetPub = nh.advertise<pipol_tracker_pkg::personArray>("peopleSet", 100);
	this->imagePub = it.advertise("image_out", 1);
      //this->tldBB_publisher_ = nh.advertise<tld_msgs::Target>("tld_init", 1000, true);
  
      // init subscribers
      this->odometrySubs = nh.subscribe("odometry", 100, &CpipolTrackerNode::odometry_callback, this);
      this->followMeSubs = nh.subscribe("followMe", 100, &CpipolTrackerNode::followMe_callback, this);      
      this->imageSubs = it.subscribe("image_in", 1, &CpipolTrackerNode::image_callback, this);  
      this->cameraInfoSubs = nh.subscribe("cameraInfo_in", 100, &CpipolTrackerNode::cameraInfo_callback, this);
      
      //init subscribers (year dependent)
      nh.getParam("year", year);
      switch ( year ) //according year , some messages changed. This assure the execution of 2013 rosbags. Default is 2014. 
      {
            case 2013:
                  this->legDetectionsSubs = nh.subscribe("legDetections", 100, &CpipolTrackerNode::legDetections_callback_2013, this);
                  this->bodyDetectionsSubs = nh.subscribe("bodyDetections", 100, &CpipolTrackerNode::bodyDetections_callback_2013, this);
                  this->faceDetectionsSubs = nh.subscribe("faceDetections", 100, &CpipolTrackerNode::faceDetections_callback_2013, this);
                  //this->tldDetectionsSubs = nh.subscribe("tldDetections", 100, &pipolTrackerNode::tldDetections_callback, this);           
                  break;
            case 2014:
                  this->legDetectionsSubs = nh.subscribe("legDetections", 100, &CpipolTrackerNode::legDetections_callback, this);
                  this->bodyDetectionsSubs = nh.subscribe("bodyDetections", 100, &CpipolTrackerNode::bodyDetections_callback, this);
                  this->faceDetectionsSubs = nh.subscribe("faceDetections", 100, &CpipolTrackerNode::faceDetections_callback, this);
                  this->body3dDetectionsSubs = nh.subscribe("body3dDetections", 100, &CpipolTrackerNode::body3dDetections_callback, this);
                  break;
            default:
                  break;
      }
}

CpipolTrackerNode::~CpipolTrackerNode()
{
      // free allocated memory resources, if any
}

void CpipolTrackerNode::process()
{	
      if (this->verboseMode) std::cout << std::endl << "************* NEW ITERATION **************" << std::endl;

      //LOCK data reception mutexes
//       this->legDetections_mutex_.enter(); 
//       this->bodyDetections_mutex_.enter(); 
//       this->faceDetections_mutex_.enter(); 
//       this->tldDetections_mutex_.enter(); 
//       this->followMe_mutex_.enter(); 
//       this->image_mutex_.enter();

      //PRINT DETECTIONS AVAILABLE AT THIS ITERATION
      if (this->verboseMode) tracker.printDetectionSets();
      
      //FILTER PREDICTION
      if (this->verboseMode) std::cout << std::endl << "*** Prior peopleSet:" << std::endl;
      //this->odometry_mutex_.enter(); 
      tracker.propagateFilters(platformOdometry); //platform motion
      platformOdometry.resetDeltas();
      //this->odometry_mutex_.exit(); 
      tracker.propagateFilters(); //people motion
      
      //OCCLUSIONS
      tracker.computeOcclusions();
      
      //UPDATE STATUS & ESTIMATES
      tracker.updateFilterEstimates();
      tracker.updateTargetStatus();
      if (this->verboseMode) tracker.printPeopleSet();

      //DATA ASSOCIATION
      //if (this->verboseMode) std::cout << std::endl << "*** Target/Detection association" << std::endl;
      tracker.updateAssociationTables();
            
      //MARK BOUNDING BOXES OF VISUAL DETECTIONS (& LEARN CURRENT DETECTED APPEARANCES -> TO DO !!)
      if ( cvImgPtrSubs!=NULL )
      {
            //debug
            //frameCount++;
            //std::cout << "det/frames = " << (double)hogDetCount/(double)frameCount << std::endl;
            
            //Set image to the tracker
            tracker.setCurrentImage(cvImgPtrSubs->image);
            
            //compute appearances of body detections
            //tracker.computeTargetAppearance();
            
            //mark box bodies on image
            tracker.markBodies();            
            
            //mark box faces on image
            tracker.markFaces();
            
            //mark tld box
            //tracker.markTld();
            
            //get marked image from the tracker
            tracker.getCurrentImage(cvImgPub.image);
      }                                
                        
      //CORRECTION
      if (this->verboseMode) std::cout << std::endl << "*** Posterior peopleSet:" << std::endl;
      tracker.correctFilters();
            
      //UPDATE FILTER ESTIMATES AND ADD THEM TO EACH TARGET TRACK
      tracker.updateFilterEstimates();
      tracker.addEstimatesToTracks();
      if (this->verboseMode) tracker.printPeopleSet();
                  
      //LAUNCH NEW FILTERS IF NEW DETECTIONS ARE NOT ASSOCIATED
      if (this->verboseMode) std::cout << std::endl << "*** Create Filters" << std::endl;
      tracker.createFilters();

      //REMOVE UNSUPPORTED TARGETS
      if (this->verboseMode) std::cout << std::endl << "*** Delete Filters" << std::endl;
      tracker.deleteFilters();
            
      //RESAMPLING PARTICLE SETS 
      if (this->verboseMode) std::cout << std::endl << "*** Resampling" << std::endl;
      tracker.resampleFilters();
      
      //Check if TLD tracker can be initialized, if so, initTLD
//       if ( (exeMode == MULTI_TRACKING) && (tracker.getFollowMeTargetId()>0) )
//       {
//             exeMode = SHOOT_TLD;
//             std::cout << "TRACKER EXE MODE UPDATED TO: " << exeMode << "(" << __LINE__ << ")" << std::endl;
//             tracker.initTLD();
//       }

      // fill msg structures
      if (this->verboseMode) std::cout << std::endl << "*** Filling Output Messages" << std::endl;
      this->fillMessages();

      //RESET DETECTION SETS
      tracker.resetDetectionSets(LEGS);
      tracker.resetDetectionSets(BODY);
      tracker.resetDetectionSets(FACE);
      tracker.resetDetectionSets(BODY3D);
      //tracker.resetDetectionSets(TLD);

      //UNLOCK data reception mutexes (except image mutex which will be unlocked below)
//       this->legDetections_mutex_.exit(); 
//       this->bodyDetections_mutex_.exit(); 
//       this->faceDetections_mutex_.exit();
//       this->tldDetections_mutex_.exit();
//       this->followMe_mutex_.exit(); 

      // publish messages
      this->particleSetPub.publish(this->MarkerArrayMsg);
      this->peopleSetPub.publish(this->personArrayMsg);
      if ( &cvImgPub != NULL ) imagePub.publish(cvImgPub.toImageMsg());
//       if ( tldMessageFilled )
//       {
//             tldMessageFilled = false;
//             exeMode = FOLLOW_ME;
//             std::cout << "TRACKER EXE MODE UPDATED TO: " << exeMode << "(" << __LINE__ << ")" << std::endl;
//             tldBB_publisher_.publish(tldBB_msg_);//this message is published once, just to start-up tld tracker
//       }

      //unlock image mutex
      //this->image_mutex_.exit();        
}

void CpipolTrackerNode::fillMessages()
{
    std::list<CpersonTarget>::iterator iiF;
    std::list<CpersonParticle>::iterator iiP;
    std::list<CbodyObservation>::iterator iiB;
    std::list<Cpoint3dObservation>::iterator iiL;
    std::list<Cpoint3dObservation>::iterator iiB3;
    filterEstimate iiEst;
    ostringstream markerText;
    CbodyObservation tldDet;
    double bodyBearing, transpFactor, vOrientation;
    unsigned int ii=0, jj=0;
    unsigned int bbx,bby,bbw,bbh;
    
    //1. Main output message: peopleTrackingArray
    personArrayMsg.peopleSet.clear();
    personArrayMsg.header.frame_id = "/base_link";
    personArrayMsg.header.stamp = ros::Time::now();
    std::list<CpersonTarget> & targets = tracker.getTargetList();
    personArrayMsg.peopleSet.resize(targets.size());
    for (iiF=targets.begin(); iiF!=targets.end(); iiF++)
    {
        if ( iiF->getMaxStatus() > CANDIDATE )
        {
            iiF->getEstimate(iiEst);
            personArrayMsg.peopleSet[ii].targetId = iiF->getId();
            personArrayMsg.peopleSet[ii].targetStatus = iiF->getStatus();
            personArrayMsg.peopleSet[ii].x = iiEst.position.getX();
            personArrayMsg.peopleSet[ii].y = iiEst.position.getY();
            personArrayMsg.peopleSet[ii].vx = iiEst.velocity.getX();
            personArrayMsg.peopleSet[ii].vy = iiEst.velocity.getY();
            personArrayMsg.peopleSet[ii].covariances[0] = iiEst.position.getMatrixElement(0,0);
            personArrayMsg.peopleSet[ii].covariances[5] = iiEst.position.getMatrixElement(1,1);
            personArrayMsg.peopleSet[ii].covariances[10] = iiEst.velocity.getMatrixElement(0,0);
            personArrayMsg.peopleSet[ii].covariances[15] = iiEst.velocity.getMatrixElement(1,1);
            ii++;
        }
    }
      
    //erase message data if previous iteration had greater array size
    personArrayMsg.peopleSet.erase(personArrayMsg.peopleSet.begin()+ii,personArrayMsg.peopleSet.end());
    
    //2. VISUALIZATION MESSAGE: Marker array
    if (this->verboseMode) std::cout << std::endl << "*** Filling Debug Markers Message" << std::endl;
    std::list<Cpoint3dObservation> & laserDetSet = tracker.getLaserDetSet();
    std::list<CbodyObservation> & bodyDetSet = tracker.getBodyDetSet();
    std::list<Cpoint3dObservation> & body3dDetSet = tracker.getBody3dDetSet();
    unsigned int markerArraySize;
    markerArraySize = laserDetSet.size() + bodyDetSet.size() + body3dDetSet.size() + targets.size()*3;
    if ( this->viewParticles ) markerArraySize += targets.size()*30; //will send 30 markers corresponding to 30 particles per target
    MarkerArrayMsg.markers.clear();
    MarkerArrayMsg.markers.resize( markerArraySize );
    ii = 0;
    //if (this->verboseMode) std::cout << "\tlaserDetSet.size(): " << laserDetSet.size() << "\tbodyDetSet.size(): " << bodyDetSet.size() << "\ttargets.size(): " << targets.size() << "\tMarkerArrayMsg.markers.size() " << MarkerArrayMsg.markers.size() << std::endl; 

    //2a. laser detections
    for (iiL=laserDetSet.begin(); iiL!=laserDetSet.end(); iiL++)
    {
        //if (this->verboseMode) std::cout << "mainNodeThread: " << __LINE__ << std::endl;
        this->MarkerArrayMsg.markers[ii].header.frame_id = "/base_link";
        this->MarkerArrayMsg.markers[ii].header.stamp = ros::Time::now();
        this->MarkerArrayMsg.markers[ii].id = ii;
        this->MarkerArrayMsg.markers[ii].type = visualization_msgs::Marker::CYLINDER;
        this->MarkerArrayMsg.markers[ii].action = visualization_msgs::Marker::ADD;
        this->MarkerArrayMsg.markers[ii].pose.position.x = iiL->point.getX();
        this->MarkerArrayMsg.markers[ii].pose.position.y = iiL->point.getY();
        this->MarkerArrayMsg.markers[ii].pose.position.z = 0.;
        this->MarkerArrayMsg.markers[ii].scale.x = MARKER_SIZE/3;
        this->MarkerArrayMsg.markers[ii].scale.y = MARKER_SIZE/3;
        this->MarkerArrayMsg.markers[ii].scale.z = MARKER_SIZE/3;
        this->MarkerArrayMsg.markers[ii].color.a = MARKER_TRANSPARENCY;
        this->MarkerArrayMsg.markers[ii].color.r = 1;
        this->MarkerArrayMsg.markers[ii].color.g = 0.2;
        this->MarkerArrayMsg.markers[ii].color.b = 0.2;
        this->MarkerArrayMsg.markers[ii].lifetime = ros::Duration(MARKER_DURATION);
        ii++;                   
    }
    
    //2b. body detections, both 2D and 3D
    if (this->viewBodyDetections)
    {
        //2D
        for (iiB=bodyDetSet.begin(); iiB!=bodyDetSet.end(); iiB++)
        {
            //if (this->verboseMode) std::cout << "mainNodeThread: " << __LINE__ << std::endl;
            //arrow indicates detection bearing
            this->MarkerArrayMsg.markers[ii].header.frame_id = "/base_link";
            this->MarkerArrayMsg.markers[ii].header.stamp = ros::Time::now();
            this->MarkerArrayMsg.markers[ii].id = ii;
            this->MarkerArrayMsg.markers[ii].type = visualization_msgs::Marker::ARROW;
            this->MarkerArrayMsg.markers[ii].action = visualization_msgs::Marker::ADD;
            this->MarkerArrayMsg.markers[ii].pose.position.x = 0;
            this->MarkerArrayMsg.markers[ii].pose.position.y = 0;
            this->MarkerArrayMsg.markers[ii].pose.position.z = MARKER_Z;
            this->MarkerArrayMsg.markers[ii].scale.x = MARKER_SIZE*10;
            this->MarkerArrayMsg.markers[ii].scale.y = MARKER_SIZE/10;
            this->MarkerArrayMsg.markers[ii].scale.z = MARKER_SIZE/10;
            this->MarkerArrayMsg.markers[ii].color.a = MARKER_TRANSPARENCY;
            this->MarkerArrayMsg.markers[ii].color.r = 0;
            this->MarkerArrayMsg.markers[ii].color.g = 1;
            this->MarkerArrayMsg.markers[ii].color.b = 1;
            this->MarkerArrayMsg.markers[ii].lifetime = ros::Duration(MARKER_DURATION*3);
            bodyBearing = atan2(iiB->direction.getY(),iiB->direction.getX());
            geometry_msgs::Quaternion bearingQuaternion = tf::createQuaternionMsgFromYaw(bodyBearing);
            this->MarkerArrayMsg.markers[ii].pose.orientation.x = bearingQuaternion.x;
            this->MarkerArrayMsg.markers[ii].pose.orientation.y = bearingQuaternion.y;
            this->MarkerArrayMsg.markers[ii].pose.orientation.z = bearingQuaternion.z;
            this->MarkerArrayMsg.markers[ii].pose.orientation.w = bearingQuaternion.w;
            ii++;
        }
        
        //3D
        //std::cout << "body3dDetSet.size(): "<< body3dDetSet.size() << std::endl;
        for (iiB3=body3dDetSet.begin(); iiB3!=body3dDetSet.end(); iiB3++)
        {
            //if (this->verboseMode) std::cout << "mainNodeThread: " << __LINE__ << std::endl;
            //arrow indicates detection bearing
            this->MarkerArrayMsg.markers[ii].header.frame_id = "/base_link";
            this->MarkerArrayMsg.markers[ii].header.stamp = ros::Time::now();
            this->MarkerArrayMsg.markers[ii].id = ii;
            this->MarkerArrayMsg.markers[ii].type = visualization_msgs::Marker::CUBE;
            this->MarkerArrayMsg.markers[ii].action = visualization_msgs::Marker::ADD;
            this->MarkerArrayMsg.markers[ii].pose.position.x = iiB3->point.getX();
            this->MarkerArrayMsg.markers[ii].pose.position.y = iiB3->point.getY();
            this->MarkerArrayMsg.markers[ii].pose.position.z = MARKER_Z*2.;
            this->MarkerArrayMsg.markers[ii].scale.x = MARKER_SIZE/3.;
            this->MarkerArrayMsg.markers[ii].scale.y = MARKER_SIZE/3.;
            this->MarkerArrayMsg.markers[ii].scale.z = MARKER_SIZE/3.;
            this->MarkerArrayMsg.markers[ii].color.a = MARKER_TRANSPARENCY;
            this->MarkerArrayMsg.markers[ii].color.r = 1.;
            this->MarkerArrayMsg.markers[ii].color.g = 0.;
            this->MarkerArrayMsg.markers[ii].color.b = 1.;
            this->MarkerArrayMsg.markers[ii].lifetime = ros::Duration(MARKER_DURATION);
            ii++;                   
        }        
    }

    //2c. target positions, target Id's and particles
    for (iiF=targets.begin(); iiF!=targets.end(); iiF++)
    {
        //if (this->verboseMode) std::cout << "mainNodeThread: " << __LINE__ << std::endl;
        if ( iiF->getMaxStatus() > CANDIDATE )
        {
            //get target position
            iiF->getEstimate(iiEst);
            
            //cylinder: target position
            this->MarkerArrayMsg.markers[ii].header.frame_id = "/base_link";
            this->MarkerArrayMsg.markers[ii].header.stamp = ros::Time::now();
            this->MarkerArrayMsg.markers[ii].id = ii;
            this->MarkerArrayMsg.markers[ii].type = visualization_msgs::Marker::CYLINDER;
            this->MarkerArrayMsg.markers[ii].action = visualization_msgs::Marker::ADD;
            this->MarkerArrayMsg.markers[ii].pose.position.x = iiEst.position.getX();
            this->MarkerArrayMsg.markers[ii].pose.position.y = iiEst.position.getY();
            if (iiF->pOcclusion > 0.5) this->MarkerArrayMsg.markers[ii].pose.position.z = 0.;
            else this->MarkerArrayMsg.markers[ii].pose.position.z = 0.;
            this->MarkerArrayMsg.markers[ii].scale.x = MARKER_SIZE;
            this->MarkerArrayMsg.markers[ii].scale.y = MARKER_SIZE;
            if (iiF->pOcclusion > 0.5) this->MarkerArrayMsg.markers[ii].scale.z = MARKER_SIZE*2;//marker vertical size twice if occluded
            else this->MarkerArrayMsg.markers[ii].scale.z = MARKER_SIZE;
            switch(iiF->getMaxStatus()) //marker transp depending on status
            {
                case LEGGED_TARGET: transpFactor = 0.1; break;
                case VISUALLY_CONFIRMED: transpFactor = 0.4; break;
                case FRIEND_IN_SIGHT: transpFactor = 0.8; break;
                //case FRIEND_OUT_OF_RANGE: transpFactor = 1; break;
                default: transpFactor = 0.1; break; 
            }
            this->MarkerArrayMsg.markers[ii].color.a = MARKER_TRANSPARENCY*transpFactor;
            this->MarkerArrayMsg.markers[ii].color.r = 0;
            this->MarkerArrayMsg.markers[ii].color.g = 0;
            this->MarkerArrayMsg.markers[ii].color.b = 1.0;
            this->MarkerArrayMsg.markers[ii].lifetime = ros::Duration(MARKER_DURATION);
            ii++;
            
            //add extra info for well confirmed targets
            if ( iiF->getMaxStatus() > LEGGED_TARGET )
            {
                //velocity arrow
                this->MarkerArrayMsg.markers[ii].header.frame_id = "/base_link";
                this->MarkerArrayMsg.markers[ii].header.stamp = ros::Time::now();
                this->MarkerArrayMsg.markers[ii].id = ii;
                this->MarkerArrayMsg.markers[ii].type = visualization_msgs::Marker::ARROW;
                this->MarkerArrayMsg.markers[ii].action = visualization_msgs::Marker::ADD;
                this->MarkerArrayMsg.markers[ii].pose.position.x = iiEst.position.getX();;
                this->MarkerArrayMsg.markers[ii].pose.position.y = iiEst.position.getY();;
                this->MarkerArrayMsg.markers[ii].pose.position.z = MARKER_Z;
                this->MarkerArrayMsg.markers[ii].scale.x = MARKER_SIZE*5*iiEst.velocity.norm();
                this->MarkerArrayMsg.markers[ii].scale.y = MARKER_SIZE/10;
                this->MarkerArrayMsg.markers[ii].scale.z = MARKER_SIZE/10;
                this->MarkerArrayMsg.markers[ii].color.a = MARKER_TRANSPARENCY;
                this->MarkerArrayMsg.markers[ii].color.r = 0;
                this->MarkerArrayMsg.markers[ii].color.g = 0;
                this->MarkerArrayMsg.markers[ii].color.b = 1;
                this->MarkerArrayMsg.markers[ii].lifetime = ros::Duration(MARKER_DURATION*3);
                vOrientation = atan2(iiEst.velocity.getY(),iiEst.velocity.getX());
                geometry_msgs::Quaternion bearingQuaternion = tf::createQuaternionMsgFromYaw(vOrientation);
                this->MarkerArrayMsg.markers[ii].pose.orientation.x = bearingQuaternion.x;
                this->MarkerArrayMsg.markers[ii].pose.orientation.y = bearingQuaternion.y;
                this->MarkerArrayMsg.markers[ii].pose.orientation.z = bearingQuaternion.z;
                this->MarkerArrayMsg.markers[ii].pose.orientation.w = bearingQuaternion.w;
                ii++; //increment marker index due to velocity arrow

                //text marker: Text is: target ID + used detections + STOP/GO mode
                this->MarkerArrayMsg.markers[ii].header.frame_id = "/base_link";
                this->MarkerArrayMsg.markers[ii].header.stamp = ros::Time::now();
                this->MarkerArrayMsg.markers[ii].id = ii;
                this->MarkerArrayMsg.markers[ii].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                this->MarkerArrayMsg.markers[ii].action = visualization_msgs::Marker::ADD;
                this->MarkerArrayMsg.markers[ii].pose.position.x = iiEst.position.getX()+0.4;
                this->MarkerArrayMsg.markers[ii].pose.position.y = iiEst.position.getY()+0.4;
                this->MarkerArrayMsg.markers[ii].pose.position.z = MARKER_Z;
                this->MarkerArrayMsg.markers[ii].scale.z = MARKER_TEXT_SIZE;
                this->MarkerArrayMsg.markers[ii].color.a = 1;
                this->MarkerArrayMsg.markers[ii].color.r = 0.0;
                this->MarkerArrayMsg.markers[ii].color.g = 0.0;
                this->MarkerArrayMsg.markers[ii].color.b = 0.0;
                this->MarkerArrayMsg.markers[ii].lifetime = ros::Duration(MARKER_DURATION);
                markerText.str("");
                markerText << iiF->getId() << "/";
                for (unsigned int jj=0; jj<NUM_DETECTORS; jj++) //add used detectors
                {
                    for (unsigned int kk=0; kk<iiF->aDecisions[jj].size(); kk++)
                    {
                        if ( iiF->aDecisions[jj].at(kk) )
                        {
                            markerText << jj;
                            break;
                        }
                    }
                }
                /*switch( iiF->getMotionMode() ) //add a label according current motion mode
                {
                    case MODE_STOP: markerText << ", STOP"; break;
                    case MODE_GO: markerText << ", GO"; break;    
                    default: break;
                } */           
                this->MarkerArrayMsg.markers[ii].text = markerText.str();
                ii++; //increment marker index due to marker text
            }

            //particles
            if ( this->viewParticles )
            {
                std::list<CpersonParticle> & pSet = iiF->getParticleSet();
                for (iiP=pSet.begin(),jj=0;iiP!=pSet.end();iiP++,jj++)
                {
                    //just publish markers corresponding to start, center and end of the set
                    if ( (jj<10) || (((pSet.size()/2-5)<jj)&&(jj<=(pSet.size()/2 + 5))) || (((pSet.size()-10)<=jj)&&(jj<(pSet.size()))) )
                    {
                        this->MarkerArrayMsg.markers[ii].header.frame_id = "/base_link";
                        this->MarkerArrayMsg.markers[ii].header.stamp = ros::Time::now();
                        this->MarkerArrayMsg.markers[ii].id = ii;
                        this->MarkerArrayMsg.markers[ii].type = visualization_msgs::Marker::CYLINDER;
                        this->MarkerArrayMsg.markers[ii].action = visualization_msgs::Marker::ADD;
                        this->MarkerArrayMsg.markers[ii].pose.position.x = iiP->position.getX();
                        this->MarkerArrayMsg.markers[ii].pose.position.y = iiP->position.getY();
                        this->MarkerArrayMsg.markers[ii].pose.position.z = MARKER_Z;
                        this->MarkerArrayMsg.markers[ii].scale.x = MARKER_SIZE/10;
                        this->MarkerArrayMsg.markers[ii].scale.y = MARKER_SIZE/10;
                        this->MarkerArrayMsg.markers[ii].scale.z = MARKER_SIZE/10;
                        this->MarkerArrayMsg.markers[ii].color.a = MARKER_TRANSPARENCY;
                        this->MarkerArrayMsg.markers[ii].color.r = 0.5;
                        this->MarkerArrayMsg.markers[ii].color.g = 1.5;
                        this->MarkerArrayMsg.markers[ii].color.b = 0.0;
                        this->MarkerArrayMsg.markers[ii].lifetime = ros::Duration(MARKER_DURATION);
                        ii++;
                    }
                }           
            }

            //std::list<CpersonParticle> & pSet = iiF->getParticleSet();
            //if (this->verboseMode) std::cout << "mainNodeThread: " << __LINE__ << "; pSet.size(): " << pSet.size() << std::endl;
/*
            for (iiP=pSet.begin();iiP!=pSet.end();iiP++)
            {
                //if (this->verboseMode) std::cout << "mainNodeThread: " << __LINE__ << "; ii: " << ii << std::endl;
                double rnd = (double)rand()/(double)RAND_MAX;
                if ( rnd < ratioParticlesDisplayed  )
                {
                    this->MarkerArrayMsg.markers[ii].header.frame_id = "/base_link";
                    this->MarkerArrayMsg.markers[ii].header.stamp = ros::Time::now();
                    this->MarkerArrayMsg.markers[ii].id = ii;
                    this->MarkerArrayMsg.markers[ii].type = visualization_msgs::Marker::CYLINDER;
                    this->MarkerArrayMsg.markers[ii].action = visualization_msgs::Marker::ADD;
                    this->MarkerArrayMsg.markers[ii].pose.position.x = iiP->position.getX();
                    this->MarkerArrayMsg.markers[ii].pose.position.y = iiP->position.getY();
                    this->MarkerArrayMsg.markers[ii].pose.position.z = MARKER_Z;
                    this->MarkerArrayMsg.markers[ii].scale.x = MARKER_SIZE/10;
                    this->MarkerArrayMsg.markers[ii].scale.y = MARKER_SIZE/10;
                    this->MarkerArrayMsg.markers[ii].scale.z = MARKER_SIZE/10;
                    this->MarkerArrayMsg.markers[ii].color.a = MARKER_TRANSPARENCY;
                    this->MarkerArrayMsg.markers[ii].color.r = 0.5;
                    this->MarkerArrayMsg.markers[ii].color.g = 1.5;
                    this->MarkerArrayMsg.markers[ii].color.b = 0.0;
                    this->MarkerArrayMsg.markers[ii].lifetime = ros::Duration(MARKER_DURATION);
                    ii++;
                }
            } */          
        }
    }
            
      
    //2d. Arrow mark for the current TLD detection
//       tracker.getTLDdetection(tldDet);
//       if (tldDet.bbW != 0) //check if a bounding box is set
//       {
//             this->MarkerArrayMsg.markers[ii].header.frame_id = "/base_link";
//             this->MarkerArrayMsg.markers[ii].header.stamp = ros::Time::now();
//             this->MarkerArrayMsg.markers[ii].id = ii;
//             this->MarkerArrayMsg.markers[ii].type = visualization_msgs::Marker::ARROW;
//             this->MarkerArrayMsg.markers[ii].action = visualization_msgs::Marker::ADD;
//             this->MarkerArrayMsg.markers[ii].pose.position.x = 0;
//             this->MarkerArrayMsg.markers[ii].pose.position.y = 0;
//             this->MarkerArrayMsg.markers[ii].pose.position.z = MARKER_Z;
//             this->MarkerArrayMsg.markers[ii].scale.x = MARKER_SIZE;
//             this->MarkerArrayMsg.markers[ii].scale.y = MARKER_SIZE;
//             this->MarkerArrayMsg.markers[ii].scale.z = MARKER_SIZE*3;
//             this->MarkerArrayMsg.markers[ii].color.r = 240./255.;
//             this->MarkerArrayMsg.markers[ii].color.g = 180./255.;
//             this->MarkerArrayMsg.markers[ii].color.b = 20./255.;      
//             this->MarkerArrayMsg.markers[ii].color.a = MARKER_TRANSPARENCY;
//             this->MarkerArrayMsg.markers[ii].lifetime = ros::Duration(MARKER_DURATION*3);
//             bodyBearing = atan2(tldDet.direction.getY(),tldDet.direction.getX());
//             geometry_msgs::Quaternion bearingQuaternion = tf::createQuaternionMsgFromYaw(bodyBearing);
//             this->MarkerArrayMsg.markers[ii].pose.orientation.x = bearingQuaternion.x;
//             this->MarkerArrayMsg.markers[ii].pose.orientation.y = bearingQuaternion.y;
//             this->MarkerArrayMsg.markers[ii].pose.orientation.z = bearingQuaternion.z;
//             this->MarkerArrayMsg.markers[ii].pose.orientation.w = bearingQuaternion.w;
//             ii++;
//       }
      
      //3. TLD message
//       if ( (exeMode == SHOOT_TLD) && (cv_ptr!=NULL) )
//       {
//             tldBB_msg_.bb.header.frame_id = "/base_link";
//             tldBB_msg_.bb.header.stamp = ros::Time::now();
//             tracker.getTLDbb(bbx,bby,bbw,bbh);
//             tldBB_msg_.bb.x = bbx;
//             tldBB_msg_.bb.y = bby;
//             tldBB_msg_.bb.width = bbw;
//             tldBB_msg_.bb.height = bbh;
//             tldBB_msg_.bb.confidence = 1.0;
//             cv_ptr->toImageMsg(tldBB_msg_.img);
//             tldMessageFilled = true;
//       }

    //erase message data if previous iteration had greater array size
    MarkerArrayMsg.markers.erase(MarkerArrayMsg.markers.begin()+ii,MarkerArrayMsg.markers.end());   
//     std::cout << "markerArraySize: " << markerArraySize << std::endl;
//     std::cout << "ii: " << ii << std::endl;
//     std::cout << "MarkerArrayMsg.markers.size(): " << MarkerArrayMsg.markers.size() << std::endl;
}

void CpipolTrackerNode::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{ 
      //ROS_INFO("pipolTrackerNode::odometry_callback: New Message Received"); 
      
      double vx,vy,vz,vTrans;
      double tLast, dT;

      //use appropiate mutex to shared variables if necessary 
      //this->odometry_mutex_.enter(); 

      //if (this->verboseMode) std::cout << msg->data << std::endl; 
      tLast = platformOdometry.timeStamp.get();
      platformOdometry.timeStamp.setToNow();
      dT = platformOdometry.timeStamp.get() - tLast;
      //if (this->verboseMode) std::cout << "******************** dT = " << dT << std::endl;
      vx = msg->twist.twist.linear.x; 
      vy = msg->twist.twist.linear.y;
      vz = msg->twist.twist.linear.z;
      vTrans = sqrt(vx*vx+vy*vy+vz*vz);  //odometry observation considers only forward velocity
      platformOdometry.accumDeltaTrans(dT*vTrans);
      platformOdometry.accumDeltaH(dT*msg->twist.twist.angular.z);
      
      //unlock previously blocked shared variables 
      //this->alg_.unlock(); 
      //this->odometry_mutex_.exit(); 
}

void CpipolTrackerNode::legDetections_callback_2013(const pal_vision_msgs::LegDetections::ConstPtr& msg) 
{ 
      Cpoint3dObservation newDetection;
        
      //sets current (received) detections
      for (unsigned int ii=0; ii<msg->position3D.size(); ii++)
      {
            newDetection.timeStamp.set(msg->header.stamp.sec, msg->header.stamp.nsec);
            newDetection.point.setXYZ(msg->position3D[ii].x, msg->position3D[ii].y, 0.0);
            newDetection.point.setXYcov(0.2,0.2,0);
            tracker.addDetection(newDetection);
      }
}

void CpipolTrackerNode::bodyDetections_callback_2013(const pal_vision_msgs::HogDetections::ConstPtr& msg) 
{ 
      unsigned int ii, jj;
      CbodyObservation newDetection;
      std::list<CbodyObservation>::iterator iiB;
      
      //sets current (received) detections
      for (ii=0; ii<msg->persons.size(); ii++)
      {
		//time stamp
		newDetection.timeStamp.set(msg->header.stamp.sec, msg->header.stamp.nsec);
		
		//bounding box
		newDetection.bbX = msg->persons[ii].imageBoundingBox.x;
		newDetection.bbY = msg->persons[ii].imageBoundingBox.y;
		newDetection.bbW = msg->persons[ii].imageBoundingBox.width;
		newDetection.bbH = msg->persons[ii].imageBoundingBox.height;
		
		//direction
		newDetection.direction.setXYZ(msg->persons[ii].direction.x, msg->persons[ii].direction.y, msg->persons[ii].direction.z);
		
		//rgb Eigen
		newDetection.rgbEigen.setXYZ(msg->persons[ii].principalEigenVectorRGB.r, msg->persons[ii].principalEigenVectorRGB.g, msg->persons[ii].principalEigenVectorRGB.b);
		
		//rgb centers
		newDetection.rgbCenters.resize(msg->persons[ii].rgbClusterCenters.size());
		for(jj=0; jj<newDetection.rgbCenters.size(); jj++)
		{
			newDetection.rgbCenters.at(jj).setXYZ(msg->persons[ii].rgbClusterCenters[jj].r, msg->persons[ii].rgbClusterCenters[jj].g, msg->persons[ii].rgbClusterCenters[jj].b);
		}	
		
		//hog descriptor
		newDetection.hog.resize(msg->persons[ii].hog.size());
		for(jj=0; jj<newDetection.hog.size(); jj++)
		{
			newDetection.hog[jj] = msg->persons[ii].hog[jj];
			//hogFile << msg->persons[ii].hog[jj] << " ";
		}
		//hogFile << std::endl;
		
		//rgbDescriptors
		if (this->verboseMode) std::cout << "rgbDescriptor1.size(): " << msg->persons[ii].rgbDescriptor1.size() << std::endl;
				
		//add detection to tracker list
		tracker.addDetection(newDetection);
	}
}

void CpipolTrackerNode::faceDetections_callback_2013(const pal_vision_msgs::FaceDetections::ConstPtr& msg) 
{ 
      unsigned int ii;
      CfaceObservation newDetection;

      //sets current (received) detections
      for (ii=0; ii<msg->faces.size(); ii++)
      {
            //set time stamp
            newDetection.timeStamp.set(msg->header.stamp.sec, msg->header.stamp.nsec);
            
            //get message data
            newDetection.faceLoc.setX(msg->faces[ii].position3D.x);
            newDetection.faceLoc.setY(msg->faces[ii].position3D.y);
            newDetection.faceLoc.setZ(msg->faces[ii].position3D.z);
            newDetection.bbX = msg->faces[ii].imageBoundingBox.x;
            newDetection.bbY = msg->faces[ii].imageBoundingBox.y;
            newDetection.bbW = msg->faces[ii].imageBoundingBox.width;
            newDetection.bbH = msg->faces[ii].imageBoundingBox.height;
            
            //add detection to tracker list
            tracker.addDetection(newDetection);
      }
}

void CpipolTrackerNode::legDetections_callback(const pal_detection_msgs::LegDetections::ConstPtr& msg) 
{ 
      unsigned int ii;
      Cpoint3dObservation newDetection;
      std::string frame_id;      
      Eigen::Vector3d tlaser_base; //translation of laser device wrt base
      Eigen::Quaterniond qlaser_base; //quaternion of laser device wrt base
      Eigen::Vector3d det_laser; //detection wrt laser frame
      Eigen::Vector3d det_base; //detection wrt laser frame
            
      //get msg->laser_pose.frame_id and check if it is the base_link. If not, warn and exit callback
      frame_id = msg->laser_pose.header.frame_id;
      if (frame_id != "base_link")
      {
            std::cout << "WARNING: Leg detections not referenced to base_link" << std::endl;
            return;
      }

      //sensor frame pose (translation + orientation in quaternion form)
      tlaser_base << msg->laser_pose.transform.translation.x,
                  msg->laser_pose.transform.translation.y, 
                  msg->laser_pose.transform.translation.z;
      qlaser_base = Eigen::Quaterniond(msg->laser_pose.transform.rotation.w,
                                    msg->laser_pose.transform.rotation.x,
                                    msg->laser_pose.transform.rotation.y,
                                    msg->laser_pose.transform.rotation.z);
      
      //sets current (received) detections
      for (unsigned int ii=0; ii<msg->position3D.size(); ii++)
      {
            newDetection.timeStamp.set(msg->header.stamp.sec, msg->header.stamp.nsec);
            det_laser << msg->position3D[ii].x, msg->position3D[ii].y, 0.0;
            det_base = qlaser_base.matrix()*det_laser + tlaser_base;
            newDetection.point.setXYZ(det_base(0), det_base(1), 0.0);
            newDetection.point.setXYcov(0.2,0.2,0);
            tracker.addDetection(newDetection);
      }
}

void CpipolTrackerNode::bodyDetections_callback(const pal_detection_msgs::Detections2d::ConstPtr& msg) 
{ 
      unsigned int ii;
      CbodyObservation newDetection;
      std::string frame_id;
      Eigen::Vector3d pxDetection;//homogeneous pixel coordinates of detection central point
      Eigen::Vector3d ray_cam; //ray direction in 3D, wrt camera frame
      Eigen::Vector3d ray_base; //ray direction in 3D, wrt base_link frame
      Eigen::Vector3d tcam_base; //translation of camera wrt base
      Eigen::Quaterniond qcam_base; //quaternion of camera wrt base

      //get msg->camera_pose.frame_id and check if it is the base_link. If not, warn and exit callback
      frame_id = msg->camera_pose.header.frame_id;
      if (frame_id != "base_link")
      {
            std::cout << "WARNING: Body detections not referenced to base_link" << std::endl;
            return;
      }

      //sensor frame pose (translation + orientation in quaternion form)
      tcam_base << msg->camera_pose.transform.translation.x,
                  msg->camera_pose.transform.translation.y, 
                  msg->camera_pose.transform.translation.z;
      qcam_base = Eigen::Quaterniond(msg->camera_pose.transform.rotation.w,
                                    msg->camera_pose.transform.rotation.x,
                                    msg->camera_pose.transform.rotation.y,
                                    msg->camera_pose.transform.rotation.z);
      
      //Loop over all available detections to get them, and compute 3D directions
      for (ii=0; ii<msg->detections.size(); ii++)
      {
            //set detection TS
            newDetection.timeStamp.set(msg->header.stamp.sec, msg->header.stamp.nsec);
            
            //bounding box
            newDetection.bbX = msg->detections[ii].x;
            newDetection.bbY = msg->detections[ii].y;
            newDetection.bbW = msg->detections[ii].width;
            newDetection.bbH = msg->detections[ii].height;
                              
            //set homogeneous vector for pixel coordinate detection
            pxDetection << msg->detections[ii].x+0.5*msg->detections[ii].width, 
                           msg->detections[ii].y+0.5*msg->detections[ii].width,
                           1;
            
            //From pixel coordinates and camera matrix K, compute ray_cam
            ray_cam = camK.inverse()*pxDetection;
            //std::cout << "ray_cam: " << ray_cam.transpose() << std::endl;
                        
            // with qcam_base, tcam_base and ray_cam, compute ray_base
            ray_base = qcam_base.matrix()*ray_cam + tcam_base;
            //std::cout << "ray_base: " << ray_base.transpose() << std::endl;
            
            // set newDetection.direction
            newDetection.direction.setXYZ(ray_base(0), ray_base(1), ray_base(2));
                                   
            //add detection to tracker list
            tracker.addDetection(newDetection);
      }
}

void CpipolTrackerNode::faceDetections_callback(const pal_detection_msgs::FaceDetections::ConstPtr& msg) 
{ 
      unsigned int ii;
      CfaceObservation newDetection;
      std::string frame_id;      
      Eigen::Vector3d tcam_base; //translation of camera device wrt base
      Eigen::Quaterniond qcam_base; //quaternion of camera device wrt base
      Eigen::Vector3d det_cam; //detection wrt laser frame
      Eigen::Vector3d det_base; //detection wrt laser frame      
      
      //get msg->camera_pose.frame_id and check if it is the base_link. If not, warn and exit callback
      frame_id = msg->camera_pose.header.frame_id;
      if (frame_id != "base_link")
      {
            std::cout << "WARNING: Face detections not referenced to base_link" << std::endl;
            return;
      }

      //sensor frame pose (translation + orientation in quaternion form)
      tcam_base << msg->camera_pose.transform.translation.x,
                  msg->camera_pose.transform.translation.y, 
                  msg->camera_pose.transform.translation.z;
      qcam_base = Eigen::Quaterniond(msg->camera_pose.transform.rotation.w,
                                    msg->camera_pose.transform.rotation.x,
                                    msg->camera_pose.transform.rotation.y,
                                    msg->camera_pose.transform.rotation.z);
      
      //sets current (received) detections
      for (ii=0; ii<msg->faces.size(); ii++)
      {
            //set time stamp
            newDetection.timeStamp.set(msg->header.stamp.sec, msg->header.stamp.nsec);
            
            //compute transf from camera coordinates to base coordinates
            det_cam << msg->faces[ii].position.x, msg->faces[ii].position.y, msg->faces[ii].position.z;
            det_base = qcam_base.matrix()*det_cam + tcam_base;

            //Set detection
            newDetection.faceLoc.setXYZ(det_base(0), det_base(1), det_base(2));
            newDetection.bbX = msg->faces[ii].x;
            newDetection.bbY = msg->faces[ii].y;
            newDetection.bbW = msg->faces[ii].width;
            newDetection.bbH = msg->faces[ii].height;

            //add detection to tracker list
            tracker.addDetection(newDetection);
      }
}

void CpipolTrackerNode::body3dDetections_callback(const pal_detection_msgs::PersonDetections::ConstPtr& msg)
{
    unsigned int ii;
    Cpoint3dObservation newDetection;
    std::string frame_id;      
    Eigen::Vector3d tcam_base; //translation of camera device wrt base
    Eigen::Quaterniond qcam_base; //quaternion of camera device wrt base
    Eigen::Vector3d det_cam; //detection wrt camera frame
    Eigen::Vector3d det_base; //detection wrt base frame      
    
    //get msg->camera_pose.frame_id and check if it is the base_link. If not, warn and exit callback
    frame_id = msg->camera_pose.header.frame_id;
    if (frame_id != "base_link")
    {
        std::cout << "WARNING: Body 3d detections not referenced to base_link" << std::endl;
        return;
    }

    //sensor frame pose (translation + orientation in quaternion form)
    tcam_base << msg->camera_pose.transform.translation.x,
                msg->camera_pose.transform.translation.y, 
                msg->camera_pose.transform.translation.z;
    qcam_base = Eigen::Quaterniond(msg->camera_pose.transform.rotation.w,
                                msg->camera_pose.transform.rotation.x,
                                msg->camera_pose.transform.rotation.y,
                                msg->camera_pose.transform.rotation.z);
    
    //sets current (received) detections
    tracker.resetDetectionSets(BODY3D);//required since this callback can be executed faster than tracker itself
    for (ii=0; ii<msg->persons.size(); ii++)
    {
        //set time stamp
        newDetection.timeStamp.set(msg->header.stamp.sec, msg->header.stamp.nsec);
        
        //empiric calibration of depth
        if ( msg->persons[ii].position3D.point.z < 1. )
            det_cam << msg->persons[ii].position3D.point.x, msg->persons[ii].position3D.point.y, msg->persons[ii].position3D.point.z;
        else  
            det_cam << msg->persons[ii].position3D.point.x, msg->persons[ii].position3D.point.y, msg->persons[ii].position3D.point.z*4.1/3.5;
 
        det_base = qcam_base.matrix()*det_cam + tcam_base;

        //Set detection
        newDetection.point.setXYZ(det_base(0), det_base(1), 0.0); //projection to XY plane

        //add detection to tracker list
        tracker.addDetectionBody3d(newDetection);
    }

}

void CpipolTrackerNode::followMe_callback(const std_msgs::Int32::ConstPtr& msg) 
{
      tracker.setFollowMeTargetId(msg->data);
}

void CpipolTrackerNode::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
            this->cvImgPtrSubs = cv_bridge::toCvCopy(msg, msg->encoding);
            cvImgPub.encoding = msg->encoding;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void CpipolTrackerNode::cameraInfo_callback(const sensor_msgs::CameraInfo & msg)
{
      
      camK << msg.K[0],msg.K[1],msg.K[2],msg.K[3],msg.K[4],msg.K[5],msg.K[6],msg.K[7],msg.K[8];
      std::cout << "Camera Info Message Received. Camera calibration data loaded." << std::endl;
      this->cameraInfoSubs.shutdown();
}

/*
void pipolTrackerNode::tldDetections_callback(const tld_msgs::BoundingBox::ConstPtr& msg) 
{ 
      double dirX, dirY, dirMod;
      double cx,fx; //camera calibration params cx = P[0,2], fx = P[0,0].
      CbodyObservation newDetection;
      
      //hardcoded for REEM head cameras!! ToDo: get it from camera_info message !!
      cx = 286.37;
      fx = 338.49;
      
      //ROS_INFO("pipolTrackerNode::tldDetections_callback: New Message Received"); 
      
      //blocks tld detection mutex
      //this->tldDetections_mutex_.enter(); 

      //time stamp
      newDetection.timeStamp.set(msg->header.stamp.sec, msg->header.stamp.nsec);
      
      //bounding box
      newDetection.bbX = msg->x;
      newDetection.bbY = msg->y;
      newDetection.bbW = msg->width;
      newDetection.bbH = msg->height;
      
      //confidence set in the rgbEigen.X field (to be improved !)
      newDetection.rgbEigen.setX(msg->confidence);
      
      //direction (robot: X->forward, Y->left. Camera: x->horizontal, y->vertical, z->depth)
      dirX = 1.0;
      dirY = -((msg->x+msg->width/2.0)-cx)/fx;
      dirMod = sqrt(dirX*dirX+dirY*dirY);
      newDetection.direction.setXYZ(dirX/dirMod, dirY/dirMod, 0);
      
      //sets TLD detection to tracker
      tracker.setTLDdetection(newDetection);

      //unblocks tld detection mutex
      //this->tldDetections_mutex_.exit(); 
}
*/
