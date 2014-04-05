
#include "peopleTracker.h"

enum debugModeIds {TARGET_MODE = 0, TRACKER_INIT, TRACKER_LOOP, OTHERS};
const double DET_RATIO = 0.8;
const unsigned int NUM_ITERATIONS = 1000;
const double LOOP_PERIOD = 100; //loop period for this test [ms]
const double LEG_DET_SIGMA2 = 0.2;

int main(int argc, char *argv[])
{
	unsigned int ii;
	double rnd, platTrans, platRot, matchingValue;
	filterEstimate estimate;
	Cpoint3d detection, p1, p2;
	unsigned int debugMode;
	Cpoint3dObservation det1, det2, det3, det4, det5;
	CpeopleTracker myTracker;
	CodometryObservation platformOdometry;
	std::list<CpersonTarget>::iterator iiF;
	std::list<CpersonTarget> & filters = myTracker.getTargetList();
	decisionElement e1, e2, e3;
	std::list<decisionElement> deList;
	std::list<decisionElement>::iterator iiE;
	CpersonTarget filter(1);
	
	if (argc != 2)
	{
		cout << "This example requires one parameter:" << endl;
		cout << "	0 -> debugging CpersonTarget (obsolete)" << endl;
		cout << "	1 -> debugging CpeopleTracker (init's)" << endl;
		cout << "	2 -> debugging CpeopleTracker (loop)" << endl;
		cout << "	3 -> debugging others" << endl;
		return -1;
	}
	else
	{
		debugMode = atoi(argv[1]);
	}	
	
	switch (debugMode)
	{
		case TARGET_MODE:
			//target.setId(23);
			for (ii=0; ii<20; ii++)
			{
				estimate.ts.setToNow();
				estimate.position.setXYZ(ii*0.1,10+ii*0.1,0);
				estimate.position.setXYcov(0.1+ii*0.01,0.1+ii*0.01,0.05+ii*0.01);
				estimate.velocity.setXYZ(1,1,0);
				//target.addToPath(estimate);
				//target.print();
				detection.setXYZ(ii*0.1+0.1,10+ii*0.1+0.1,0);
				//ap = target.associationProb(detection);
				//std::cout << "prob = " << ap << std::endl;
				sleep(1);
			}
			break;
			
		case TRACKER_INIT:
			det1.timeStamp.setToNow();
			det1.point.setXYZ(1,1,0);
			det1.point.setXYcov(LEG_DET_SIGMA2,LEG_DET_SIGMA2,0);
			det2.timeStamp.setToNow();
			det2.point.setXYcov(LEG_DET_SIGMA2,LEG_DET_SIGMA2,0);
			det3.timeStamp.setToNow();
			det3.point.setXYcov(LEG_DET_SIGMA2,LEG_DET_SIGMA2,0);
			det4.timeStamp.setToNow();
			det4.point.setXYcov(LEG_DET_SIGMA2,LEG_DET_SIGMA2,0);
			myTracker.addDetection(det1);
			myTracker.addDetection(det2);
			myTracker.addDetection(det3);
			myTracker.addDetection(det4);
			myTracker.printDetectionSets();
			
			myTracker.updateAssociationTables();
			myTracker.createFilters();
			myTracker.deleteFilters();
			myTracker.addEstimatesToTracks();
			myTracker.printPeopleSet();

			platTrans = 0.1;
			platRot = 0.05;
			platformOdometry.accumDeltaTrans(platTrans);
			platformOdometry.accumDeltaH(platRot);
			myTracker.propagateFilters();
			//myTracker.propagateFilters(platformOdometry);
			myTracker.updateFilterEstimates();
			myTracker.addEstimatesToTracks();
			myTracker.printPeopleSet();
			
			cout << "Get filter List !" << endl;
			for (iiF=filters.begin();iiF!=filters.end();iiF++) iiF->print();
			
			break;
			
		case TRACKER_LOOP:
			for(ii=0;ii<NUM_ITERATIONS;ii++)
			{
				//simulates detections of people motion
				det1.timeStamp.setToNow();
				det1.point.setXYZ(0.05*ii,0.05*ii,0);
				det1.point.setXYcov(LEG_DET_SIGMA2,LEG_DET_SIGMA2,0);
				det2.timeStamp.setToNow();
				det2.point.setXYZ(0.05*ii,-0.05*ii,0);
				det2.point.setXYcov(LEG_DET_SIGMA2,LEG_DET_SIGMA2,0);
				det3.timeStamp.setToNow();
				det3.point.setXYZ(5,-5+0.05*ii,0);
				det3.point.setXYcov(LEG_DET_SIGMA2,LEG_DET_SIGMA2,0);				
				det4.timeStamp.setToNow();
				det4.point.setXYZ(2,3.5,0);
				det4.point.setXYcov(LEG_DET_SIGMA2,LEG_DET_SIGMA2,0);												
				det5.timeStamp.setToNow();
				det5.point.setXYZ(7,5-0.05*ii,0);
				det5.point.setXYcov(LEG_DET_SIGMA2,LEG_DET_SIGMA2,0);								
				
				//deletes previous detections
				myTracker.resetDetectionSets(LEGS);
				
				//adds detections to the tracker acoording to detection ratio
				rnd=((double)rand()) / ((double)RAND_MAX);
				if ( rnd < DET_RATIO ){myTracker.addDetection(det1);}
				rnd=((double)rand()) / ((double)RAND_MAX);
				if ( rnd < DET_RATIO ){myTracker.addDetection(det2);}
 				rnd=((double)rand()) / ((double)RAND_MAX);
 				if ( rnd < DET_RATIO ){myTracker.addDetection(det3);}
				rnd=((double)rand()) / ((double)RAND_MAX); 
				if ( rnd < DET_RATIO ){myTracker.addDetection(det4);}
				rnd=((double)rand()) / ((double)RAND_MAX); 
				if ( rnd < DET_RATIO ){myTracker.addDetection(det5);}
				
				
				//iterates tracker
				std::cout << std::endl << "************* Iteration " << ii << " ***************" << std::endl;
				std::cout << std::endl << "*** DetectionSet:" << std::endl; 
				myTracker.printDetectionSets();
				
				//myTracker.propagateFilters(platformOdometry);
				std::cout << std::endl << "*** Prior peopleSet:" << std::endl;
				myTracker.propagateFilters();
				myTracker.updateFilterEstimates();
				myTracker.printPeopleSet();
				
				std::cout << std::endl << "*** Target/Detection association" << std::endl;
				myTracker.updateAssociationTables();
				
				//std::cout << std::endl << "*** Remove cross associated particles" << std::endl;
				//myTracker.removeCrossAssociatedParticles();				

				std::cout << std::endl << "*** Posterior peopleSet:" << std::endl;
				myTracker.correctFilters();
				myTracker.updateFilterEstimates();
                        myTracker.addEstimatesToTracks();
				myTracker.printPeopleSet();
							
				std::cout << std::endl << "*** Create Filters" << std::endl;
				myTracker.createFilters();
				
				std::cout << std::endl << "*** Delete Filters" << std::endl;
				myTracker.deleteFilters();
				
				std::cout << std::endl << "*** Resampling" << std::endl;
				myTracker.resampleFilters();
				
				std::cout << "**********************************************" << std::endl;

				usleep(LOOP_PERIOD*1e3);
			}
			
		case OTHERS:
			e1.aProb = 0.3;
			e2.aProb = 0.5;
			e3.aProb = 0.4;
			deList.push_back(e1);
			deList.push_back(e2);
			deList.push_back(e3);
			deList.sort();
			for (iiE=deList.begin(); iiE!=deList.end(); iiE++)
			{
				std::cout << (iiE->aProb) << std::endl;
			}
			//std::cout << "e1<e2: " << (e1<e2) << std::endl;
			
			p1.setXYZ(3,3,0);
			p2.setXYZ(2.5,3,0);
			matchingValue = filter.bodyMatchingFunction(p1,p2);
			cout << "mV = " << matchingValue << endl;
			break;
			
		default:
			break;
	}
}
