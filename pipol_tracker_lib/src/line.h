/**
*************************************************************************
*                                                                     	*
* FILE NAME:	line.h	                                         	*
* DATE:		Jan 2007                                        	*
* VERSION:	1.0                                             	*
* PURPOSE:	Specifies line object			          	*
* CONTRIBUTORS:	Andreu Corominas Murtra (acorominas@iri.upc.edu)	*
* AFFILIATIONS:	Institut de Rob�tica Industrial (CSIC-UPC)		*
*									*
*************************************************************************
*/

#ifndef LINE_H
#define LINE_H

//include
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "point.h"
#include <string.h>
#include <vector>

//define
#define iAA 0
#define iBB 1
#define iAB 2

//data types
/**
  Cline class has two Cpoint objects dfining the origin (point aa) and the end (point bb) of a segment in a 2D space. vv is the oriented vector of the segment (from aa to bb). 
*/
class Cline
{
	private:
		Cpoint aa; /**<origin of the segment */
		Cpoint bb; /**<end of the segment*/
		Cpoint vv; /**<vector from aa to bb*/
		float length; /**<Segment length*/
		float height; /**<height of the segment (obstacle)*/
		bool inOutDoor; /**<says whatever the line is indoor or outdoor =0 indoor; =1 outdoor*/
		char description[40];/**<Semantic information about line segment*/
		
		void set_vv(); 

		//float denom, t1, t2;
		//float ax,ay,bx,by,sax,say,sbx,sby;
		//float iPx, iPy;


	public:
		Cline();
		Cline(Cpoint *paa, Cpoint *pbb);/**<Segment Constructor with aa and bb coordinates*/
		Cline(const Cpoint &paa, const Cpoint &pbb);/**<Segment Constructor with aa and bb coordinates*/
		Cline(Cpoint *paa, Cpoint *pbb, float hght, bool inOut);/**<Segment Constructor with aa,bb,height and inOutDoor settings*/
		Cline(const Cpoint &paa, const Cpoint &pbb, const float &hght, const bool & inOut);
		~Cline(); /**<destructor*/
		int set_line(Cpoint *paa, Cpoint *pbb); /**<sets ss and bb points*/
		int set_line(const Cpoint &paa, const Cpoint &pbb); /**<sets ss and bb points*/
		int set_aa(Cpoint *paa);/**<sets aa point*/
		int set_bb(Cpoint *pbb);/**<sets bb point*/
		void set_point_coord(float ax, float ay, float bx, float by);  /**<sets coordinates of aa and bb points*/
		void setDescription(char *descript, int nn); /**<sets semantic descritpion*/
		int get_aa(Cpoint *paa);/**<gets aa point to paa*/
		int get_bb(Cpoint *pbb);/**<gets bb point to pbb*/
		Cpoint get_aa();/**<gets aa point to paa*/
		Cpoint get_bb();/**<gets bb point to pbb*/
		Cpoint get_vv() const;/**<gets vv vector to pbb*/
		Cpoint getDirection() const;/**<gets vv vector normalized*/
		int get_point_coord(float *ax, float *ay, float *bx, float *by);  /**<gets coordinates of aa and bb points*/
		Cpoint* getAA(); /**returns a pointer to A point*/
		Cpoint* getBB(); /**returns a pointer to B point*/
		Cpoint* getVV(); /**returns a pointer to V vector*/
		float getRads(); // returns the angle of the line in radians.
		float get_length();/**<gets length*/
		float get_height();/**<gets height*/
		bool get_inOutDoor();/**<gets inOutDoor*/
		void printLine() const;/**<prints line parameters to standar output*/
		float d2point(Cpoint *qq); /**<Distance from this segment to point qq.*/
		float d2point(Cpoint *qq, Cpoint *iPoint); /**<Distance from this segment to point qq. Sets to *iPoint the closest point of the segment to qq*/
		int d2point(const Cpoint &qq, Cpoint &iPoint, float & dist) const;
		float d2point2(Cpoint *qq, Cpoint *iPoint); /**<Distance square from this segment to point qq. Sets to *iPoint the closest point of the segment to qq*/
		bool segmentInterference(Cline *sgmnt); /**<Evaluates if "this" intersects with a given segment (1) or not (0)*/
		bool iPsegmentInterference(Cline *sgmnt, Cpoint *iPoint);/**<if "this" and "sgmnt" intersects sets *iPoint as the intersection point and returns 1. Otherwise returns 0*/
		float segmentInterferenceRange(Cline *sgmnt, float rMax);/**<if "this" and "sgmnt" intersects returns the distance from sgmnt->aa to the intersection point. Otherwise returns rMax*/
		Cpoint fraction(float factor); /** Computes a point over the line that are the factor of the line, for example, if the factor is 1/3, then it returns a point on the line that are at 1/3 distance of a to b*/
		Cpoint chunk(float d); /** Computes a point over the line that are at distance d of a to b*/
		bool chunk(float d, Cpoint &p); /** Same with boolean return to signal whether chunking was necessary */
		bool operator==(Cline *xline); /**< == Operator is overloaded using operator== of Cpoint. Returns 1 if lines are equal*/
		float area(Cline &xline); /**Calculates the area between two lines*/
		void adjust(); /**Verify that the point set as AA, is the most left and below between points. If not, changes the points.*/
		void swap(); /**Swap the values of aa and bb*/
		
		unsigned short int intersectionsWithCircle(Cpoint c, double r, vector<Cpoint> &intersections);/**< Returns number of intersection points between the segment and the circle defined by (c,r) */
};

#endif
