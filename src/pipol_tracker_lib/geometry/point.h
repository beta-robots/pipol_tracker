/**
*************************************************************************
*                                                                     	*
* FILE NAME:	point.h                                         	*
* DATE:		Jan 2007                                        	*
* VERSION:	1.0                                             	*
* PURPOSE:	Specifies point object			          	*
* CONTRIBUTORS:	Andreu Corominas Murtra (acorominas@iri.upc.edu)	*
* AFFILIATIONS:	Institut de Rob�tica Industrial (CSIC-UPC)		*
*									*
************************************************************************/

#ifndef POINT_H
#define POINT_H

#include <iostream>
#include <math.h>

#define TOL_RADI 0.1 

using namespace std;

// Cpoint class has two coordinates in the 2D space: xx and yy. 
class Cpoint
{
	protected:
		float xx; /**<x coordinate*/
		float yy; /**<y coordinate*/
	public:
		Cpoint();
		Cpoint(float xcoord, float ycoord);
		~Cpoint();
		void set_point(float xcoord, float ycoord);
		void setPolar(float r, float t); // Set point values using polar coordinate values radio and theta.
		void set_point(Cpoint *val);
		void set_point(Cpoint& val);
		void set_xx(float xcoord);
		void set_yy(float ycoord);
		float get_xx() const;
		float get_yy() const;
		float getByIndex(unsigned short int index) const; /**<0 for xx, 1 for yy; unsafe-ish, yet convenient*/
		void setByIndex(float value, unsigned short int index); /**<same as above*/
		void printPoint() const; /**<Prints the point to std output*/
		float scalar(Cpoint*) const; /**<Scalar product of this point (vector) with qq (vector)*/
		float scalar(const Cpoint &) const; /**<Scalar product of this point (vector) with qq (vector)*/
		float d2point(Cpoint*); /**<Distance from this point to point qq*/
		float d2point(const Cpoint & qq) const;
		float d2point2(Cpoint p); /**<Distance^2 from this point to point qq*/
		float d2point2(Cpoint *p); /**<Distance^2 from this point to point qq*/
		float distance(const Cpoint&) const;
		float distance(float val_x, float val_y);

		float getRad(); // Gets the distance from (0,0) to _this_
		float getTheta(); // Gets the theta value for polar coordinates.
		void getPolar(float *r, float *t); // Get the polar coordinates in r and t.
		
		bool inRadius(Cpoint *bpoint); /**<Operator == is overloaded with a tolerancy radi TOL_RADI. Returns 1 if points are equal*/
		void operator=(Cpoint *bpoint); /**<Operator = is overloaded. Assignation*/
		void operator=(const Cpoint &bpoint); /**<Operator = is overloaded. Assignation*/
		bool operator==(Cpoint pose);
		bool operator!=(Cpoint pose);
		Cpoint operator-(const Cpoint &bpoint);
		Cpoint operator+(const Cpoint &bpoint);

friend ostream& operator << (ostream& os, const Cpoint& point)
{return os << "(" << point.get_xx() << ", " << point.get_yy() << ")";}


	void display();
	void display(string str);
};

#endif


