#ifndef point3d_H
#define point3d_H

#include <iostream>
#include <math.h>

using namespace std;

/**
 * \brief Points in 3D space
 * 
 *  Cpoint3d class implements data structure and functions to hold and manipulate points in a 3D space
 * 
 *  xx,yy,zz are the coordinates of the point in some reference frame
 * 
*/
class Cpoint3d
{
	protected:
		/**
		 * \brief X coordinate
		 * 
		 * X coordinate with respect to some reference frame. 
		 * 
		*/
		double xx; 
		
		/**
		 * \brief Y coordinate
		 * 
		 * Y coordinate with respect to some reference frame. 
		 * 
		*/
		double yy; 
		
		/**
		 * \brief Z coordinate
		 * 
		 * Z coordinate with respect to some reference frame. 
		 * 
		*/
		double zz; 
		
		
	public:
		/**
		 * \brief Constructor without coordinates
		 * 
		 * Constructor without coordinates. Point is initialized to (0,0,0)
		 * 
		*/		
		Cpoint3d();
		
		/**
		 * \brief Constructor with initial coordinates
		 * 
		*/				
		Cpoint3d(const double cx, const double cy, const double cz);

		/**
		 * \brief Destructor
		 * 
		*/				
		virtual ~Cpoint3d();

		/**
		 * \brief Set x coordinate
		*/						
		void setX(const double cx); 
		
		/**
		 * \brief Set y coordinate
		*/								
		void setY(const double cy); 
		
		/**
		 * \brief Set z coordinate
		*/								
		void setZ(const double cz); 
		
		/**
		 * \brief Set x,y,z coordinates
		*/						
		void setXYZ(const double cx, const double cy, const double cz);
		
		/**
		 * \brief Sets from point pointer
		 *
		 * Sets "this" coordinates from qq coordinates
		 *
		*/								
		void setPoint(const Cpoint3d *qq);

		/**
		 * \brief Sets from point reference
		 *
		 * Sets "this" coordinates from qq coordinates
		 *
		*/								
		void setPoint(const Cpoint3d & qq);
		
		/**
		 * \brief Increment point coordinates
		 *
		 * Increments point coordinates by the given deltas dx,dy,dz
		 *
		*/										
		void incXYZ(const double & dx, const double & dy, const double & dz);

		/**
		 * \brief Get x coordinate
		*/										
		double getX() const; 

		/**
		 * \brief Get y coordinate
		*/										
		double getY() const; 
		
		/**
		 * \brief Get z coordinate
		*/												
		double getZ() const;

		/**
		 * \brief Norm
		 * 
		 * Returns the norm of the point (euclidean distance to the origin)
		 *
		*/												
		double norm() const;
		
		/**
		 * \brief Norm^2
		 * 
		 * Returns the squared value of the norm of the point (squared euclidean distance to the origin)
		 *
		*/														
		double norm2() const;
		
		/**
		 * \brief Distance to point
		 * 
		 * Returns the euclidean distance from "this" to qq
		 *
		*/																
		double d2point(const Cpoint3d *qq) const;
		
		/**
		 * \brief Squared distance to point
		 * 
		 * Returns the squared euclidean distance from "this" to qq
		 *
		*/																		
		double d2point2(const Cpoint3d *qq) const;

		/**
		 * \brief Distance to point
		 * 
		 * Returns the euclidean distance from "this" to qq
		 *
		*/																
		double d2point(const Cpoint3d & qq) const;

		/**
		 * \brief Squared distance to point
		 * 
		 * Returns the squared euclidean distance from "this" to qq
		 *
		*/																				
		double d2point2(const Cpoint3d & qq) const;
        
        /** \brief Returns azimuth of the point
         * 
         * Returns azimuth of the point
         * 
         **/
        double getAzimuth() const; 

		/**
		 * \brief Assignement operator
		 * 
		 * Sets x,y,z coordinates form qq coordinates
		 *
		*/																						
		void operator=(const Cpoint3d & qq);

		/**
		 * \brief Prints point coordinates to stdout
		 * 
		 * Prints point coordinates to stdout in a single row format "(xx,yy,zz)"
		 *
		*/																						
		void printPoint() const;
};
#endif
