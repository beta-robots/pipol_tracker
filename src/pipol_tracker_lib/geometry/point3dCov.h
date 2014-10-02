#ifndef point3dCov_H
#define point3dCov_H

#include "point3d.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;
using namespace std;

/**
 * \brief Points with covariance in 3D space
 * 
 *  Cpoint3d inherits 3d points and adds a covariance matrix to them
 * 
 * 
*/
class Cpoint3dCov : public Cpoint3d
{
      protected:
      
            /** \brief Covariance matrix
             * 
             * Covarianc matrix
             * 
             **/
            Matrix3f covMat;
            
      public:

            /** \brief Constructor without arguments
             * 
             * Constructor without arguments
             * 
             **/
            Cpoint3dCov();
            
            /** \brief Constructor with arguments
             * 
             * Constructor with arguments
             * 
             **/
            Cpoint3dCov(const double cx, const double cy, const double cz, const Matrix3f &covM);

            /** \brief Default destructor
             * 
             * Default destructor
             * 
             **/            
            ~Cpoint3dCov();
            
		/**
		 * \brief Sets the full covariance matrix.
		 * 
		 * Sets the full covariance matrix.
		 * 
		*/		
		void setMatrix(const Matrix3f &covM);
				
		/**
		 * \brief Sets the diagonal of the covariance matrix.
		 * 
		 * Sets the diagonal of the covariance matrix. Other elements are reset to 0.
		 * 
		*/		
		void setDiagonal(const double sx2, const double sy2, const double sz2);
		
		/**
		 * \brief Sets the XY elements of the covariance matrix.
		 * 
		 * Sets the XY elements of the covariance matrix (s^2_x, s^2_y, s_xy).
		 * Other elements are reset to 0.
		 * 
		*/		
		void setXYcov(const double sx2, const double sy2, const double sxy);

		/**
		 * \brief Gets the full covariance matrix.
		 * 
		 * Gets the full covariance matrix.
		 * 
		*/		
		void getMatrix(Matrix3f &mat) const;
		
		/**
		 * \brief Gets an element of the covariance matrix.
		 * 
		 * Gets the i-th row, j-th column, element of the covariance matrix.
		 * 
		*/		
		double getMatrixElement(const unsigned int ii, const unsigned int jj) const;
		
		/**
		 * \brief Gets the trace of the covariance matrix.
		 * 
		 * Gets the trace of the covariance matrix.
		 * 
		*/		
		double getCovTrace() const;		

		/**
		 * \brief Computes the mahalanobis distance
		 * 
		 * Computes the mahalanobis distance between this point and the deterministic point qq
		 * 
		*/				
		double mahalanobisDistance(const Cpoint3d &qq) const;
		
		/**
		 * \brief Computes the mahalanobis distance through XY dimensions
		 * 
		 * Computes the mahalanobis distance between this point and the deterministic point qq.
		 * It takes into account only x-y components (z is ignored)
		 * 
		*/				
		double mahalanobisDistance2D(const Cpoint3d &qq) const;
				
		/**
		 * \brief Assignement operator
		 * 
		 * Sets x,y,z coordinates form qq coordinates
		 *
		*/																						
		void operator=(const Cpoint3dCov & qq);		
		
		/**
		 * \brief Prints point coordinates and covariance matrix
		 * 
		 * Prints point coordinates and covariance matrix
		 * 
		*/						
		void printPointCov();
		
};
#endif
