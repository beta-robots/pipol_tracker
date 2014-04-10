#include "point3dCov.h"

Cpoint3dCov::Cpoint3dCov():Cpoint3d()
{
	covMat = Matrix3f::Zero();
}

Cpoint3dCov::Cpoint3dCov(const double cx, const double cy, const double cz, const Matrix3f &covM):Cpoint3d(cx,cy,cz)
{
	covMat = covM;
}

Cpoint3dCov::~Cpoint3dCov()
{}

void Cpoint3dCov::setMatrix(const Matrix3f &covM)
{
	this->covMat = covM;
}

void Cpoint3dCov::setDiagonal(const double sx2, const double sy2, const double sz2)
{
	covMat = Matrix3f::Zero();
	covMat(0,0)=sx2;
	covMat(1,1)=sy2;
	covMat(2,2)=sz2;
}

void Cpoint3dCov::setXYcov(const double sx2, const double sy2, const double sxy)
{
	covMat = Matrix3f::Zero();
	covMat(0,0)=sx2;
	covMat(1,1)=sy2;
	covMat(0,1)=sxy;	
	covMat(1,0)=sxy;	
}

void Cpoint3dCov::getMatrix(Matrix3f &mat) const
{
	mat = this->covMat;
}

double Cpoint3dCov::getMatrixElement(const unsigned int ii, const unsigned int jj) const
{
	return covMat(ii,jj);
}

double Cpoint3dCov::getCovTrace() const
{
	return covMat(0,0)+covMat(1,1)+covMat(2,2);
}

double Cpoint3dCov::mahalanobisDistance(const Cpoint3d &qq) const
{
	double dm2;
	Vector3f vDiff, vAux; 
		
	vDiff << ( this->xx - qq.getX() ), ( this->yy - qq.getY() ), ( this->zz - qq.getZ() );//difference vector
	vAux = covMat.inverse()*vDiff;
	dm2 = vDiff(0)*vAux(0) + vDiff(1)*vAux(1) + vDiff(2)*vAux(2);	
	return sqrt(dm2);
}

double Cpoint3dCov::mahalanobisDistance2D(const Cpoint3d &qq) const
{
	double dm2;
	Vector2f vDiff, vAux; 
		
	vDiff << ( this->xx - qq.getX() ), ( this->yy - qq.getY() );//difference vector
	vAux = (covMat.block<2,2>(0,0).inverse())*vDiff;
	dm2 = vDiff(0)*vAux(0) + vDiff(1)*vAux(1);	
	return sqrt(dm2);
}

void Cpoint3dCov::operator=(const Cpoint3dCov & qq)
{
	//Matrix3f auxM; 
	this->xx = qq.getX();
	this->yy = qq.getY();
	this->zz = qq.getZ();
	qq.getMatrix(this->covMat);
	
}

void Cpoint3dCov::printPointCov()
{
	this->printPoint();
	cout << covMat << endl;
}
