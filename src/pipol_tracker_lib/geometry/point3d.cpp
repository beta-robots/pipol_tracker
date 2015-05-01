#include "point3d.h"

Cpoint3d::Cpoint3d()
{
	xx=0; yy=0; zz=0; 
}

Cpoint3d::Cpoint3d(const double cx, const double cy, const double cz)
{
	xx=cx; yy=cy; zz=cz;
}

Cpoint3d::~Cpoint3d()
{
}

void Cpoint3d::setX(const double cx){xx=cx;} 

void Cpoint3d::setY(const double cy){yy=cy;} 

void Cpoint3d::setZ(const double cz){zz=cz;}  

void Cpoint3d::setXYZ(const double cx, const double cy, const double cz){xx=cx;yy=cy;zz=cz;}

void Cpoint3d::setPoint(const Cpoint3d *qq)
{
	xx=qq->getX(); yy=qq->getY(); zz=qq->getZ();
}

void Cpoint3d::setPoint(const Cpoint3d &qq)
{
	xx=qq.getX(); yy=qq.getY(); zz=qq.getZ();
}

double Cpoint3d::getX() const {return xx;}

double Cpoint3d::getY() const {return yy;} 

double Cpoint3d::getZ() const {return zz;} 

double Cpoint3d::norm() const
{
	return sqrt(xx*xx+yy*yy+zz*zz);
}

double Cpoint3d::norm2() const
{
	return (xx*xx+yy*yy+zz*zz);
}

void Cpoint3d::incXYZ(const double & dx, const double & dy, const double & dz)
{
	xx += dx;
	yy += dy;
	zz += dz;
}

double Cpoint3d::d2point(const Cpoint3d *qq) const
{
	return sqrt(d2point2(qq));
}

double Cpoint3d::d2point2(const Cpoint3d *qq) const
{
	double dx, dy, dz;
	dx=xx-qq->getX();
	dy=yy-qq->getY();
	dz=zz-qq->getZ();
	return dx*dx+dy*dy+dz*dz;
}

double Cpoint3d::d2point(const Cpoint3d & qq) const
{
  return sqrt(d2point2(qq));
}

double Cpoint3d::d2point2(const Cpoint3d & qq) const
{
	double dx, dy, dz;
	dx=xx-qq.getX();
	dy=yy-qq.getY();
	dz=zz-qq.getZ();
	return dx*dx+dy*dy+dz*dz;
}

double Cpoint3d::getAzimuth() const
{
    return atan2(yy, xx);
}

void Cpoint3d::operator=(const Cpoint3d & qq)
{
	xx = qq.getX();
	yy = qq.getY();
	zz = qq.getZ();
}

void Cpoint3d::printPoint() const
{
	cout << "(" << xx << "," << yy << "," << zz << ")" << endl;
}
