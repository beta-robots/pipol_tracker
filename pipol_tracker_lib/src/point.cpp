// point.cpp
#include "point.h"

using namespace std;
// Default constructor. Zeros to x and y coordinates.
Cpoint::Cpoint():xx(0), yy(0) { }
// Constructor with xx and yy coordinates.
Cpoint::Cpoint(float xcoord, float ycoord): xx(xcoord), yy(ycoord) { }
// Cpoint destructor
Cpoint::~ Cpoint(){ }

// Sets xx and yy coordinate.
void Cpoint::set_point(float xcoord, float ycoord)
{
	xx=xcoord;
	yy=ycoord;
}

void Cpoint::set_point(Cpoint & val)
{
	xx = val.get_xx();
	yy = val.get_yy();
}

void Cpoint::set_point(Cpoint *val)
{
	xx = val->get_xx();
	yy = val->get_yy();
}

// Set point values using polar coordinate values radio and theta
void Cpoint::setPolar(float r, float t)
{
	xx = r * cos(t);
	yy = r * sin(t);
}

// Sets xx coordinate.
void Cpoint::set_xx(float xcoord)
{
	xx=xcoord;
}

/**
  Sets yy coordinate.
*/
void Cpoint::set_yy(float ycoord)
{
	yy=ycoord;
}

/**
  Returns xx coordinate.
*/
float Cpoint::get_xx() const
{
	return xx;
}

/**
  Returns yy coordinate.
*/
float Cpoint::get_yy() const
{
	return yy;
}

/**
  Return coordinate by index
	Unsafe (tbd: throw exception if index is out of bounds)
*/
float Cpoint::getByIndex(unsigned short int index) const
{
	if(index==0)
		return xx;
	else
		return yy;
}

/**
  Set coordinate by index
	Unsafe (tbd: throw exception if index is out of bounds)
*/
void Cpoint::setByIndex(float value, unsigned short int index)
{
	if(index==0)
		xx = value;
	else
		yy = value;
}

/**
  Prints point to the standard output.
*/
void Cpoint::printPoint() const
{
	std::cout << "(" << xx << ", " << yy << ")";
}

/**
  Scalar product of this point (vector) with qq (vector)
*/
float Cpoint::scalar(Cpoint *qq) const
{
	return(xx*qq->get_xx()+yy*qq->get_yy());
}

/**
  Scalar product of this point (vector) with qq (vector)
*/
float Cpoint::scalar(const Cpoint & qq) const
{
	return(xx*qq.get_xx() + yy*qq.get_yy());
}

/**
  Distance from this point to qq point
*/
float Cpoint::d2point(Cpoint *qq)
{
	return (sqrt(pow((qq->get_xx()-xx),2)+pow((qq->get_yy()-yy),2)));	
}

float Cpoint::d2point(const Cpoint & qq) const
{
	return sqrt(pow((qq.get_xx()-xx),2) + pow((qq.get_yy()-yy),2));
}

/**
  Distance^2 from this point to qq point
*/
float Cpoint::d2point2(Cpoint *p)
{
	return (p->get_xx()-xx)*(p->get_xx()-xx)+(p->get_yy()-yy)*(p->get_yy()-yy);
}

float Cpoint::d2point2(Cpoint p)
{
	return d2point2(&p);
}

float Cpoint::distance(const Cpoint & val) const
{
	return sqrt(pow((val.get_xx()-xx),2)+pow((val.get_yy()-yy),2));
}
float Cpoint::distance(float val_x, float val_y)
{
	return sqrt(pow((val_x-xx),2)+pow((val_y-yy),2));
}

float Cpoint::getRad()
{
	return distance(0,0);
}

float Cpoint::getTheta()
{
	float add = 0, arctanxy;

	if(yy < 0) add = M_PI;

	if(!xx)	return M_PI/2 - add;

	arctanxy = atan (yy/xx);

	if(xx > 0) return arctanxy;
	if(xx < 0) return arctanxy + M_PI - 2*add;
  return 0;
}

void Cpoint::getPolar(float *r, float *t)
{
	*r = getRad();
	*t = getTheta();
}


/**
  Overloading of == operator to compare points with a tolerancy radi. 
  Returns 1 if distance between points is less than TOL_RADI or 0 if distance is greater than TOL_RADI.
*/
bool Cpoint::inRadius(Cpoint *bpoint)
{
	return (d2point(bpoint) < TOL_RADI);
}

// Overloading of = operator to assign coordinates of this point from bpoint.
void Cpoint::operator=(Cpoint *bpoint)
{
	xx=bpoint->get_xx();
	yy=bpoint->get_yy();
}

// Overloading of - operator
Cpoint Cpoint::operator-(const Cpoint &bpoint)
{
	Cpoint ret;
	ret.set_point(xx - bpoint.get_yy(), yy - bpoint.get_yy());
	// TODO: OST 05-05-09: ¿Aquí hay un error? ¿No debería ser xx - bpoint.get_xx()?
	return ret;
}

// Overloading of + operator
Cpoint Cpoint::operator+(const Cpoint &bpoint)
{
	Cpoint ret;
	ret.set_point(xx + bpoint.get_xx(), yy + bpoint.get_yy());
	return ret;
}

// Overloading of = operator to assign coordinates of this point from bpoint.
void Cpoint::operator=(const Cpoint &bpoint)
{
	xx=bpoint.get_xx();
	yy=bpoint.get_yy();
}


bool Cpoint::operator==(Cpoint pose)
{
	if( (xx == pose.get_xx()) && (yy == pose.get_yy()) ){return 1;}
	else {return 0;}
}

bool Cpoint::operator!=(Cpoint pose)
{
	if( (xx == pose.get_xx()) && (yy == pose.get_yy()) ){return 0;}
	else {return 1;}
}

void Cpoint::display() { cout << "x = " << xx << "\ty = " << yy << endl; }
void Cpoint::display(string str) { cout << str; display(); }

