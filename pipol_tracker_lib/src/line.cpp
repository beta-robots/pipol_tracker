/**
***********************************************************************
*                                                                     *
* FILE NAME:          robot.cpp                                       *
* DATE:               Feb 2006                                        *
* VERSION:            1.0                                             *
* PURPOSE:            Implement class Cline                           *
* ORIGINAL CODE :     Andreu Corominas Murtra (acorominas@iri.upc.edu)*
*                     Institut de Rob�tica Industrial                 *
*                     CSIC - UPC                                      *
* CONTRIBUTOR(S):                                                     *
*                                                                     *
***********************************************************************
*/

//include
#include "line.h"

//define

// member functions of Cline Class
/**
  Default constructor. All points to zero. 
*/
Cline::Cline() //default constructor
{
	aa.set_point(0,0);
	bb.set_point(0,0);
	vv.set_point(0,0);
	length=0;
	height=0;
	inOutDoor=0;
	description[0]='\0';
}

/**
  Constructor of a segment going from paa to pbb.
*/
Cline::Cline(Cpoint *paa, Cpoint *pbb)
{
	aa.set_point(paa->get_xx(),paa->get_yy());
	bb.set_point(pbb->get_xx(),pbb->get_yy());
	length=aa.d2point(&bb);
	set_vv();
	height=0;
	inOutDoor=0;
	description[0]='\0';
}

/**
  Segment Constructor with aa and bb coordinates
*/
Cline::Cline(const Cpoint &paa, const Cpoint &pbb)
{
	aa.set_point(paa.get_xx(),paa.get_yy());
	bb.set_point(pbb.get_xx(),pbb.get_yy());
	length=aa.d2point(&bb);
	set_vv();
	height=0;
	inOutDoor=0;
	description[0]='\0';
}

/**
  Constructor of a segment going from paa to pbb with height hght and indoor or outdoor especification.
*/
Cline::Cline(Cpoint *paa, Cpoint *pbb, float hght, bool inOut)
{
	aa.set_point(paa->get_xx(),paa->get_yy());
	bb.set_point(pbb->get_xx(),pbb->get_yy());
	length=aa.d2point(&bb);
	set_vv();
	height=hght;
	inOutDoor=inOut;
	description[0]='\0';
}

Cline::Cline(const Cpoint &paa, const Cpoint &pbb, const float &hght, const bool & inOut)
{
	aa.set_point(paa.get_xx(),paa.get_yy());
	bb.set_point(pbb.get_xx(),pbb.get_yy());
	length=aa.d2point(&bb);
	set_vv();
	height=hght;
	inOutDoor=inOut;
	description[0]='\0';
}

/**
  Destructor
*/
Cline::~Cline()
{
//	cout << "\t\tLINE destructor" << endl;
}

void Cline::set_vv()
{
	vv.set_xx(bb.get_xx()-aa.get_xx());
	vv.set_yy(bb.get_yy()-aa.get_yy());
}

/**
  Sets aa and bb points of the segment.
*/
int Cline::set_line(Cpoint *paa, Cpoint *pbb)
{
	aa.set_point(paa->get_xx(),paa->get_yy());
	bb.set_point(pbb->get_xx(),pbb->get_yy());
	length=aa.d2point(&bb);
	set_vv();
	return 0;
}

/**
  Sets aa and bb points of the segment.
*/
int Cline::set_line(const Cpoint &paa, const Cpoint &pbb)
{
	aa.set_point(paa.get_xx(),paa.get_yy());
	bb.set_point(pbb.get_xx(),pbb.get_yy());
	length=aa.d2point(&bb);
	set_vv();
	return 0;
}

/**
  Sets aa point. (x and y coordinates)
*/
int Cline::set_aa(Cpoint *paa)
{
	aa.set_point(paa->get_xx(),paa->get_yy());
	length=aa.d2point(&bb);
	set_vv();
	return 0;
}

/**
  Sets bb point. (x and y coordinates)
*/
int Cline::set_bb(Cpoint *pbb)
{
	bb.set_point(pbb->get_xx(),pbb->get_yy());
	length=aa.d2point(&bb);
	set_vv();
	return 0;
}

/**
  sets coordinates of aa and bb points
*/
void Cline::set_point_coord(float ax, float ay, float bx, float by)
{
	aa.set_point(ax,ay);
	bb.set_point(bx,by);
	length=aa.d2point(&bb);
	set_vv();
}


void Cline::setDescription(char *descript, int nn)
{
	strncpy(description,descript,nn);
	//std::cout << description << std::endl;
}

/**
  Gets aa point. Loads it to paa adress.
*/
int Cline::get_aa(Cpoint *paa)
{
	paa->set_point(aa.get_xx(),aa.get_yy());
	return 0;
}

/**
  Gets bb point. Loads it to pbb adress.
*/
int Cline::get_bb(Cpoint *pbb)
{
	pbb->set_point(bb.get_xx(),bb.get_yy());
	return 0;
}

/**
  Puts coordinates of aa and bb points to ax, ay, bx and by.
*/
int Cline::get_point_coord(float *ax, float *ay, float *bx, float *by)
{
	*ax=aa.get_xx();
	*ay=aa.get_yy();
	*bx=bb.get_xx();
	*by=bb.get_yy();
	return 0;
}

Cpoint *Cline::getAA()
{
	return &aa;
}

Cpoint *Cline::getBB()
{
	return &bb;
}

Cpoint Cline::get_aa()
{
	return aa;
}

Cpoint Cline::get_bb()
{
	return bb;
}

/**
  Gets vv vector to pbb
*/
Cpoint Cline::get_vv() const
{
	return vv;
}

/**
  Gets vv vector normalized
*/
Cpoint Cline::getDirection() const
{
	return Cpoint(vv.get_xx()/length, vv.get_yy()/length);
}

Cpoint *Cline::getVV()
{
	return &vv;
}

float Cline::getRads()
{
  return atan2(bb.get_yy()-aa.get_yy(),bb.get_xx()-aa.get_xx());
}

/**
  Returns length of the segment.
*/
float Cline::get_length()
{
	return length;
}

/**
  Returns heigth of the segment.
*/
float Cline::get_height()
{
	return height;
}

/**
  Returns inOutDoor especification.
*/
bool Cline::get_inOutDoor()
{
	return inOutDoor;
}

/**
  Prints Segment to the standard output.
*/
void Cline::printLine() const
{
	cout << "AA" << aa << "==>BB" << bb << "\t vv=" << vv;
	//std::cout << " [height=" << height << "; inOutDoor=" << inOutDoor << "; " << description << "]" << std::endl;
}

/**
  Returns the distance from this segment to point qq.
*/
float Cline::d2point(Cpoint *qq)
{
	float t1, iPx, iPy;
	Cpoint iPoint;
	
	Cpoint *ww = new Cpoint((qq->get_xx()-aa.get_xx()),(qq->get_yy()-aa.get_yy())); //ww is the two coordinate vector going from aa to qq
	if (vv.scalar(ww)<=0)
	{
		delete ww;
		iPoint = &aa; //aa is the closest point to qq
		return (aa.d2point(qq));
	}
	
	ww->set_point((qq->get_xx()-bb.get_xx()),(qq->get_yy()-bb.get_yy())); //ww is the two coordinate vector going from bb to qq
	
	if (vv.scalar(ww)>=0)
	{
		delete ww;
		iPoint = &bb;//bb is the closest point to qq
		return (bb.d2point(qq));
	}
	
	delete ww;
	
	t1=(qq->get_xx()-aa.get_xx())*(bb.get_xx()-aa.get_xx())+(qq->get_yy()-aa.get_yy())*(bb.get_yy()-aa.get_yy());
	t1=t1/(length*length);
	
	//sets iP as normal projection 
	iPx=aa.get_xx()+t1*vv.get_xx(); 
	iPy=aa.get_yy()+t1*vv.get_yy(); 
	iPoint.set_point(iPx,iPy);

	return (iPoint.d2point(qq));
}

/**
  Returns the distance from this segment to point qq. Evaluate first if distance is from one of the limiting points (aa or bb). If not, evaluate distance from the segment. Sets to *iPoint the closest point of the segment to qq.
*/
float Cline::d2point(Cpoint *qq, Cpoint *iPoint)
{
	float t1, iPx, iPy;
	
	Cpoint *ww = new Cpoint((qq->get_xx()-aa.get_xx()),(qq->get_yy()-aa.get_yy())); //ww is the two coordinate vector going from aa to qq
	if (vv.scalar(ww)<=0)
	{
		delete ww;
		*iPoint=&aa; //aa is the closest point to qq
		return (aa.d2point(qq));
	}
	
	ww->set_point((qq->get_xx()-bb.get_xx()),(qq->get_yy()-bb.get_yy())); //ww is the two coordinate vector going from bb to qq
	
	if (vv.scalar(ww)>=0)
	{
		delete ww;
		*iPoint=&bb;//bb is the closest point to qq
		return (bb.d2point(qq));
	}
	
	delete ww;
	
	t1=(qq->get_xx()-aa.get_xx())*(bb.get_xx()-aa.get_xx())+(qq->get_yy()-aa.get_yy())*(bb.get_yy()-aa.get_yy());
	t1=t1/(length*length);
	
	//sets iP as normal projection 
	iPx=aa.get_xx()+t1*vv.get_xx(); 
	iPy=aa.get_yy()+t1*vv.get_yy(); 
	iPoint->set_point(iPx,iPy);

	return (iPoint->d2point(qq));
}

/**
  Returns the distance from this segment to point qq. Evaluate first if distance is from one of the limiting points (aa or bb). If not, evaluate distance from the segment. Sets to *iPoint the closest point of the segment to qq.
*/
int Cline::d2point(const Cpoint & qq, Cpoint & iPoint, float & dist) const
{
	//float t1, iPx, iPy;
	float dotProd;
	
//cout << endl << endl << "************* d2point *************" << endl;
//cout << "qq=" << qq << endl;

	//ww is the two coordinate vector going from bb to qq
	Cpoint ww((qq.get_xx()-bb.get_xx()),(qq.get_yy()-bb.get_yy()));

	//check if BB is the closest point
	dotProd = vv.scalar(ww);
	if (dotProd >= 0)
	{
		iPoint=bb;//bb is the closest point to qq
		dist = bb.d2point(qq);
		return iBB;
	}
	
	//ww is the two coordinate vector going from aa to qq
	ww.set_point((qq.get_xx()-aa.get_xx()),(qq.get_yy()-aa.get_yy()));

	//check if AA is the closest point
	dotProd = vv.scalar(ww);
	if (dotProd <= 0)
	{
		iPoint = aa; //aa is the closest point to qq
		dist = aa.d2point(qq);
		return iAA;
	}

	//point between AA and BB
	//compute projection of ww onto vv
	dotProd = dotProd/(length*length);
	iPoint.set_point(aa.get_xx()+dotProd*vv.get_xx(), aa.get_yy()+dotProd*vv.get_yy());

/*
cout << "iPoint=" << iPoint << endl;

Cline auxLine1(aa, iPoint);
Cline auxLine2(iPoint, bb);
cout << "SEGMENT!" << endl;
printLine();
cout << "\tdir1=" << getDirection() << endl;
cout << "auxLine1(aa, iPoint)=" << endl;
auxLine1.printLine();
cout << "\tdir1=" << auxLine1.getDirection() << endl;
cout << "auxLine2(iPoint, bb)=" << endl;
auxLine2.printLine();
cout << "\tdir2=" << auxLine2.getDirection() << endl;
*/
	dist = iPoint.d2point(qq);
	return iAB;
}


/**
  Returns the distance squared from this segment to point qq. Evaluate first if distance is from one of the limiting points (aa or bb). If not, evaluate distance from the segment. Sets to *iPoint the closest point of the segment to qq.
*/
float Cline::d2point2(Cpoint *qq, Cpoint *iPoint)
{
	float t1, iPx, iPy;
	
	Cpoint *ww = new Cpoint((qq->get_xx()-aa.get_xx()),(qq->get_yy()-aa.get_yy())); //ww is the two coordinate vector going from aa to qq
	if (vv.scalar(ww)<=0)
	{
		delete ww;
		*iPoint=&aa; //aa is the closest point to qq
		return (aa.d2point2(qq));
	}
	
	ww->set_point((qq->get_xx()-bb.get_xx()),(qq->get_yy()-bb.get_yy())); //ww is the two coordinate vector going from bb to qq
	
	if (vv.scalar(ww)>=0)
	{
		delete ww;
		*iPoint=&bb; //aa is the closest point to qq
		return (bb.d2point2(qq));
	}
	
	delete ww;
		
	t1=(qq->get_xx()-aa.get_xx())*(bb.get_xx()-aa.get_xx())+(qq->get_yy()-aa.get_yy())*(bb.get_yy()-aa.get_yy());
	t1=t1/(length*length);
	
	//sets iP as normal projection 
	iPx=aa.get_xx()+t1*vv.get_xx(); 
	iPy=aa.get_yy()+t1*vv.get_yy(); 
	iPoint->set_point(iPx,iPy);

	return (iPoint->d2point2(qq));
}


/**
  Segment interference evaluates intersection between "this" and a given segment. Returns 1 if intersection and  0 if no intersection.
*/
bool Cline::segmentInterference(Cline *sgmnt)
{
	float det1,det2,det3,det4;
	float px, py, qx, qy;
	bool s1=0, s2=0, s3=0, s4=0;
	
	sgmnt->get_point_coord(&px, &py, &qx, &qy);
	det1= aa.get_xx()*bb.get_yy() + bb.get_xx()*py + px*aa.get_yy() - px*bb.get_yy() - aa.get_xx()*py - bb.get_xx()*aa.get_yy();
	det2= aa.get_xx()*bb.get_yy() + bb.get_xx()*qy + qx*aa.get_yy() - qx*bb.get_yy() - aa.get_xx()*qy - bb.get_xx()*aa.get_yy();	
	det3= px*qy + qx*aa.get_yy() + aa.get_xx()*py - aa.get_xx()*qy - px*aa.get_yy() - qx*py;
	det4= px*qy + qx*bb.get_yy() + bb.get_xx()*py - bb.get_xx()*qy - px*bb.get_yy() - qx*py;
	
	if (det1>=0) {s1=1;}
	if (det2>=0) {s2=1;}
	if (det3>=0) {s3=1;}
	if (det4>=0) {s4=1;}
	
	if (((s1^s2)&&(s3^s4))==1){return 1;}
	else {return 0;}
}

/**
  iPsegmentInterference returns 1 if "this" and "sgmnt" intersects and sets *iPoint. Otherwise returns 0
*/
bool Cline::iPsegmentInterference(Cline *sgmnt, Cpoint *iPoint)
{
	float denom, t1, t2;
	float ax,ay,bx,by,sax,say,sbx,sby;
	float iPx, iPy;
	
	ax=aa.get_xx();ay=aa.get_yy();bx=bb.get_xx();by=bb.get_yy();
	sgmnt->get_point_coord(&sax, &say, &sbx, &sby);
	
	denom=(sby-say)*(bx-ax)-(sbx-sax)*(by-ay);
	if (denom==0){return 0;}/**<segments are parallel*/
	
	t1=((sbx-sax)*(ay-say)-(sby-say)*(ax-sax))/denom;
	t2=((bx-ax)*(ay-say)-(by-ay)*(ax-sax))/denom;
	
	iPx=ax+t1*(bx-ax); /**<iPx of the LINE intersection*/
	iPy=ay+t1*(by-ay); /**<iPy of the LINE intersection*/
	
	if (((t1>=0)&&(t1<=1))&&((t2>=0)&&(t2<=1))) {iPoint->set_point(iPx,iPy); return 1;}
	else {return 0;}
}

/**
  if "this" and "sgmnt" intersects returns the distance from sgmnt->aa to the intersection point. Otherwise returns rMax
*/
float Cline::segmentInterferenceRange(Cline *sgmnt, float rMax)
{
	//float det1,det2,det3,det4;
	float ax, ay, bx, by, sax,say,sbx,sby, tl,ts, denom;
	//bool s1=0, s2=0, s3=0, s4=0;
	
	
	sgmnt->get_point_coord(&sax, &say, &sbx, &sby);
	/*det1= aa.get_xx()*bb.get_yy() + bb.get_xx()*say + sax*aa.get_yy() - sax*bb.get_yy() - aa.get_xx()*say - bb.get_xx()*aa.get_yy();
	det2= aa.get_xx()*bb.get_yy() + bb.get_xx()*sby + sbx*aa.get_yy() - sbx*bb.get_yy() - aa.get_xx()*sby - bb.get_xx()*aa.get_yy();	
	det3= sax*sby + sbx*aa.get_yy() + aa.get_xx()*say - aa.get_xx()*sby - sax*aa.get_yy() - sbx*say;
	det4= sax*sby + sbx*bb.get_yy() + bb.get_xx()*say - bb.get_xx()*sby - sax*bb.get_yy() - sbx*say;
	
	if (det1>=0) {s1=1;}
	if (det2>=0) {s2=1;}
	if (det3>=0) {s3=1;}
	if (det4>=0) {s4=1;}
	
	if (((s1^s2)&&(s3^s4))==0){return rMax;}//no intersection case: return rMax
	*/

	//Intersection case: compute intersection distance from the sgmnt->aa
	ax=aa.get_xx();ay=aa.get_yy();bx=bb.get_xx();by=bb.get_yy();
	denom = (bx-ax)*(say-sby)+(sbx-sax)*(by-ay);
	if (denom==0){return rMax;}/**<segments are parallel*/

	tl=((sax-ax)*(say-sby)-(sax-sbx)*(say-ay))/denom;
	if ((tl<0)||(tl>1)){return rMax;}
	ts=((bx-ax)*(say-ay)-(by-ay)*(sax-ax))/denom;
	if((ts<0)||(ts>1)){return rMax;}
	return ts*sgmnt->get_length();
}

/**
Computes a point over the line that are the factor of the line, for example, if the factor is 1/3, then it returns a point on the line that are at 1/3 distance of a
*/
Cpoint Cline::fraction(float factor)
{
	return chunk(length * factor);
}

/** Computes a point over the line that are at x distance of a to b*/
Cpoint Cline::chunk(float d)
{
	Cpoint increment;

	//adjust();
	if (d > length) return bb;
	
	increment.setPolar(d, getRads());

	return (aa + increment);
}

bool Cline::chunk(float d, Cpoint &p)
{
	//adjust();
	if (d > length)
	{
		p = bb;
		return false;
	}
	else
	{
		p.setPolar(d, getRads()); // increment
		p = p + aa;
		return true;
	}
}

/**
  == operator is overloaded using operator== of Cpoint
*/
bool Cline::operator==(Cline *xline)
{
	bool ret_val=1;
	Cpoint xpoint;
		
	xpoint = xline->get_aa();
	ret_val=ret_val*(aa==xpoint);
	
	xpoint = xline->get_bb();
	ret_val=ret_val*(bb==xpoint);
	
	return ret_val;	
}
/**Calculates the area between two lines.*/
//! Calculating the area as a trapezoid.
float Cline::area(Cline &xline)
{
	float h1, h2;
	Cpoint aux_point;
	
	// We ensure that the AA and BB are properly allocated
	adjust();
	xline.adjust();
	// Draw a diagonal across the trapezoid
	Cline aux(aa, xline.get_bb());

	aux_point = xline.get_aa();
	h1 = aux.d2point(&aux_point);
	h2 = aux.d2point(&bb);
	
	return ((h1+h2) * aux.get_length())/2.0;
}

/**Verify that the point aa is the most left and below between points. If not, swap the points.*/
void Cline::adjust()
{
	if(aa.get_xx() == bb.get_xx()){
		if(aa.get_yy() > bb.get_yy()){ swap(); }
	}
	if(aa.get_xx() > bb.get_xx()){ swap(); }
}

void Cline::swap()
{
	Cpoint aux;
	
	aux = aa;
	aa = bb;
	bb = aux;
	set_vv();
}

unsigned short int Cline::intersectionsWithCircle(Cpoint c, double r, vector<Cpoint> &intersections)
{
	intersections.clear();

	// move circle to (0,0)	
	//Cpoint p1 = aa - c; cout << "DEBUG: p1 is " << p1 << endl;
	//Cpoint p2 = bb - c; cout << "DEBUG: p2 is " << p2 << endl;
	Cpoint p1(aa.get_xx() - c.get_xx(), aa.get_yy() - c.get_yy()); //cout << "DEBUG: p1 is " << p1 << endl;
	Cpoint p2(bb.get_xx() - c.get_xx(), bb.get_yy() - c.get_yy()); //cout << "DEBUG: p1 is " << p1 << endl;
	
	// discriminant
	double dx = p2.get_xx() - p1.get_xx();
	double dy = p2.get_yy() - p1.get_yy();
	double dr_sq = dx*dx + dy*dy;
	double D = p1.get_xx() * p2.get_yy() - p2.get_xx() * p1.get_yy();
	double disc = r*r * dr_sq - D*D;
	
	if(disc < 0)
		return 0;
	else if(disc == 0)
	{
		double x = D * dy / dr_sq;
		double y = -D * dx / dr_sq;
		
		// check if point belongs to segment
		Cpoint i = Cpoint(x + c.get_xx(), y + c.get_yy());
		if( i.d2point2(&aa) <= aa.d2point2(&bb) && i.d2point2(&bb) <= aa.d2point2(&bb))
			intersections.push_back( i );
		return intersections.size();
	}
	else
	{
		int sign_dy = (dy >= 0) ? 1 : -1;
		double x1 = ( D * dy + sign_dy * dx * sqrt(disc) ) / dr_sq;
		double x2 = ( D * dy - sign_dy * dx * sqrt(disc) ) / dr_sq;
		double abs_dy = ( dy >= 0 ) ? dy : -dy;
		double y1 = ( -D * dx + abs_dy * sqrt(disc) ) / dr_sq;
		double y2 = ( -D * dx - abs_dy * sqrt(disc) ) / dr_sq;
		Cpoint i1 = Cpoint(x1 + c.get_xx(), y1 + c.get_yy());
		Cpoint i2 = Cpoint(x2 + c.get_xx(), y2 + c.get_yy());
		if( i1.d2point2(&aa) <= aa.d2point2(&bb) && i1.d2point2(&bb) <= aa.d2point2(&bb))
			intersections.push_back( i1 );
		if( i2.d2point2(&aa) <= aa.d2point2(&bb) && i2.d2point2(&bb) <= aa.d2point2(&bb))
			intersections.push_back( i2 );
		return intersections.size();
	}
}

