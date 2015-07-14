#include "timeStamp.h"

TimeStamp::TimeStamp()
{
	timeStamp = 0;
}

TimeStamp::~TimeStamp()
{
	//nothing to do
}

void TimeStamp::setToNow()
{
	timeval ts;
	gettimeofday(&ts, NULL); 
	timeStamp = (double)(ts.tv_sec + ts.tv_usec/1e6);	
}

void TimeStamp::set(timeval ts)
{
	timeStamp = (double)(ts.tv_sec + ts.tv_usec/1e6);	
}

void TimeStamp::set(unsigned long int sec, unsigned long int nanosec)
{
	timeStamp = (double)(sec + nanosec/1e9);	
}

void TimeStamp::set(double ts)
{
	timeStamp = ts;
}

double TimeStamp::get() const
{
	return timeStamp;
}

unsigned long int TimeStamp::getSeconds()
{
	unsigned long int ts;
	ts = (unsigned long int)floor(timeStamp);
	return ts;
}

unsigned long int TimeStamp::getNanoSeconds()
{
	double ts;
	ts = floor(timeStamp);
	return (unsigned long int)((timeStamp-ts)*1e9);
}

void TimeStamp::print(std::ostream *ost) const
{
	std::streamsize nn;
	//std::streamsize ww;
	std::ios_base::fmtflags fmtfl;
	//char cc; 
	
	fmtfl = ost->flags(std::ios::left);
        ost->setf(std::ios::fixed,std::ios::floatfield);
	//cc = ost->fill('0');
        nn = ost->precision(TIME_STAMP_DIGITS);
	//ww = ost->width(TIME_STAMP_DIGITS);
	
	(*ost) << this->timeStamp;
	
	ost->flags(fmtfl);
	//ost->fill(cc);
        ost->precision(nn);
	//ost->width(ww);
}
