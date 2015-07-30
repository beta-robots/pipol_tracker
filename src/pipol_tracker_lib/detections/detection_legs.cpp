#include "detection_legs.h"

DetectionLegs::DetectionLegs(const Eigen::Vector2d & _point, const ParamsDetectionLegs *_params) :
	point_(_point(0),_point(1)),
	params_(_params)
{
	
}

DetectionLegs::~DetectionLegs()
{

}
		
double DetectionLegs::likelihood(const Eigen::VectorXs & _state) const
{
	double dist_sq, score;

    dist_sq = (_state(0)-_point(0))*(_state(0)-_point(0)) + (_state(1)-_point(1))*(_state(1)-_point(1));
    if ( dist_sq <= params_->person_radius_sq_ )
    {
        score = params_->K1_ * params_->person_radius_sq_ + 1;
    }
    else
    {
        score = params_->matching_alpha_ * exp( (params_->person_radius_-sqrt(dist_sq))*params_->matching_beta_ );
    }
    return  score;
}

double DetectionLegs::cost(const Eigen::VectorXs & _state)
{
	//to do
	return 0; 
}
