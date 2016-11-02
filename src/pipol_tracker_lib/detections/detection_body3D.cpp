#include "detection_legs.h"

DetectionBody3D::DetectionBody3D(const Eigen::Vector3d & _point, const ParamsDetectionBody3D *_params) :
	point_(_point(0),_point(1),_point(2)),
	params_(_params)
{
	
}

DetectionBody3D::~DetectionBody3D()
{

}
		
double DetectionBody3D::likelihood(const Eigen::VectorXs & _state) const
{
    double dist_sq, score;

    dist_sq = (_state(0)-_point(0))*(_state(0)-_point(0)) + (_state(1)-_point(1))*(_state(1)-_point(1)) + (_state(2)-_point(2))*(_state(2)-_point(2));
    if ( dist_sq <= params_->person_radius_sq_ ) //closer than radius
    {
        score = params_->K1_ * params_->person_radius_sq_ + 1;
    }
    else //farther than radius
    {
        score = params_->matching_alpha_ * exp( (params_->person_radius_-sqrt(dist_sq))*params_->matching_beta_ );
    }
    return  score;
}

double DetectionBody3D::cost(const Eigen::VectorXs & _state)
{
	//to do
	return 0; 
}
