
#include "association_solver.h"

AssociationSolver::AssociationSolver() :
    nd_(0),
    nt_(0)
{
    //
}

AssociationSolver::~AssociationSolver()
{
    //
}
        
void AssociationSolver::resize(const unsigned int _n_det, const unsigned int _n_tar)
{
    nd_ = _n_det;
    nt_ = _n_tar;
    scores_.resize(nd_,nt_+1); //"+1" to account for void target, which manages unassociated detections
//     scores_.resize(nd_);
//     for ( unsigned int ii = 0; ii < nd_; ii++)
//     {
//         scores_.at(ii).resize(nt_ + 1);//"+1" to account for void target, which manages unassociated detections
//     }
}

unsigned int AssociationSolver::numDetections()
{
    return nd_;
}
     
unsigned int AssociationSolver::numTargets()
{
    return nt_;
}
     
void AssociationSolver::setScore(const unsigned int _det_i, const unsigned int _tar_j, const double _m_ij)
{
    //scores_.at(_det_i).at(_tar_j) = _m_ij;
    scores_(_det_i,_tar_j) = _m_ij;
}
     
void AssociationSolver::printScoreTable() const
{
//     for (unsigned int ii=0; ii<scores_.size(); ii++)
//     {
//         for (unsigned int jj=0; jj<scores_.at(ii).size(); jj++)
//         {
//             std::cout << scores_.at(ii).at(jj) << " ";
//         }
//         std::cout << std::endl;
//     }
//     std::cout << std::endl;
    scores_.print();
}
    