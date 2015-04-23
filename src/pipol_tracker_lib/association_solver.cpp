
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
    scores_(_det_i,_tar_j) = _m_ij;
}
     
void AssociationSolver::printScoreTable() const
{
    scores_.print();
}
    