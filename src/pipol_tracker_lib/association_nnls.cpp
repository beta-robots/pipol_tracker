#include "association_nnls.h"

AssociationNNLS::AssociationNNLS() :
    max_dist_(MAX_DIST_DEFAULT)
{
    //
}
        
AssociationNNLS::~AssociationNNLS()
{
    //
}

void AssociationNNLS::setMaxDist(const double _max_dist)
{
    max_dist_ = _max_dist;
}
        
void AssociationNNLS::reset()
{
    nd_ = 0; 
    nt_ = 0;
    scores_.clear(); 
    i_mask_.clear();
    j_mask_.clear();
}
            
void AssociationNNLS::resize(const unsigned int _n_det, const unsigned int _n_tar)
{
    nd_ = _n_det; //detections 
    nt_ = _n_tar; //targets
    scores_.resize(nd_, nt_); 
    i_mask_.resize(nd_, false);
    j_mask_.resize(nt_, false);
}
               
//void AssociationNNLS::solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<unsigned int> & _unassoc)
void AssociationNNLS::solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<bool> & _associated_mask)
{
    bool min_found = true; 
    double min_value; 
    unsigned int ii, jj, ii_min, jj_min;
    
    //resize _associated_mask and resets it to false
    _associated_mask.resize(nd_,false);
    
    //find nearest neighbors by successive passing and masking through the scores_ matrix
    while(min_found)
    {
        min_found = false; 
        min_value = max_dist_; 

        for(ii = 0; ii< nd_; ii++)
        {
            if ( i_mask_[ii] == false)
            {
                for(jj = 0; jj< nt_; jj++)
                {
                    if ( j_mask_[jj] == false)
                    {
                        if ( (scores_(ii,jj) < max_dist_) && (scores_(ii,jj) < min_value) )
                        {
                            min_value = scores_(ii,jj); 
                            min_found = true; 
                            ii_min = ii; 
                            jj_min = jj; 
                        }
                    }
                }
            }
        }
        
        if (min_found)
        {
            i_mask_[ii_min] = true;
            j_mask_[jj_min] = true;
            _associated_mask.at(ii_min) = true; 
            _pairs.push_back( std::pair<unsigned int, unsigned int>(ii_min, jj_min) );
        }
    }
    
    //set unassociated detections
//     for(ii = 0; ii< nd_; ii++)
//     {
//         if (i_mask_[ii] == false)
//         {
//             _unassoc.push_back(ii);
//         }
//     }
    
}