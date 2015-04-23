
#include <stdlib.h> //srand, rand 
#include <time.h>  //time
#include <iostream>
#include "../association_tree.h"
#include "../association_nnls.h"


int main(int argc, char *argv[])
{
    //variables
    AssociationTree tree_;
    AssociationNNLS nnls_;
    unsigned int nd_ = 2; //# detections
    unsigned int nt_ = 2; //# targets
    std::vector<std::pair<unsigned int, unsigned int> > associationsT_, associationsN_;
    std::vector<unsigned int> unassociatedT_, unassociatedN_;
    double sc_;
    
    //inits
    srand ( time(NULL) );
    
    //splash
    std::cout << "----------------------------------------------------" << std::endl;
    std::cout << "         TEST ASSOCIATION TREE and NNLS"              << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;
        
    std::cout << "STEP 1. Resize and Fill score table" << std::endl;
    tree_.resize(nd_,nt_);
    nnls_.resize(nd_,nt_);
    for (unsigned int ii=0; ii<nd_; ii++)
    {
        for (unsigned int jj=0; jj<nt_; jj++)
        {
            sc_ = (double)rand()/(double)RAND_MAX; 
            tree_.setScore(ii,jj,sc_);
            nnls_.setScore(ii,jj,1-sc_);
        }
    }
    tree_.printScoreTable();
    nnls_.printScoreTable();
    
    std::cout << "STEP 2. Reset, Resize & Fill score table again  " << std::endl;
    tree_.reset();
    nnls_.reset();
    tree_.resize(nd_*2,nt_);
    nnls_.resize(nd_*2,nt_);
    for (unsigned int ii=0; ii<nd_*2; ii++)
    {
        for (unsigned int jj=0; jj<nt_; jj++)
        {
            sc_ = (double)rand()/(double)RAND_MAX; 
            tree_.setScore(ii,jj,sc_);
            nnls_.setScore(ii,jj,1-sc_);
        }
    }
    tree_.printScoreTable();
    nnls_.printScoreTable();

    std::cout << "STEP 3. Solve tree" << std::endl;
    tree_.growTree();
    tree_.computeTree();
    tree_.normalizeTree();
    tree_.solve(associationsT_,unassociatedT_);
    tree_.printTree();//display tree
    
    std::cout << "STEP 4. Solve NNLS" << std::endl;
    nnls_.solve(associationsN_,unassociatedN_);

    //display associations
    std::cout << std::endl; 
    std::cout << "BEST ASSOCIATION EVENT: " << std::endl;
    std::cout << "   TREE PAIRS (" << associationsT_.size() << "): ";
    for(unsigned int ii=0; ii< associationsT_.size(); ii++)
        std::cout << associationsT_.at(ii).first << ":" << associationsT_.at(ii).second << " ";
    std::cout << std::endl;
    std::cout << "   NNLS PAIRS (" << associationsN_.size() << "): ";
    for(unsigned int ii=0; ii< associationsN_.size(); ii++)
        std::cout << associationsN_.at(ii).first << ":" << associationsN_.at(ii).second << " ";
    std::cout << std::endl; 
    
    //display unassociatied dets
    std::cout << std::endl; 
    std::cout << "UNASSOCIATED DETs: " << std::endl;
    std::cout << "   TREE: ";
    for(unsigned int ii=0; ii< unassociatedT_.size(); ii++)
        std::cout << unassociatedT_.at(ii) << " ";
    std::cout << std::endl; 
    std::cout << "   NNLS: ";
    for(unsigned int ii=0; ii< unassociatedN_.size(); ii++)
        std::cout << unassociatedN_.at(ii) << " ";
    std::cout << std::endl;     
    
    //reset
    tree_.reset();
    nnls_.reset();
    
    //end
    return 0;
}

