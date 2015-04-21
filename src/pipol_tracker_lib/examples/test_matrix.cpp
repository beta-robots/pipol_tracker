
#include "../matrix.h"

int main(int argc, char *argv[])
{
    //declare a matrix with sizes
    Matrixx<double> M(3,7);
    
    //fill content & print
    M(1,2) = 1;  
    M.print(); 
    
    //resize
    M.resize(5,9);
    
    //fill content & print
    M(3,5) = 2;
    M.print(); 
    
    //clear
    M.clear();
    
    //resize
    M.resize(5,3);

    //fill content & print
    M(4,1) = 3;
    M.print(); 
    
    //fill with bad indexes
    M(3,5) = 4;  
    
    //end
    return 0;
}

