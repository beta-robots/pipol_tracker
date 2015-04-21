
#ifndef matrix_H
#define matrix_H

//std
#include <iostream>
#include <vector>
#include <assert.h> //assert


template <typename T>
class Matrixx
{
    protected: 
        unsigned int rows_, cols_;
        std::vector<T> inner_;

    public:
        Matrixx() :
            rows_(0),
            cols_(0)
        {
            //
        }
        
        Matrixx(unsigned int _rows, unsigned int _cols) :
            rows_ (_rows), 
            cols_ (_cols),
            inner_(rows_*cols_)
        {
            //
        }
        
        ~Matrixx() 
        {
            //
        }
                
        void clear()
        {
            inner_.clear();
        }
        
        void resize(unsigned int _rows, unsigned int _cols)
        {
            rows_ = _rows; 
            cols_ = _cols; 
            inner_.resize (rows_*cols_);
            //std::cout << "Resizing matrix to " << rows_ << " x " << cols_ << std::endl;
        }
        
        unsigned int size() const
        {
            return rows_*cols_;
        }

        T& operator()(unsigned int _i, unsigned int _j)
        {
            assert( (_i < rows_) && (_j < cols_) && "Matrix::operator(): Wrong matrix indexes. Program abort.");
            return inner_[cols_*_i + _j];
        }
        
        const T& operator()(unsigned int _i, unsigned int _j) const
        {
            assert( (_i < rows_) && (_j < cols_) && "Matrix::operator(): Wrong matrix indexes. Program abort.");
            return inner_[cols_*_i + _j];
        }
                
        void print() const
        {
            for(unsigned int ii=0; ii<rows_; ii++)
            {
                for(unsigned int jj=0; jj<cols_; jj++)
                {
                    std::cout << inner_[cols_*ii + jj] << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
        
};
#endif
