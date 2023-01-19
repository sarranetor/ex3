#include<vector>
#include "utils.hpp"

#ifndef COMBINATIONS_H
#define COMBINATIONS_H

/* Class for computing the "N choose K" combinations of N points with K=3 with recursive method */
class Combinations {
    std::vector<point> _combination;
    std::vector<triangle> _combinations;

    /*
        @param data pointer to the first point of the vector
        @param size size of the vector of points
        @param m needs to be set to 0
        @param k needs to be set to 3
    */
    void _compute(const point* data, const int size, const int m, const int k); 

    public:
    /* Default constructor */
    Combinations() = default;

    /* Default destructor */
    ~Combinations() = default;

    /* 
        @return vector of computed combinations 
        @note computation needs to be run before otherwise will return an empty vector
    */
    std::vector<triangle> get_combinations();

    /*
        Compute combinations in a recursive fashion

        @param data pointer to the first point of the vector
        @param size size of the vector of points
    */
    void compute(const point* data, const int size);  
    
    /* Reset class for computing combinations of a different set of points */
    void reset();  
};

void Combinations::compute(const point* data, const int size) {
    _compute(data, size, 0, 3);
};

void Combinations::_compute(const point* data, const int size, const int m, const int k) {
    if (k==0) {
        _combinations.push_back({_combination.at(0), _combination.at(1), _combination.at(2)});
    }
    else {
        for (int i=0; i<size; i++) {
                _combination.push_back(*(data+i));
                _compute(data+1+i, size-1-i, m+1, k-1);
                _combination.pop_back();
            }
    }
};

std::vector<triangle> Combinations::get_combinations() {
    return std::move(_combinations);
};

void Combinations::reset() {
    _combination.clear();
    _combinations.clear();
};

#endif