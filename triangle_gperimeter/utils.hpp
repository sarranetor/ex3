#include<vector>

#ifndef UTILS_H
#define UTILS_H

/* Struc representing a point in euclidean space */
struct point {
    double x{0};
    double y{0};
};

/* Struct representing a triangle in euclidean space */
struct triangle {
    point p1;
    point p2;
    point p3;
};

/* Class for computing the "N choose K" combinations of N points with K=3 */
class Combinations {
    std::vector<point> _combination;
    std::vector<triangle> _combinations;

    /*
        @param data pointer to the first point of the vector
        @param size size of the vector of points
        @param m needs to be set to 0
        @param k needs to be set to 3
    */
    void _compute(const point* data, int size, int m, int k); 

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
        Compute combinations in a recursive fation

        @param data pointer to the first point of the vector
        @param size size of the vector of points
    */
    void compute(const point* data, int size);  
    
    /* Reset class for computing combinations of a different set of points */
    void reset();  
};

void Combinations::compute(const point* data, int size) {
    _compute(data, size, 0, 3);
};

void Combinations::_compute(const point* data, int size, int m, int k) {
    if (k==0) {
        triangle tr{_combination.at(0), _combination.at(1), _combination.at(2)}; // è uno spreco di mem??
        _combinations.push_back(tr);
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