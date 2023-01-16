#include<vector>

struct point {
    double x{0};
    double y{0};
};

struct triangle {
    point p1;
    point p2;
    point p3;
};

/* */
class Combinations {
    std::vector<point> _combination;
    std::vector<triangle> _combinations;

    public:
    /* Default constructor */
    Combinations() = default;

    /* Default destructor */
    ~Combinations() = default;

    /* */
    std::vector<triangle> get_combinations();

    /* */
    void compute(point* points, int size, int m, int k);  
    
    /* */
    void reset();  
};

void Combinations::compute(point* p, int size, int offset, int k) {
    if (k==0) {
        triangle tr{_combination.at(0), _combination.at(1), _combination.at(2)}; // è uno spreco di mem??
        _combinations.push_back(tr);
    }
    else {
        for (int i=0; i<size; i++) {
                _combination.push_back(*(p+i));
                compute(p+1+i, size-1-i, offset+1, k-1);
                _combination.pop_back();
            }
    }

    return;
};

std::vector<triangle> Combinations::get_combinations() {
    return std::move(_combinations);
};

void Combinations::reset() {
    _combination.clear();
    _combinations.clear();
};