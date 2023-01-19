#include<vector>

#ifndef UTILS_H
#define UTILS_H

#define PI 3.14159265

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

/* Struct representing a triangle inscribene in a circumference through the phases of its points */
struct triangle_phis {
    double phi0{0};
    double phi1{0};
    double phi2{0};
};

/*
    Get random points belonging to a circumference of radius r and origin (0,0)

    @param n number of points to output
    @param r radius of circumference
    @param seed seed for random generation
    @return vector of points

*/
std::vector<point> get_rand_circ_points(const int n, const double r, const int seed) {
    std::srand(seed);
    std::vector<point> points(n);

    double phi;
    int n_rand;
    for (int i=0; i<n; i++) {
        n_rand = std::rand();
        // generate random angle from 0 to 2*PI
        phi = (n_rand / static_cast<double>(RAND_MAX)) * 2.0 * PI;

        points[i].x = r * std::cos(phi);
        points[i].y = r * std::sin(phi);
    }

    return points;
};

/*
    Compute euclidean distance btw two points
*/
double distance(const point &p1, const point &p2) {
    double dy = p1.y - p2.y;
    double dx = p1.x - p2.x;
    return std::sqrt(dx*dx + dy*dy);
};

/*
    Get perimete of a triangle
*/
double get_perimeter(const triangle &t) {
    return distance(t.p1, t.p2) + distance(t.p2, t.p3) + distance(t.p3, t.p1);
};

/*
    Print the coordinates (x,y) of each point belonging to a triangle
*/
void print_triangle_points(const triangle &tr) {
    std::cout << "points: (" << tr.p1.x << "," << tr.p1.y << ")  ("
                << tr.p2.x << "," << tr.p2.y << ")  ("
                << tr.p3.x << "," << tr.p3.y << ")" << std::endl;
};

#endif