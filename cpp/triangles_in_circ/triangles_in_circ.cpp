#include<iostream>
#include<vector>
#include<cmath>
#include "combinations.hpp"

#define PI 3.14159265
//#define RAND_MAX 100000

struct tr_phases {
    double phi0{0};
    double phi_l{0};
    double phi_r{0};
};

/* */
double get_phi(point &p);

/* */
std::vector<point> get_rand_circ_points(int i, double r);

/* */
triangle get_equilater_triangle(point &p1, double r);

tr_phases get_equilater_tr_phis(double phi1);

/* */
void order_a_basedon_b(std::vector<point> &a, std::vector<double> &b);

/* */
double distance(point &p1, point &p2) {
    double dy = p1.y - p2.y;
    double dx = p1.x - p2.x;
    return std::sqrt(dx*dx + dy*dy);
};

/* */
double get_perimeter(triangle &t) {
    return distance(t.p1, t.p2) + distance(t.p2, t.p3) + distance(t.p3, t.p1);
};

 int main()
{   
    // get random points belonging to a circonference of radius r and center in the origin (0,0)
    int n_points = 20;
    double radius = 2.0;
    std::vector<point> points = get_rand_circ_points(n_points, radius);

    // construct phases vector from points
    std::vector<double> phis(n_points);
    for (int i=0; i<n_points; i++) { 
        phis[i] = get_phi(points[i]);
        }

    // order_a_basedon_b(points, phis);

    // std::cout << points.size() << std::endl;
    // std::cout << phis.size() << std::endl;

    for (int i=0; i<n_points; i++) { 
        std::cout << "point: " << points[i].x << "  " << points[i].y << std::endl;
        std::cout << "phi: " << phis[i] << std::endl;
    }


    // ..
    double phi0;
    double phi_left;
    double phi_right;

    double temp_perimeter;
    double max_perimeter;
    triangle temp_triangle;
    triangle max_triangle;

    for (int i=0; i<n_points; i++) {
        phi0 = phis.at(i);
        // phase shift by 120 degrees anti clock wise
        phi_left = phi0 + 2.0/3.0 * PI; 
        phi_left = (phi_left < 0) ? phi_left + 2*PI : phi_left;
        // phase shift by 120 degrees clock wise
        phi_right = phi0 - 2.0/3.0 * PI;
        phi_right = (phi_right < 0) ? phi_right + 2*PI : phi_right;

        // find nearest value to phi_left in phis vector
        double diff1;
        double diff2;
        double diff = 100; //init to a very large value
        double temp_diff;

        int index_left;
        for (int k=0; k<n_points; k++) {
            diff1 = std::abs(phis.at(k) - phi_left);
            diff2 = std::abs((phi_left + 2*PI) - phis.at(i));
            temp_diff = (diff1 < diff2) ? diff1 : diff2;
            
            if (diff > temp_diff) {
                diff = temp_diff;
                index_left = k;
            }
        }

        diff = 100;
        int index_right;
        for (int j=0; j<n_points; j++) {
            diff1 = std::abs(phis.at(j) - phi_right);
            diff2 = std::abs((phi_right + 2*PI) - phis.at(i));
            temp_diff = (diff1 < diff2) ? diff1 : diff2;
            
            if (diff > temp_diff) {
                diff = temp_diff;
                index_right = j;
            }
        }

        temp_triangle = {points.at(i), points.at(index_left), points.at(index_right)};
        temp_perimeter = get_perimeter(temp_triangle);

        if (temp_perimeter > max_perimeter) {
            max_perimeter = temp_perimeter;
            max_triangle = temp_triangle;
        }
    }

    // print ..
    std::cout << "max perimeter: " << max_perimeter << std::endl;
    std::cout << "points: (" << max_triangle.p1.x << "," << max_triangle.p1.y << ")  ("
                << max_triangle.p2.x << "," << max_triangle.p2.y << ")  ("
                << max_triangle.p3.x << "," << max_triangle.p3.y << ")" << std::endl;


    // combinations ..
    Combinations comb;
    int k = 3;
    comb.compute(points.data(), points.size(), 0, k); // can i use an iterator? do i have advantages in that?
    std::vector<triangle> tr_combinations = comb.get_combinations();

    /*std::cout << tr_combinations.size() << std::endl;
    int count=0;
    for (auto  t : tr_combinations) {
            count++;

            std::cout << "points: (" << t.p1.x << "," << t.p1.y << ")  ("
                << t.p2.x << "," << t.p2.y << ")  ("
                << t.p3.x << "," << t.p3.y << ")" << std::endl;
        }
    std::cout << count << std::endl;*/

    max_perimeter = 0;
    for (auto  t : tr_combinations) {
        temp_perimeter = get_perimeter(t);

        if (temp_perimeter > max_perimeter) {
            max_perimeter = temp_perimeter;
            max_triangle = t;
        }
    }
    // print ..
    std::cout << "max perimeter: " << max_perimeter << std::endl;
    std::cout << "points: (" << max_triangle.p1.x << "," << max_triangle.p1.y << ")  ("
                << max_triangle.p2.x << "," << max_triangle.p2.y << ")  ("
                << max_triangle.p3.x << "," << max_triangle.p3.y << ")" << std::endl;


    return 0; 
}


double get_phi(point &p) {
    double phi = std::atan(p.y / p.x);

    if (p.x < 0 && p.y > 0)
        phi = phi + PI;
    else if (p.x < 0 && p.y < 0)
        phi = phi + PI;
    else if (p.x > 0 && p.y < 0)
        phi = phi + 2*PI;

    return phi;
};

std::vector<point> get_rand_circ_points(int n, double r) {
    std::srand(1);

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

triangle get_equilater_triangle(point &p1, double r) {
    double phi = get_phi(p1);
    // phase shift by 60 degrees anti clock wise
    double phi_left = phi + 2.0/3.0 * PI; 
    // phase shift by 60 degrees clock wise
    double phi_right = phi - 2.0/3.0 * PI;

    point p2 {r*std::cos(phi_left), r*std::sin(phi_left)};
    point p3 {r*std::cos(phi_right), r*std::sin(phi_right)};

    return triangle{p1, p2, p3};
};


void order_a_basedon_b(std::vector<point> &a, std::vector<double> &b) {
    if (a.size() != b.size())
        int lll;
    
    int n = a.size();
    double min;
    int ind_min;
    for (int i=0; i<n-1; i++) {
        // swap min in [i,n] with i elem
        min = b[i];
        for (int k=i+1; k<n; k++) {
            if (b[k] < min) {
                min = b[k]; // se non entra main in questo if succede un seg fault
                ind_min = k;
                }
            }
        //swap ind_min with i
        std::swap<point>(a[i], a[ind_min]);
        std::swap<double>(b[i], b[ind_min]);
        }
};

