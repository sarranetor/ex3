#include<iostream>
#include<vector>
#include<cmath>
#include "utils.hpp"

#define PI 3.14159265

struct triangle_phis {
    double phi0{0};
    double phi1{0};
    double phi2{0};
};

/* */
double get_phi(point &p);

/* */
std::vector<point> get_rand_circ_points(int n, double r);

/* */
triangle_phis get_equilater_tr_phis(double phi0);

/* */
int get_closest_phi(std::vector<double> &phis, double phi);

/* */
double distance(point &p1, point &p2);

/* */
double get_perimeter(triangle &t);

/* */
void print_triangle_points(triangle &tr);

/* ------------->  MAIN FUNCTION */
 int main()
{   
    // get random points belonging to a circonference of radius r and center in the origin (0,0)
    int n_points = 20;
    double radius = 2.0;
    std::vector<point> points = get_rand_circ_points(n_points, radius);

    /* Algorithim Start */
    // store algorithm resulting triangle with greatest perimeter and its perimeter
    double temp_perimeter;
    double max_perimeter;
    triangle temp_triangle;
    triangle max_triangle;

    /* --> Use Equilater triangle method -> method1 */
    // construct phases vector from points
    std::vector<double> phis(n_points);
    for (int i=0; i<n_points; i++) { 
        phis[i] = get_phi(points[i]);
        }

    for (int i=0; i<n_points; i++) {
        triangle_phis tr_phis = get_equilater_tr_phis(phis.at(i));

        int index1 = get_closest_phi(phis, tr_phis.phi1);
        int index2 = get_closest_phi(phis, tr_phis.phi2);

        temp_triangle = {points.at(i), points.at(index1), points.at(index2)};
        temp_perimeter = get_perimeter(temp_triangle);

        if (temp_perimeter > max_perimeter) {
            max_perimeter = temp_perimeter;
            max_triangle = temp_triangle;
        }
    }

    // method1 results
    std::cout << "max perimeter: " << max_perimeter << std::endl;
    print_triangle_points(max_triangle);


    /* --> Use combination method -> method2 */
    Combinations comb;
    int k = 3;
    comb.compute(points.data(), points.size(), 0, k); // can i use an iterator? do i have advantages in that?
    std::vector<triangle> tr_combinations = comb.get_combinations();

    max_perimeter = 0;
    for (auto  tr : tr_combinations) {
        temp_perimeter = get_perimeter(tr);

        if (temp_perimeter > max_perimeter) {
            max_perimeter = temp_perimeter;
            max_triangle = tr;
        }
    }

    // method2 results
    std::cout << "max perimeter: " << max_perimeter << std::endl;
    print_triangle_points(max_triangle);

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

triangle_phis get_equilater_tr_phis(double phi0) {
    // phase shift by 120 degrees anti clock wise
    double phi_left = phi0 + 2.0/3.0 * PI; 
    phi_left = (phi_left < 0) ? phi_left + 2*PI : phi_left;
    // phase shift by 120 degrees clock wise
    double phi_right = phi0 - 2.0/3.0 * PI;
    phi_right = (phi_right < 0) ? phi_right + 2*PI : phi_right;

    return triangle_phis{phi0, phi_left, phi_right};
};

int get_closest_phi(std::vector<double> &phis, double phi) {
    int n = phis.size();
    double diff1(0);
    // diff2 is used to correctly compute the distance in rad
    // btw points near the zero phi angle. for example btw 0.1 rad and 6.26 rad.
    double diff2(0);
    //init to a very large value
    double diff = 100; 
    double temp_diff(0);
    int index(0);

    for (int i=0; i<n; i++) {
        diff1 = std::abs(phis.at(i) - phi);
        diff2 = std::abs((phi + 2*PI) - phis.at(i));

        temp_diff = (diff1 < diff2) ? diff1 : diff2;
        
        // if newly computed difference is les than stored, store new data
        if (diff > temp_diff) {
            diff = temp_diff;
            index = i;
        }
    }  

    return index;
};

double distance(point &p1, point &p2) {
    double dy = p1.y - p2.y;
    double dx = p1.x - p2.x;
    return std::sqrt(dx*dx + dy*dy);
};

double get_perimeter(triangle &t) {
    return distance(t.p1, t.p2) + distance(t.p2, t.p3) + distance(t.p3, t.p1);
};

void print_triangle_points(triangle &tr) {
    std::cout << "points: (" << tr.p1.x << "," << tr.p1.y << ")  ("
                << tr.p2.x << "," << tr.p2.y << ")  ("
                << tr.p3.x << "," << tr.p3.y << ")" << std::endl;
};