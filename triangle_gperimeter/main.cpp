#include<iostream>
#include<vector>
#include<cmath>
#include "utils.hpp"
#include "combinations.hpp"

/*
    @param p 
    @return phase of the point in [0,2PI] range
*/
double get_phi(const point &p);

/* 
    Given the phase of a point belonging to a circumference, compute the phases of 
    all points that with it construct an equilater

    @param phi0 phase of one point
    @return phases of the three points belonging to the equilater triangle in [0,2PI] range
    @note given a circumference an equilater triangle is the triangle with greatest perimeter
*/
triangle_phis get_equilater_tr_phis(double phi0);

/* 
    Find the nearest point to phi in phis vector, points are assumed to belong to a circumference

    @param phis vector of phases belonging to points on a circumference
    @param phi
    @return index of nearest point to phi in phis
*/
int get_closest_phi(const std::vector<double> &phis, double phi);


 int main()
{   
    // get random points belonging to a circumference of radius r and center in the origin (0,0)
    int n_points = 20;
    double radius = 2.0;
    std::vector<point> points = get_rand_circ_points(n_points, radius, 1);

    /* Algorithims Start */

    double temp_perimeter(0);
    triangle temp_triangle;

    /* ------> Use Equilater triangle method -> method1 */
    // store variable for resulting triangle with greatest perimeter and its perimeter
    double max_perimeter_m1(0);
    triangle max_triangle_m1;

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

        // ..
        if (temp_perimeter > max_perimeter_m1) {
            max_perimeter_m1 = temp_perimeter;
            max_triangle_m1 = temp_triangle;
        }
    }

    // method1 results
    std::cout << "Equilater Triangles Method " << std::endl << "perimeter: " << max_perimeter_m1 << std::endl;
    print_triangle_points(max_triangle_m1);


    /* ------> Use combination method -> method2 */
    double max_perimeter_m2(0);
    triangle max_triangle_m2;

    Combinations comb;
    comb.compute(points.data(), points.size());
    std::vector<triangle> tr_combinations = comb.get_combinations();

    // find combination with greatest perimeter
    for (auto&  tr : tr_combinations) {
        temp_perimeter = get_perimeter(tr);

        // ..
        if (temp_perimeter > max_perimeter_m2) {
            max_perimeter_m2 = temp_perimeter;
            max_triangle_m2 = tr;
        }
    }

    // method2 results
    std::cout << std::endl;
    std::cout << "Find Combinations Method " << std::endl << "perimeter: "  << max_perimeter_m2 << std::endl;
    print_triangle_points(max_triangle_m2);

    return 0; 
}



double get_phi(const point &p) {
    double phi = std::atan(p.y / p.x);

    if (p.x < 0 && p.y > 0)
        phi = phi + PI;
    else if (p.x < 0 && p.y < 0)
        phi = phi + PI;
    else if (p.x > 0 && p.y < 0)
        phi = phi + 2*PI;

    return phi;
};

triangle_phis get_equilater_tr_phis(double phi0) {
    // phase shift by 120 degrees anti clock wise
    double phi_left = phi0 + 2.0/3.0 * PI;
    // phase shift by 120 degrees clock wise
    double phi_right = phi0 - 2.0/3.0 * PI;
    
    // to get a phase in [0,2PI] range
    phi_left = (phi_left < 0) ? phi_left + 2*PI : phi_left;
    phi_right = (phi_right < 0) ? phi_right + 2*PI : phi_right;

    return triangle_phis{phi0, phi_left, phi_right};
};

int get_closest_phi(const std::vector<double> &phis, double phi) {
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