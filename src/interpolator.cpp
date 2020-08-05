#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "package1/interpolation.h"

using namespace alglib;


int main(int argc, char **argv)
{
    //
    // This example demonstrates polynomial fitting.
    //
    // Fitting is done by two (M=2) functions from polynomial basis:
    //     f0 = 1
    //     f1 = x
    // Basically, it just a linear fit; more complex polynomials may be used
    // (e.g. parabolas with M=3, cubic with M=4), but even such simple fit allows
    // us to demonstrate polynomialfit() function in action.
    //
    // We have:
    // * x      set of abscissas
    // * y      experimental data
    //
    // Additionally we demonstrate weighted fitting, where second point has
    // more weight than other ones.
    //
    real_1d_array x = "[0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]";
    real_1d_array y = "[0.00,0.05,0.26,0.32,0.33,0.43,0.60,0.60,0.77,0.98,1.02]";
    ae_int_t m = 2;
    double t = 2;
    ae_int_t info;
    barycentricinterpolant p;
    polynomialfitreport rep;
    double v;

    //
    // Fitting without individual weights
    //
    // NOTE: result is returned as barycentricinterpolant structure.
    //       if you want to get representation in the power basis,
    //       you can use barycentricbar2pow() function to convert
    //       from barycentric to power representation (see docs for
    //       POLINT subpackage for more info).
    //
    polynomialfit(x, y, m, info, p, rep);
    v = barycentriccalc(p, t);
    printf("%.2f\n", double(v)); // EXPECTED: 2.011

    //
    // Fitting with individual weights
    //
    // NOTE: slightly different result is returned
    //
    real_1d_array w = "[1,1.414213562,1,1,1,1,1,1,1,1,1]";
    real_1d_array xc = "[]";
    real_1d_array yc = "[]";
    integer_1d_array dc = "[]";
    polynomialfitwc(x, y, w, xc, yc, dc, m, info, p, rep);
    v = barycentriccalc(p, t);
    printf("%.2f\n", double(v)); // EXPECTED: 2.023
    return 0;
}
