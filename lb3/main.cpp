#include <iostream>

#include <cmath>


double
f(const double M, const double gamma, double nu_target)
{
    if (M <= 1.0) return NAN;
    
    double sqrt_M2_sub_1 = std::sqrt(M * M - 1);
    return std::sqrt((gamma + 1) / (gamma - 1)) *
           std::atan(std::sqrt((gamma - 1) / (gamma + 1)) * sqrt_M2_sub_1) -
           std::atan(sqrt_M2_sub_1) -
           nu_target;
}


double
df(const double M, const double gamma)
{
    if (M <= 1.0) return NAN;
    
    // M^2 - 1 is common term
    double M2_sub_1 = M * M - 1;
    double numerator = 2 * std::sqrt(M2_sub_1);
    double denominator = M * ((gamma + 1) + M2_sub_1 * (gamma - 1));
    return numerator / denominator;
}


void
find_bounds(double* a, double* b, const double gamma, const double nu_target)
{
    // Doesn't work with this. <TODO> later
    // *a = 1.0 + 1e-20;
    *a = 1.0 + 1e-5;
    *b = *a;

    double step = 0.1;

    while (f(*a, gamma, nu_target) * f(*b, gamma, nu_target) > 0) {
        *b += step;
        step *= 2.0;
        if (*b > 1e6)
            throw std::runtime_error("find_bounds(): Bounds was not found");
    }
}


double
bisection_method(const double nu_target, const double gamma,
                 const size_t max_iterations,
                 const double tolerance)
{
    printf("Bisection method for [nu_target=%lf], [gamma=%lf]\n",
           nu_target, gamma);

    double a, b;
    find_bounds(&a, &b, gamma, nu_target);
    printf("Initial bisection bound: [%lf;%lf]\n", a, b);

    double c;
    for (size_t i = 0; i < max_iterations; ++i) {
        c = (a + b) * 0.5;
        printf("iteraton=%ld: c=%lf\n", i, c);

        const double fc = f(c, gamma, nu_target);
        if (fabs(fc) < tolerance) return c;

        if (f(a, gamma, nu_target) * fc < 0)
            b = c;
        else
            a = c;
    }

    throw std::runtime_error("bisection_method(): Failed to converge");
}

double
chord_method(const double nu_target, const double gamma,
             const size_t max_iterations,
             const double tolerance)
{
    printf("Chords method for [nu_target=%lf], [gamma=%lf]\n",
           nu_target, gamma);

    double a, b;
    find_bounds(&a, &b, gamma, nu_target);
    printf("Initial chords bound: [%lf;%lf]\n", a, b);

    double fa = f(a, gamma, nu_target);
    double fb = f(b, gamma, nu_target);

    for (size_t i = 0; i < max_iterations; ++i) {
        if (fabs(fa) < tolerance) return a;
        if (fabs(fb) < tolerance) return b;

        // Linear interpolation between (a, fa) and (b, fb)
        double c = (a * fb - b * fa) / (fb - fa);
        double fc = f(c, gamma, nu_target);
        printf("iteraton=%ld: c=%lf\n", i, c);

        if (fabs(fc) < tolerance) return c;

        if (fa * fb < 0) {
            b = c;
            fb = fc;
        } else {
            a = c;
            fa = fc;
        }
    }

    throw std::runtime_error("bisection_method(): Failed to converge");
}


int
main(void)
{
    const double nu_target = .46;   // Angle
    const double gamma = 1.4;       // Heat capacity ratio
    

    printf("%lf\n", bisection_method(nu_target, gamma, 1000, 1e-10));
    printf("%lf\n", chord_method(nu_target, gamma, 1000, 1e-10));

    return 0;
}
