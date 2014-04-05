

/*Antonio Larrosa Jimenez, antlarr@arrakis.es, 6-9-98*/

#ifndef simpleRnd_h
#define simpleRnd_h


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define RAND_UNIFORME (double)rand()/(double)RAND_MAX

double random_exp(double landa);
double random_normal(double esp,double var);
//unsigned long random_poisson(double landa);
unsigned long random_binomial(unsigned long n, double p);
long double fact(unsigned long x, unsigned long f);
double hipergeom_p0(unsigned long N, unsigned long D, unsigned long n);
//long random_hipergeom(unsigned long N, unsigned long D, unsigned long n,double p=0.0);

#endif
