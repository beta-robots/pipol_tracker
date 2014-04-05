
/*Antonio Larrosa Jimenez, antlarr@arrakis.es, 6-9-98*/

#include "simpleRnd.h"

//#define RAND_UNIFORME (double)random()/(double)RAND_MAX

double random_exp(double landa)
{
return (-log(1.0-RAND_UNIFORME)/(double)landa);
}


double random_normal(double esp,double var)
{
double u1=RAND_UNIFORME;
double u2=RAND_UNIFORME;
double z=sqrt(-2.0*log(u1))*cos(2*M_PI*u2);
return esp+var*z;
}

/*unsigned long random_poisson(double landa)
{
double p=exp(-landa);
double g=p;
double u=RAND_UNIFORME;
unsigned long k=0;
while (u>g)
    {
    p*=(landa/(double)(++k));
    g+=p;
    };
return k;
}*/

unsigned long random_binomial(unsigned long n, double p)
{
double t=p/(1-p);
double u=RAND_UNIFORME;
double p0=pow((1-p),n);
double g=p0;
unsigned int k=0;
while (u>g)
    {
    p0*=t*(n-k)/(k+1);
    g+=p0;
    k++;
    };
return k;
}

long double fact(unsigned long x, unsigned long f)
{
if (x==f) return 1.0;
if (x<f) return 1.0/fact(f,x);
long double t=1.0;
while (x>f)
    {
    t*=x;
    x--;
    };
return (long double)t;
}

double hipergeom_p0(unsigned long N, unsigned long D, unsigned long n)
{
long k=n-N+D;
if (k<0) k=0;
return (double)((fact(D,D-k)*fact(N-D,N-D-n+k)*fact(n,n-k))/(fact(k,1)*fact(N,N-n)));

//return ((double)(fact(D)/fact(D-k))*(fact(N-D)/fact(N-D-n+k))*(fact(n)/fact(n-k)))/(double)(fact(k)*(fact(N)/fact(N-n)));
}

/*long random_hipergeom(unsigned long N, unsigned long D, unsigned long n,double p=0.0)
{
double u=RAND_UNIFORME;
long k=n-N+D;
if (k<0) k=0;
if (p==0.0) p=hipergeom_p0(N,D,n);
double g=p;
while (u>g)
    {
    p*=((double)(D-k)*(n-k))/(double)((k+1)*(N-D-n+k+1));
    g+=p;
    k++;
    };
return k;
}*/
