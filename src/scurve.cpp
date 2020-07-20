#include "scurve.h"
#include <stdlib.h>
#include <math.h>

SCurveGenerator::SCurveGenerator(double in_v0, double in_vs, double in_as, double in_jmax)
{
	v0 = in_v0;
	vs = in_vs;
	double delta_v = vs-v0; 

	a_s   = (delta_v>0)? in_as: (-in_as);
	j_max = (delta_v>0)? in_jmax: (-in_jmax);

	if (abs(delta_v)<=abs(a_s))
	{
		T = 2.0*abs(delta_v/a_s);
		j_max = 2.0*a_s/T;
		t1 = T/2.0;
		v1 = (v0+vs)/2.0;
		v2 = v1;
	}
	else
	{
        t1 = a_s/j_max;
        v1 = v0+(a_s*a_s)/(2.0*j_max);
        v2 = vs-(a_s*a_s)/(2.0*j_max);
        T = (v2-v1)/a_s+t1*2.0;		
	}

    s1 = (v0+(a_s*a_s)/(6.0*j_max))*a_s/j_max;        
	s2 = s1+v1*(T-2.0*t1)+a_s*((T-2.0*t1)*(T-2.0*t1))/2.0;
}

SCurveGenerator::~SCurveGenerator() {}

double SCurveGenerator::s(double t)
{
    if ((t>=0) && (t<=t1))
        return v0*t + j_max*pow(t,3.0)/6.0;
    else if ((t<=T-t1) && (T>2*t1))
    {	t = t - t1;
        return s1 + v1*t + a_s*pow(t,2.0)/2.0;
    }
    else if (t<=T)
    {   t = t - (T-t1);
        return s2 + v2*t + a_s*pow(t,2.0)/2.0 - j_max*pow(t,3.0)/6.0;
    }
    return 0;
}

double SCurveGenerator::v(double t)
{
    if ((t>=0) && (t<=t1))
        return v0 + j_max*pow(t,2.0)/2.0;
    else if ((t<=T-t1) && (T>2*t1))
    {	t = t - t1;
        return v1 + a_s*t;
    }        
    else if (t<=T)
    {	 t = t - (T-t1);
        return v2 + a_s*t - j_max*pow(t,2.0)/2.0;
    }
    return 0;
}

double SCurveGenerator::a(double t)
{
    if ((t>=0) && (t<=t1))
        return j_max*t;
    else if ((t<=T-t1) && (T>2.0*t1))
        return a_s;
    else if (t<=T)
    {	t = t - (T-t1);
        return a_s - j_max*t;
    }           	
}