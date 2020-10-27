#include "kinematik.h"
#include <cmath>

double inv_kin_theta1(double x,double y,double theta2)
{
    return atan((y*(18.5*cos(theta2)+16.4)-x*18.5*sin(theta2))/(x*(18.5*cos(theta2)+16.4)+y*18.5*sin(theta2)));
}

double inv_kin_theta2(double x,double y)
{
    return acos((x*x+y*y-16.4*16.4-18.5*18.5)/(2*18.5*16.4));
}
