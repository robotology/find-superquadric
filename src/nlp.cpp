/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file nlp.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cmath>
#include <limits>

#include <yarp/math/Math.h>

#include "nlp.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;

/****************************************************************/
bool SuperQuadricNLP::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,
                                   Ipopt::Index &nnz_jac_g,
                                   Ipopt::Index &nnz_h_lag,
                                   IndexStyleEnum &index_style)
{
    n=9; m=1;
    nnz_jac_g=2;
    nnz_h_lag=0;
    index_style=TNLP::C_STYLE;
    return true;
}

/****************************************************************/
bool SuperQuadricNLP::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l,
                                      Ipopt::Number *x_u, Ipopt::Index m,
                                      Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    Vector margin(3);
    margin[0]=0.25*(bounds(0,1)-bounds(0,0));
    margin[1]=0.25*(bounds(1,1)-bounds(1,0));
    margin[2]=0.25*(bounds(2,1)-bounds(2,0));

    // center
    x_l[0]=bounds(0,0)+margin[0]; x_u[0]=bounds(0,1)-margin[0];
    x_l[1]=bounds(1,0)+margin[1]; x_u[1]=bounds(1,1)-margin[1];
    x_l[2]=bounds(2,0)+margin[2]; x_u[2]=bounds(2,1)-margin[2];
    // angle around z-axis
    x_l[3]=0.0; x_u[3]=2.0*M_PI;
    // size
    x_l[4]=0.001; x_u[4]=numeric_limits<double>::infinity();
    x_l[5]=0.001; x_u[5]=numeric_limits<double>::infinity();
    x_l[6]=0.001; x_u[6]=numeric_limits<double>::infinity();
    // shape
    x_l[7]=0.1; x_u[7]=1.0;
    x_l[8]=0.1; x_u[8]=1.0;
    // limit on z-min
    g_l[0]=bounds(2,0); g_u[0]=numeric_limits<double>::infinity();
    return true;
}

/****************************************************************/
bool SuperQuadricNLP::get_starting_point(Ipopt::Index n, bool init_x,
                                         Ipopt::Number *x, bool init_z,
                                         Ipopt::Number *z_L, Ipopt::Number *z_U,
                                         Ipopt::Index m, bool init_lambda,
                                         Ipopt::Number *lambda)
{                                        
    x[0]=centroid[0];
    x[1]=centroid[1];
    x[2]=centroid[2];
    x[3]=0.0;
    x[4]=0.5*(bounds(0,1)-bounds(0,0));
    x[5]=0.5*(bounds(1,1)-bounds(1,0));
    x[6]=0.5*(bounds(2,1)-bounds(2,0));
    x[7]=1.0;
    x[8]=1.0;
    return true;
}

/****************************************************************/
bool SuperQuadricNLP::eval_f(Ipopt::Index n, const Ipopt::Number *x,
                             bool new_x, Ipopt::Number &obj_value)
{
    Vector c(3),s(3);
    c[0]=x[0];
    c[1]=x[1];
    c[2]=x[2];
    const double &a=x[3];
    s[0]=x[4];
    s[1]=x[5];
    s[2]=x[6];
    const double &e1=x[7];
    const double &e2=x[8];

    Vector rot(4,0.0); rot[2]=1.0; rot[3]=a;
    Matrix T=axis2dcm(rot);
    T.setSubcol(c,0,3);
    T=SE3inv(T);

    obj_value=0.0;
    Vector p1(4,1.0);
    for (auto &p:points)
    {
        p1.setSubvector(0,p);
        p1=T*p1;
        double tx=pow(abs(p1[0]/s[0]),2.0/e2);
        double ty=pow(abs(p1[1]/s[1]),2.0/e2);
        double tz=pow(abs(p1[2]/s[2]),2.0/e1);
        double F1=pow(pow(tx+ty,e2/e1)+tz,e1)-1.0;
        double penalty=(F1<0.0?inside_penalty:1.0);
        obj_value+=F1*F1*penalty;
    }
    obj_value*=(s[0]*s[1]*s[2])/points.size();
    return true;
}

/****************************************************************/
bool SuperQuadricNLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number *x,
                                  bool new_x, Ipopt::Number *grad_f)
{
    Vector c(3),s(3);
    c[0]=x[0];
    c[1]=x[1];
    c[2]=x[2];
    const double &a=x[3];
    s[0]=x[4];
    s[1]=x[5];
    s[2]=x[6];
    const double &e1=x[7];
    const double &e2=x[8];

    Vector rot(4,0.0); rot[2]=1.0; rot[3]=a;
    Matrix T=axis2dcm(rot);
    T.setSubcol(c,0,3);
    T=SE3inv(T);

    for (Ipopt::Index i=0; i<n; i++)
        grad_f[i]=0.0;

    double coeff=s[0]*s[1]*s[2];
    Vector p1(4,1.0);
    for (auto &p:points)
    {
        p1.setSubvector(0,p);
        p1=T*p1;
        double tx=pow(abs(p1[0]/s[0]),2.0/e2);
        double ty=pow(abs(p1[1]/s[1]),2.0/e2);
        double tz=pow(abs(p1[2]/s[2]),2.0/e1);
        double F1=pow(pow(tx+ty,e2/e1)+tz,e1)-1.0;
        double penalty=(F1<0.0?inside_penalty:1.0);

        double tmp1=2.0*coeff*F1*penalty;
        double tmp2=F1*F1*penalty;

        double t11=cos(a)*(c[0]-p[0])+sin(a)*(c[1]-p[1]);
        double t10=cos(a)*(c[1]-p[1])-sin(a)*(c[0]-p[0]);
        double t9=pow(abs(t11)/s[0],2.0/e2);
        double t8=pow(abs(t10)/s[1],2.0/e2);
        double t7=abs(c[2]-p[2])/s[2];
        double t6=pow(t9+t8,e2/e1);
        double t5=pow(t7,2.0/e1-1.0);
        double t4=pow(abs(t11)/s[0],2.0/e2-1.0);
        double t3=pow(abs(t10)/s[1],2.0/e2-1.0);
        double t2=pow(t7,2.0/e1)+t6;
        double t1=pow(t9+t8,e2/e1-1.0);
        double t0=pow(t2,e1-1.0);

        grad_f[0]+=tmp1 * (-t1*t0*2.0*((sign(t10)*sin(a)*t3)/s[1]-(sign(t11)*cos(a)*t4)/s[0]));
        grad_f[1]+=tmp1 * (t1*t0*2.0*((sign(t11)*sin(a)*t4)/s[0]+(sign(t10)*cos(a)*t3)/s[1]));
        grad_f[2]+=tmp1 * ((sign(c[2]-p[2])*t0*t5*2.0)/s[2]);
        grad_f[3]+=tmp1 * (2.0*t1*t0*((sign(t11)*t4*t10)/s[0]-(sign(t10)*t3*t11)/s[1]));
        grad_f[4]+=tmp1 * (-(abs(t11)*t0*t4*t1*2.0)/(s[0]*s[0])) + tmp2 * s[1]*s[2];
        grad_f[5]+=tmp1 * (-(abs(t10)*t0*t3*t1*2.0)/(s[1]*s[1])) + tmp2 * s[0]*s[2];
        grad_f[6]+=tmp1 * (-(abs(c[2]-p[2])*t0*t5*2.0)/(s[2]*s[2])) + tmp2 * s[0]*s[1];
        grad_f[7]+=tmp1 * ((log(t2)*pow(t2,e1)-t0*(log(t7)*pow(t7,2.0/e1)*2.0+e2*log(t9+t8)*t6)/e1));
        grad_f[8]+=tmp1 * (t0*(log(t9+t8)*t6-2.0*t1*(log(abs(t11)/s[0])*t9+log(abs(t10)/s[1])*t8)/e2));
    }

    for (Ipopt::Index i=0; i<n; i++)
        grad_f[i]/=points.size();
    return true;
}

/****************************************************************/
bool SuperQuadricNLP::eval_g(Ipopt::Index n, const Ipopt::Number *x,
                             bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    g[0]=x[2]-x[6];
    return true;
}

/****************************************************************/
bool SuperQuadricNLP::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x,
                                 bool new_x, Ipopt::Index m, Ipopt::Index nele_jac,
                                 Ipopt::Index *iRow, Ipopt::Index *jCol,
                                 Ipopt::Number *values)
{
    if (values==nullptr)
    {
        iRow[0]=0; jCol[0]=2;
        iRow[1]=0; jCol[1]=6;
    }
    else
    {
        values[0]=1.0;
        values[1]=-1.0;
    }
    return true;
}

/****************************************************************/
bool SuperQuadricNLP::eval_h(Ipopt::Index n, const Ipopt::Number *x,
                             bool new_x, Ipopt::Number obj_factor,
                             Ipopt::Index m, const Ipopt::Number *lambda,
                             bool new_lambda, Ipopt::Index nele_hess,
                             Ipopt::Index *iRow, Ipopt::Index *jCol,
                             Ipopt::Number *values)
{
    return true;
}

/****************************************************************/
void SuperQuadricNLP::finalize_solution(Ipopt::SolverReturn status,
                                        Ipopt::Index n, const Ipopt::Number *x,
                                        const Ipopt::Number *z_L,
                                        const Ipopt::Number *z_U,
                                        Ipopt::Index m, const Ipopt::Number *g,
                                        const Ipopt::Number *lambda,
                                        Ipopt::Number obj_value,
                                        const Ipopt::IpoptData *ip_data,
                                        Ipopt::IpoptCalculatedQuantities *ip_cq)
{
    result.resize(n);
    for (Ipopt::Index i=0; i<n; i++)
        result[i]=x[i];
    result[3]*=180.0/M_PI;
}

/****************************************************************/
SuperQuadricNLP::SuperQuadricNLP(const vector<Vector> &points_,
                                 const double inside_penalty_) : 
                                 points(points_), inside_penalty(inside_penalty_)
{
    bounds.resize(3,2);
    bounds(0,0)=bounds(1,0)=bounds(2,0)=numeric_limits<double>::infinity();
    bounds(0,1)=bounds(1,1)=bounds(2,1)=-numeric_limits<double>::infinity();

    for (auto &p:points)
    {
        if (p[0]<bounds(0,0))
            bounds(0,0)=p[0];
        if (p[0]>bounds(0,1))
            bounds(0,1)=p[0];

        if (p[1]<bounds(1,0))
            bounds(1,0)=p[1];
        if (p[1]>bounds(1,1))
            bounds(1,1)=p[1];

        if (p[2]<bounds(2,0))
            bounds(2,0)=p[2];
        if (p[2]>bounds(2,1))
            bounds(2,1)=p[2];
    }

    centroid.resize(3,0.0);
    for (unsigned int i=0; i<centroid.length(); i++)
        centroid[i]=0.5*(bounds(i,0)+bounds(i,1));
}

/****************************************************************/
Vector SuperQuadricNLP::get_result() const
{
    return result;
}

