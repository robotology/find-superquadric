/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file superquadric.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cmath>

#include <vtkObjectFactory.h>

#include "superquadric.h"

vtkStandardNewMacro(vtkSuperquadricStd);

/****************************************************************/
vtkSuperquadricStd::vtkSuperquadricStd()
{
    Scale[0]=Scale[1]=Scale[2]=1.0;

    PhiRoundness=0.0;
    SetPhiRoundness(1.0);

    ThetaRoundness=0.0;
    SetThetaRoundness(1.0);
}

/****************************************************************/
void vtkSuperquadricStd::SetScale(double sx, double sy, double sz)
{
    Scale[0]=sx;
    Scale[1]=sy;
    Scale[2]=sz;
    Modified();
}

/****************************************************************/
void vtkSuperquadricStd::SetThetaRoundness(double e)
{
    ThetaRoundness=e;
    Modified();
}

/****************************************************************/
void vtkSuperquadricStd::SetPhiRoundness(double e)
{
    PhiRoundness=e;
    Modified();
}

/****************************************************************/
double vtkSuperquadricStd::EvaluateFunction(double xyz[3])
{
    double p[3];
    p[0]=xyz[0]/Scale[0];
    p[1]=xyz[1]/Scale[1];
    p[2]=xyz[2]/Scale[2];

    double e=ThetaRoundness;
    double n=PhiRoundness;
    double val=pow((pow(abs(p[0]),2.0/e)+pow(abs(p[1]),2.0/e)),e/n)+
               pow(abs(p[2]),2.0/n)-1.0;

    return val;
}

/****************************************************************/
void vtkSuperquadricStd::EvaluateGradient(double vtkNotUsed(xyz)[3],
                                          double g[3])
{
    g[0]=g[1]=g[2]=0.0;
}

