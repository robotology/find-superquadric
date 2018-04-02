/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file superquadric.h
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#ifndef SUPERQUADRIC_H
#define SUPERQUADRIC_H

#include <vtkImplicitFunction.h>

/****************************************************************/
class vtkSuperquadricStd : public vtkImplicitFunction
{
public:
    using vtkImplicitFunction::EvaluateFunction;
    
    /****************************************************************/
    static vtkSuperquadricStd *New();

    /****************************************************************/
    vtkTypeMacro(vtkSuperquadricStd, vtkImplicitFunction);

    /****************************************************************/
    double EvaluateFunction(double x[3]) override;

    /****************************************************************/
    void EvaluateGradient(double x[3], double g[3]) override;

    /****************************************************************/
    void SetScale(double sx, double sy, double sz);

    /****************************************************************/
    void SetThetaRoundness(double e);

    /****************************************************************/
    void SetPhiRoundness(double e);

protected:
    /****************************************************************/
    vtkSuperquadricStd();

    /****************************************************************/
    ~vtkSuperquadricStd() override { }

    double PhiRoundness;
    double ThetaRoundness;
    double Scale[3];

private:
    vtkSuperquadricStd(const vtkSuperquadricStd&) = delete;
    void operator=(const vtkSuperquadricStd&) = delete;
};

#endif

