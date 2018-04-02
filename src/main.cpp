/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <memory>
#include <cmath>
#include <vector>
#include <set>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkTransform.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkRadiusOutlierRemoval.h>
#include <vtkActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>

#include "superquadric.h"
#include "nlp.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

Mutex mutex;

/****************************************************************/
class UpdateCommand : public vtkCommand
{
    const bool *closing;

public:
    /****************************************************************/
    vtkTypeMacro(UpdateCommand, vtkCommand);

    /****************************************************************/
    static UpdateCommand *New()
    {
        return new UpdateCommand;
    }

    /****************************************************************/
    UpdateCommand() : closing(nullptr) { }

    /****************************************************************/
    void set_closing(const bool &closing)
    {
        this->closing=&closing;
    }

    /****************************************************************/
    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), 
                 void *vtkNotUsed(callData))
    {
        LockGuard lg(mutex);
        vtkRenderWindowInteractor* iren=static_cast<vtkRenderWindowInteractor*>(caller);
        if (closing!=nullptr)
        {
            if (*closing)
            {
                iren->GetRenderWindow()->Finalize();
                iren->TerminateApp();
                return;
            }
        }

        iren->GetRenderWindow()->SetWindowName("Find Superquadric");
        iren->Render();
    }
};


/****************************************************************/
class Object
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkActor> vtk_actor;

public:
    /****************************************************************/
    vtkSmartPointer<vtkActor> &get_actor()
    {
        return vtk_actor;
    }
};


/****************************************************************/
class Points : public Object
{
protected:
    vtkSmartPointer<vtkPoints> vtk_points;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkVertexGlyphFilter> vtk_glyphFilter;

public:
    /****************************************************************/
    Points(const vector<Vector> &points, const int point_size)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

        vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
        vtk_polydata->SetPoints(vtk_points);

        vtk_glyphFilter=vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_glyphFilter->SetInputData(vtk_polydata);
        vtk_glyphFilter->Update();

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_glyphFilter->GetOutputPort());

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetPointSize(point_size);
    }

    /****************************************************************/
    void set_points(const vector<Vector> &points)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

        vtk_polydata->SetPoints(vtk_points);
    }

    /****************************************************************/
    bool set_colors(const vector<vector<unsigned char>> &colors)
    {
        if (colors.size()==vtk_points->GetNumberOfPoints())
        {
            vtk_colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
            vtk_colors->SetNumberOfComponents(3);
            for (size_t i=0; i<colors.size(); i++)
                vtk_colors->InsertNextTypedTuple(colors[i].data());

            vtk_polydata->GetPointData()->SetScalars(vtk_colors);
            return true;
        }
        else
            return false;
    }

    /****************************************************************/
    vtkSmartPointer<vtkPolyData> &get_polydata()
    {
        return vtk_polydata;
    }
};


/****************************************************************/
class Superquadric : public Object
{
protected:
    vtkSmartPointer<vtkSuperquadricStd> vtk_superquadric;
    vtkSmartPointer<vtkSampleFunction> vtk_sample;
    vtkSmartPointer<vtkContourFilter> vtk_contours;
    vtkSmartPointer<vtkTransform> vtk_transform;

public:
    /****************************************************************/
    Superquadric(const Vector &r)
    {
        double bx=2.0*r[4];
        double by=2.0*r[5];
        double bz=2.0*r[6];

        vtk_superquadric=vtkSmartPointer<vtkSuperquadricStd>::New();
        vtk_superquadric->SetScale(r[4],r[5],r[6]);
        vtk_superquadric->SetPhiRoundness(r[7]);
        vtk_superquadric->SetThetaRoundness(r[8]);

        vtk_sample=vtkSmartPointer<vtkSampleFunction>::New();
        vtk_sample->SetSampleDimensions(50,50,50);
        vtk_sample->SetImplicitFunction(vtk_superquadric);
        vtk_sample->SetModelBounds(-bx,bx,-by,by,-bz,bz);

        vtk_contours=vtkSmartPointer<vtkContourFilter>::New();
        vtk_contours->SetInputConnection(vtk_sample->GetOutputPort());
        vtk_contours->GenerateValues(1,1.0,1.0);

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_contours->GetOutputPort());
        vtk_mapper->SetScalarRange(0.0,1.2);

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetOpacity(0.25);

        vtk_transform=vtkSmartPointer<vtkTransform>::New();
        vtk_transform->Translate(r.subVector(0,2).data());
        vtk_transform->RotateZ(r[3]);
        vtk_actor->SetUserTransform(vtk_transform);
    }

    /****************************************************************/
    void set_parameters(const Vector &r)
    {
        double bx=2.0*r[4];
        double by=2.0*r[5];
        double bz=2.0*r[6];

        vtk_superquadric->SetScale(r[4],r[5],r[6]);
        vtk_superquadric->SetPhiRoundness(r[10]);
        vtk_superquadric->SetThetaRoundness(r[11]);
        
        vtk_sample->SetModelBounds(-bx,bx,-by,by,-bz,bz);

        vtk_transform->Identity();
        vtk_transform->Translate(r.subVector(0,2).data());
        vtk_transform->RotateZ(r[3]);
    }
};


/****************************************************************/
class Finder : public RFModule
{
    Bottle outliersRemovalOptions;
    unsigned int uniform_sample;
    double random_sample;
    bool test_derivative;
    double inside_penalty;
    bool closing;

    class PointsProcessor : public PortReader {
        Finder *finder;
        bool read(ConnectionReader &connection) override {
            Matrix points;
            if (!points.read(connection))
                return false;
            Bottle reply;
            finder->process(points,reply);
            if (ConnectionWriter *writer=connection.getWriter())
                reply.write(*writer);
            return true;
        }
    public:
        PointsProcessor(Finder *finder_) : finder(finder_) { }
    } pointsProcessor;

    RpcServer rpcPoints,rpcService;

    vector<Vector> all_points,in_points,out_points,dwn_points;
    vector<vector<unsigned char>> all_colors;
    
    unique_ptr<Points> vtk_all_points,vtk_out_points,vtk_dwn_points;
    unique_ptr<Superquadric> vtk_superquadric;

    vtkSmartPointer<vtkRenderer> vtk_renderer;
    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

    /****************************************************************/
    void removeOutliers()
    {
        if (outliersRemovalOptions.size()>=2)
        {
            double radius=outliersRemovalOptions.get(0).asDouble();
            int neighbors=outliersRemovalOptions.get(1).asInt();

            vtkSmartPointer<vtkPoints> vtk_points=vtkSmartPointer<vtkPoints>::New();
            for (size_t i=0; i<all_points.size(); i++)
                vtk_points->InsertNextPoint(all_points[i][0],all_points[i][1],all_points[i][2]);

            vtkSmartPointer<vtkPolyData> vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
            vtk_polydata->SetPoints(vtk_points);

            vtkSmartPointer<vtkRadiusOutlierRemoval> removal=vtkSmartPointer<vtkRadiusOutlierRemoval>::New();
            removal->SetInputData(vtk_polydata);
            removal->SetRadius(radius);
            removal->SetNumberOfNeighbors(neighbors);
            removal->Update();

            yInfo()<<"# of outliers removed / # of points ="
                   <<removal->GetNumberOfPointsRemoved()<<"/"<<all_points.size();

            for (size_t i=0; i<all_points.size(); i++)
            {
                if (removal->GetPointMap()[i]<0)
                    out_points.push_back(all_points[i]);
                else
                    in_points.push_back(all_points[i]);
            }
        }
        else
            in_points=all_points;
    }

    /****************************************************************/
    void sampleInliers()
    {
        if (random_sample>=1.0)
        {
            unsigned int cnt=0;
            for (auto &p:in_points)
            {
                if ((cnt++%uniform_sample)==0)
                    dwn_points.push_back(p);
            }
        }
        else
        {
            set<unsigned int> idx;
            while (idx.size()<(size_t)(random_sample*in_points.size()))
            {
                unsigned int i=(unsigned int)(Rand::scalar(0.0,1.0)*in_points.size());
                if (idx.find(i)==idx.end())
                {
                    dwn_points.push_back(in_points[i]);
                    idx.insert(i);
                }
            }
        }
    }

    /****************************************************************/
    Vector findSuperquadric() const
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",1e-6);
        app->Options()->SetIntegerValue("acceptable_iter",0);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetIntegerValue("max_iter",1000);
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetStringValue("derivative_test",test_derivative?"first-order":"none");
        app->Options()->SetIntegerValue("print_level",test_derivative?5:0);
        app->Initialize();

        double t0=Time::now();
        Ipopt::SmartPtr<SuperQuadricNLP> nlp=new SuperQuadricNLP(dwn_points,inside_penalty);
        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
        double t1=Time::now();

        Vector r=nlp->get_result();
        yInfo()<<"center   = ("<<r.subVector(0,2).toString(3,3)<<")";
        yInfo()<<"angle    ="<<r[3]<<"[deg]";
        yInfo()<<"size     = ("<<r.subVector(4,6).toString(3,3)<<")";
        yInfo()<<"shape    = ("<<r.subVector(7,8).toString(3,3)<<")";
        yInfo()<<"found in ="<<t1-t0<<"[s]";

        return r;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        Rand::init();

        if (rf.check("file"))
        {
            string file=rf.find("file").asString();
            ifstream fin(file.c_str());
            if (!fin.is_open())
            {
                yError()<<"Unable to open file \""<<file<<"\"";
                return false;
            }

            Vector p(3);
            vector<unsigned int> c_(3);
            vector<unsigned char> c(3);

            string line;
            while (getline(fin,line))
            {
                istringstream iss(line);
                if (!(iss>>p[0]>>p[1]>>p[2]))
                    break;
                all_points.push_back(p);

                fill(c_.begin(),c_.end(),120);
                iss>>c_[0]>>c_[1]>>c_[2];
                c[0]=(unsigned char)c_[0];
                c[1]=(unsigned char)c_[1];
                c[2]=(unsigned char)c_[2];
                all_colors.push_back(c);
            }
        }
        else
        {
            rpcPoints.open("/find-superquadric/points:rpc");
            rpcPoints.setReader(pointsProcessor);

            rpcService.open("/find-superquadric/service:rpc");
            attach(rpcService);
        }

        if (rf.check("remove-outliers"))
            if (const Bottle *ptr=rf.find("remove-outliers").asList())
                outliersRemovalOptions=*ptr;

        uniform_sample=(unsigned int)rf.check("uniform-sample",Value(1)).asInt();
        random_sample=rf.check("random-sample",Value(1.0)).asDouble();
        inside_penalty=rf.check("inside-penalty",Value(100.0)).asDouble();
        test_derivative=rf.check("test-derivative");

        removeOutliers();
        sampleInliers();

        vtk_all_points=unique_ptr<Points>(new Points(all_points,2));
        vtk_out_points=unique_ptr<Points>(new Points(out_points,4));
        vtk_dwn_points=unique_ptr<Points>(new Points(dwn_points,1));

        vtk_all_points->set_colors(all_colors);
        vtk_out_points->get_actor()->GetProperty()->SetColor(1.0,0.0,0.0);
        vtk_dwn_points->get_actor()->GetProperty()->SetColor(1.0,1.0,0.0);

        Vector r=findSuperquadric();
        vtk_superquadric=unique_ptr<Superquadric>(new Superquadric(r));

        vtk_renderer=vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow=vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetSize(600,600);
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);

        vtk_renderer->AddActor(vtk_all_points->get_actor());
        vtk_renderer->AddActor(vtk_out_points->get_actor());
        if (dwn_points.size()!=in_points.size())
            vtk_renderer->AddActor(vtk_dwn_points->get_actor());
        vtk_renderer->AddActor(vtk_superquadric->get_actor());
        vtk_renderer->SetBackground(0.1,0.2,0.2);

        vtk_axes=vtkSmartPointer<vtkAxesActor>::New();     
        vtk_widget=vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        vtk_widget->SetOutlineColor(0.9300,0.5700,0.1300);
        vtk_widget->SetOrientationMarker(vtk_axes);
        vtk_widget->SetInteractor(vtk_renderWindowInteractor);
        vtk_widget->SetViewport(0.0,0.0,0.2,0.2);
        vtk_widget->SetEnabled(1);
        vtk_widget->InteractiveOn();

        vector<double> bounds(6),centroid(3);
        vtk_all_points->get_polydata()->GetBounds(bounds.data());
        for (size_t i=0; i<centroid.size(); i++)
            centroid[i]=0.5*(bounds[i<<1]+bounds[(i<<1)+1]);

        vtk_camera=vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(centroid[0]+1.0,centroid[1],centroid[2]+0.5);
        vtk_camera->SetFocalPoint(centroid.data());
        vtk_camera->SetViewUp(0.0,0.0,1.0);
        vtk_renderer->SetActiveCamera(vtk_camera);

        vtk_style=vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);

        vtk_renderWindowInteractor->Initialize();
        vtk_renderWindowInteractor->CreateRepeatingTimer(1);

        vtk_updateCallback=vtkSmartPointer<UpdateCommand>::New();
        vtk_updateCallback->set_closing(closing);
        vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent,vtk_updateCallback);
        vtk_renderWindowInteractor->Start();
        
        return true;
    }

    /****************************************************************/
    bool updateModule() override
    {
        return false;
    }

    /****************************************************************/
    void process(const Matrix &points, Bottle &reply)
    {   
        reply.clear();
        if (points.rows()>0)
        {
            LockGuard lg(mutex);

            all_points.clear();
            all_colors.clear();
            in_points.clear();
            out_points.clear();
            dwn_points.clear();

            Vector p(3);
            vector<unsigned char> c(3);
            for (int r=0; r<points.rows(); r++)
            {
                p[0]=points(r,0);
                p[1]=points(r,1);
                p[2]=points(r,2);
                c[0]=(unsigned char)points(r,3);
                c[1]=(unsigned char)points(r,4);
                c[2]=(unsigned char)points(r,5);
                all_points.push_back(p);
                all_colors.push_back(c);
            }

            removeOutliers();
            sampleInliers();
            Vector r=findSuperquadric();
            
            vtk_all_points->set_points(all_points);
            vtk_all_points->set_colors(all_colors);
            vtk_out_points->set_points(out_points);
            vtk_dwn_points->set_points(dwn_points);
            vtk_superquadric->set_parameters(r);

            reply.read(r);
        }
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        LockGuard lg(mutex);

        bool ok=false;
        if (command.check("remove-outliers"))
        {
            if (const Bottle *ptr=command.find("remove-outliers").asList())
                outliersRemovalOptions=*ptr;
            ok=true;
        }

        if (command.check("uniform-sample"))
        {
            uniform_sample=(unsigned int)command.find("uniform-sample").asInt();
            ok=true;
        }

        if (command.check("random-sample"))
        {
            random_sample=command.find("random-sample").asDouble();
            ok=true;
        }

        if (command.check("inside-penalty"))
        {
            inside_penalty=command.find("inside-penalty").asDouble();
            ok=true;
        }

        reply.addVocab(Vocab::encode(ok?"ack":"nack"));
        return true;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        closing=true;
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        if (rpcPoints.asPort().isOpen())
            rpcPoints.close();
        if (rpcService.asPort().isOpen())
            rpcService.close();
        return true;
    }

public:
    /****************************************************************/
    Finder() : closing(false), pointsProcessor(this) { }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    ResourceFinder rf;
    rf.configure(argc,argv);

    if (!rf.check("file"))
    {
        if (!yarp.checkNetwork())
        {
            yError()<<"Unable to find Yarp server!";
            return EXIT_FAILURE;
        }
    }

    Finder finder;
    return finder.runModule(rf);
}

