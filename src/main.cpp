#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkNamedColors.h>

using namespace std;

void addVTKSphere(pcl::visualization::PCLVisualizer::Ptr viewer_ptr);

int main(int argc, char const *argv[]) {
    int height = 480, width = 640;

    pcl::visualization::PCLVisualizer::Ptr viewer_ptr;
    viewer_ptr.reset(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer_ptr->setSize(width, height);
    viewer_ptr->setBackgroundColor(0, 0, 0);
    addVTKSphere(viewer_ptr);
    viewer_ptr->resetCamera();

    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce();
    }
    return 0;
}

// Add sphere to scene
void addVTKSphere(pcl::visualization::PCLVisualizer::Ptr viewer_ptr) {
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(0.0, 0.0, 0.0);
    sphere->SetRadius(1.0);
    sphere->SetPhiResolution(100);
    sphere->SetThetaResolution(100);

    vtkSmartPointer <vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    mapper->SetInputConnection (sphere->GetOutputPort ());

    vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
    actor->SetMapper (mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d("Cornsilk").GetData());

    viewer_ptr->getRendererCollection()->GetFirstRenderer()->AddActor(actor);
}
