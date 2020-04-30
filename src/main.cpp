#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkNamedColors.h>

using namespace std;

void addVTKSphere(pcl::visualization::PCLVisualizer::Ptr viewer_ptr);

void regularPlacement(int num_point, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

int main(int argc, char const *argv[]) {
    int height = 480, width = 640;

    pcl::visualization::PCLVisualizer::Ptr viewer_ptr;
    viewer_ptr.reset(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer_ptr->setSize(width, height);
    viewer_ptr->setBackgroundColor(0, 0, 0);
    addVTKSphere(viewer_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    regularPlacement(500, cloud_ptr);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_ptr, 99, 40, 25);
    viewer_ptr->addPointCloud(cloud_ptr, color, "regular points");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "regular points");
    cout << "Result Regular Points Number: " << endl;
    cout << cloud_ptr->size() << endl;

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

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphere->GetOutputPort());

    vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d("Cornsilk").GetData());

    viewer_ptr->getRendererCollection()->GetFirstRenderer()->AddActor(actor);
}

void regularPlacement(int num_point, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
    float a = 4.0 * M_PI * 1.0 / static_cast<float>(num_point);
    float d = sqrt(a);
    size_t num_phi = round(M_PI / d);
    float d_phi = M_PI / static_cast<float>(num_phi);
    float d_theta = a / d_phi;
    for (int m = 0; m < num_phi; ++m) {
        float phi = M_PI * (m + 0.5) / num_phi;
        size_t num_theta = round(2 * M_PI * sin(phi) / d_theta);
        for (int n = 0; n < num_theta; ++n) {
            float theta = 2 * M_PI * n / static_cast<float>(num_theta);
            pcl::PointXYZ p;
            p.x = sin(phi) * cos(theta);
            p.y = sin(phi) * sin(theta);
            p.z = cos(phi);
            cloud_ptr->push_back(p);
        }
    }
}