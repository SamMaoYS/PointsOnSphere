#include <iostream>
#include <random>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkNamedColors.h>

using namespace std;

void genVTKSphere(vtkSmartPointer<vtkLODActor> &actor, const float radius);

void regularPlacement(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, const float radius, int num_point = 300);

void randomPlacement(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, const float radius, int num_point = 300);

void addActorToRenderer(const vtkSmartPointer<vtkLODActor> &actor, pcl::visualization::PCLVisualizer::Ptr viewer_ptr,
                        int view_port);

int main(int argc, char const *argv[]) {
    int height = 480, width = 1280;
    float radius = 300;
    int num_points = 2000;

    pcl::visualization::PCLVisualizer::Ptr viewer_ptr;
    viewer_ptr.reset(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer_ptr->setSize(width, height);

    // Generate a sphere
    vtkSmartPointer<vtkLODActor> sphere_actor;
    genVTKSphere(sphere_actor, radius);

    // View Port 1 for visualize regular placement
    int v1(0);
    viewer_ptr->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer_ptr->setBackgroundColor(0, 0, 0, v1);
    addActorToRenderer(sphere_actor, viewer_ptr, v1);


    pcl::PointCloud<pcl::PointXYZ>::Ptr regular_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    regularPlacement(regular_cloud_ptr, radius, num_points);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> regular_color(regular_cloud_ptr, 99, 40, 25);
    viewer_ptr->addPointCloud(regular_cloud_ptr, regular_color, "regular points", v1);
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "regular points");

    cout << "Result Regular Points Number: " << endl;
    cout << regular_cloud_ptr->size() << endl;

    // View Port 2 for visualize random placement
    int v2(0);
    viewer_ptr->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer_ptr->setBackgroundColor(0.05, 0.05, 0.05, v2);
    addActorToRenderer(sphere_actor, viewer_ptr, v2);


    pcl::PointCloud<pcl::PointXYZ>::Ptr random_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    randomPlacement(random_cloud_ptr, radius, num_points);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> random_color(random_cloud_ptr, 99, 40, 25);
    viewer_ptr->addPointCloud(random_cloud_ptr, random_color, "random points", v2);
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "random points");

    cout << "Result Random Points Number: " << endl;
    cout << random_cloud_ptr->size() << endl;

    viewer_ptr->resetCamera();
    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce();
    }
    return 0;
}

// Add sphere to scene
void genVTKSphere(vtkSmartPointer<vtkLODActor> &actor, const float radius) {
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(0.0, 0.0, 0.0);
    sphere->SetRadius(static_cast<double>(radius));
    sphere->SetPhiResolution(100);
    sphere->SetThetaResolution(100);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphere->GetOutputPort());

    actor = vtkSmartPointer<vtkLODActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d("Cornsilk").GetData());
}

void regularPlacement(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, const float radius, int num_point) {
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
            p.x = radius * sin(phi) * cos(theta);
            p.y = radius * sin(phi) * sin(theta);
            p.z = radius * cos(phi);
            cloud_ptr->push_back(p);
        }
    }
}

void randomPlacement(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, const float radius, int num_point) {
    for (int n = 0; n < num_point; ++n) {
        random_device rand_dev;
        mt19937 generator(rand_dev());
        uniform_real_distribution<float> distribution(-1, 1);

        float z = radius * distribution(generator);
        float theta = static_cast<float>(M_PI) * distribution(generator) + static_cast<float>(M_PI);
        float x = sqrt(radius * radius - z * z) * cos(theta);
        float y = sqrt(radius * radius - z * z) * sin(theta);

        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = z;
        cloud_ptr->push_back(p);
    }
}

void addActorToRenderer(const vtkSmartPointer<vtkLODActor> &actor, pcl::visualization::PCLVisualizer::Ptr viewer_ptr,
                        int view_port) {
    vtkSmartPointer<vtkRendererCollection> rens = viewer_ptr->getRendererCollection();
    rens->InitTraversal();
    vtkSmartPointer<vtkRenderer> renderer = nullptr;
    size_t i = 0;
    while ((renderer = rens->GetNextItem())) {
        if (view_port == 0)
            renderer->AddActor(actor);
        else if (view_port == i)
            renderer->AddActor(actor);
        i++;
    }
}