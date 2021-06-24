/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderPointCloud(viewer,inputCloud,"InputCloud");
    //renderRays(viewer,lidar->position,inputCloud);

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* processor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult =  processor->SegmentPlane(inputCloud,100,0.2);
    //renderPointCloud(viewer,segResult.first,"ObstacleCloud",Color(1,0,0)); //rgb
    //renderPointCloud(viewer,segResult.second,"roadCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = processor->Clustering(segResult.first,1.5,3,30);
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(int clusterID = 0; clusterID < clusters.size(); clusterID++){

        processor->numPoints(clusters[clusterID]);
        renderPointCloud(viewer,clusters[clusterID],"ClusterCloud_"+std::to_string(clusterID),colors[clusterID]);
        renderBox(viewer, processor->BoundingBox(clusters[clusterID]), clusterID, colors[clusterID]);
    }
}

/**
 * Similar function to simpleHighway
 * Load real pcd data
 * Filter data
 * Apply Segmentation
 * Find clusters
 * Draw bounding box
 */
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *processorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){

    //0.25m
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = processorI->FilterCloud(inputCloud,0.25,Eigen::Vector4f (-30,-5, -3,1.0) , Eigen::Vector4f (25,8,3,1.0));
    renderPointCloud(viewer,filtered_cloud,"FilteredCloud");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult =  processorI->SegmentPlane(filtered_cloud,500,0.4);
    //renderPointCloud(viewer,segResult.first,"ObstacleCloud",Color(1,0,0)); //rgb
    //renderPointCloud(viewer,segResult.second,"RoadCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = processorI->Clustering(segResult.first,0.6,10,500);
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(int clusterID = 0; clusterID < clusters.size(); clusterID++){

        processorI->numPoints(clusters[clusterID]);
        //renderPointCloud(viewer,clusters[clusterID],"ClusterCloud_"+std::to_string(clusterID));
        renderBox(viewer, processorI->BoundingBox(clusters[clusterID]), clusterID, colors[clusterID % colors.size()]);
    }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

/***
 * This is the main function which initialize the camera parameters and initialize the simulated environment of simple highway
 * @param argc
 * @param argv
 * @return
 */
int main (int argc, char** argv){

    std::cout << "Starting environment..." << std::endl;

    ProcessPointClouds<pcl::PointXYZI> *processorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = processorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloud = processorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, processorI, inputCloud);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}