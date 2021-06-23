/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = i+scatter*rx;
        point.y = i+scatter*ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = 5*rx;
        point.y = 5*ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIteration, float distanceTol){

    srand(time(NULL));
    std::unordered_set<int> inliers;

    int cloud_size = cloud->size();
    while(maxIteration --){

        std::unordered_set<int> result;
        /**
         * For each iteration, select 2 points randomly.
         * Point1 is represented as [x1,y1]
         * Point2 is represented as [x2,y2]
         * */
        pcl::PointXYZ point1 = cloud->points[rand()% cloud_size]; //range = [0,cloud_zize)
        pcl::PointXYZ point2 = cloud->points[rand()% cloud_size]; //range = [0,cloud_zize)

        /**
         * Calculate the model coefficients
         * Line is represented as Ax + By + C = 0
         */
        float A = point1.y - point2.y;
        float B = point2.x - point1.x;
        float C = (point1.x * point2.y) - (point2.x * point1.y);

        /**
         * For each point, find the distance from above line
         */
        for(int index = 0 ; index < cloud_size; index++){

            float numerator = fabs((A * cloud->points[index].x) +(B * cloud->points[index].y)+ C);
            float denominator = sqrt(pow(A, 2) + pow(B, 2));
            float distance =  numerator/denominator;

            if(distance <= distanceTol){
                result.insert(index);
            }
        }

        if(result.size() > inliers.size()){
            inliers.clear();
            inliers = result;
        }
    }

    return inliers;
}

/**
 * Later on, copy and paste this function in SegmentPlane method of ProcessPointCloud.cpp
 * By default, method is SAC_RANSAC and model is Plane
 * Further, pass this result to SeparateCloud method of ProcessPointCloud.cpp
*/
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIteration, float distanceTol){

    srand(time(NULL));
    std::unordered_set<int> inliers;

    int cloud_size = cloud->size();
    while(maxIteration-- ){

        std::unordered_set<int> result;

        /**
         * Select 3 points randomly,
         * Find the model coefficient Ax + By + Cz + D = 0
         * Point1 is represented as [x1,y1,z1]
         * Point2 is represented as [x2,y2,z2]
         * Point3 is represented as [x3,y3,z3]
         */
        pcl::PointXYZ point1 = cloud->points[rand()%cloud_size];
        pcl::PointXYZ point2 = cloud->points[rand()%cloud_size];
        pcl::PointXYZ point3 = cloud->points[rand()%cloud_size];

        /**
         * Find the normal vector v = v1 X v2
         */
        pcl::PointXYZ normal_vector(((point2.y - point1.y) * (point3.z - point1.z)) - ((point2.z - point1.z) * (point3.y - point1.y)),
                                    ((point2.z - point1.z) * (point3.x - point1.x)) - ((point2.x - point1.x) * (point3.z - point1.z)),
                                    ((point2.x - point1.x) * (point3.y - point1.y)) - ((point2.y - point1.y) * (point3.x - point1.x)));

        /**
         * Calculate the model coefficients
         */
        float A = normal_vector.x;
        float B = normal_vector.y;
        float C = normal_vector.z;
        float D = -(A * point1.x) - (B * point1.y) - (C * point1.z);

        /**
         * For each point, calculate the distance from plane
        */
        for(int index = 0; index <cloud_size; index++){

            float numerator = fabs((A * cloud->points[index].x) +(B * cloud->points[index].y) +(C * cloud->points[index].z) +D);
            float denominator = sqrt(pow(A,2) +pow(B,2) +pow(C,2));
            float distance = numerator/denominator;

            if(distance <= distanceTol){
                result.insert(index);
            }
        }

        if(result.size() > inliers.size()){
            inliers.clear();
            inliers = result;
        }
    }
    return inliers;
}

int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = RansacLine(cloud, 50, 0.5);
    //std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
        renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
    }
    else
    {
        renderPointCloud(viewer,cloud,"data");
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}
