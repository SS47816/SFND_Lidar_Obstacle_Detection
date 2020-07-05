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
    auto lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(lidar->scan());
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "inputCloud", Color(1, 1,1));

    // TODO:: Create point processor
    auto pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    // Segment the groud plane and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Cluster objects
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 1), Color(1, 1, 0), Color(0, 0, 1)};

    // Render the clusters
    for (auto cluster : cloudClusters)
    {
        // Render Clusters
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        
        // Render Bounding Boxes
        // Box box = pointProcessor->BoundingBox(cluster);
        // renderBox(viewer, box, clusterId);
        BoxQ box = pointProcessor->MinimumBoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, std::vector<Box>& preBoxes)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    // Filter the inputCloud
    float max_length = 30.0;
    float max_width = 6.0;
    float pos_height = 1.0;
    float neg_height = 2.0;
    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-max_length, -max_width, -neg_height, 1), Eigen::Vector4f(max_length, max_width, pos_height, 1));
    // renderPointCloud(viewer, filterCloud, "filterCloud");
    
    // Segment the filtered cloud into two parts, road and obstacles.
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->CustomizedSegmentPlane(filterCloud, 20, 0.4);
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0.1, 1, 0.1));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->CustomizedClustering(segmentCloud.first, 0.4, 30, 2000);
    
    // Render the clusters
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0.1, 0.2), Color(1, 1, 0.3), Color(0.1, 0.6, 1)};
    std::vector<Box> curBoxes;
    for (auto cluster : cloudClusters)
    {
        // Print cluster size
        std::cout << "cluster size "; pointProcessorI->numPoints(cluster);
        // Render Clusters
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);

        curBoxes.emplace_back(pointProcessorI->BoundingBox(cluster, clusterId, clusterId%colors.size()));
        
        ++clusterId;
    }

    // Tracking
    std::vector<int> matches;
    if (!preBoxes.empty() && !curBoxes.empty())
    {
        // vectors containing the id of boxes in left and right sets
        std::vector<int> left;
        std::vector<int> right;
        auto connectionPairs = pointProcessorI->associateBoxes(preBoxes, curBoxes, 0.5, 0.5);
        auto connectionMatrix = pointProcessorI->connectionMatrix(connectionPairs, left, right);
        matches = pointProcessorI->hungarian(connectionMatrix);

        for (int j = 0; j < matches.size(); ++j)
        {
            // find the index of the current box that needs to be changed
            const auto cur_id = right[j]; // right and matches has the same size
            const auto cur_index = cur_id; // for a current box, its id is actually the same as its index in the vector
            
            // find the index of the previous box that the current box corresponds to
            const auto index = matches[j];
            const auto pre_id = left[index];
            for (int i = 0; i < preBoxes.size(); ++i)
            {
                if (cur_id == preBoxes[i].id) curBoxes[cur_index].color = preBoxes[i].color; // change the color of the current box to the same as the previous box
            }
        }
    }

    // Render boxes
    for (auto& box : curBoxes)
    {
        // Render Bounding Boxes
        renderBox(viewer, box, box.id, colors[box.color], 0.3);
    }

    preBoxes.clear();
    preBoxes = curBoxes;
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    std::vector<Box> preBoxes;
    
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI, preBoxes);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }

}