// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    
    // Create the filtering object: downsample the dataset using a leaf size of  0.2m
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Cropping the ROI
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // Removing the car roof (hand-crafted region)
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (auto point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    // Pushback all the inliers into the planeCloud
    for (int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Extract the points that are not in the inliers to obstCloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty())
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (auto getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (auto index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster, int id, int colorId)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.id = id;
    box.color = colorId;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::MinimumBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.
    // TODO: Limit the PCA to the grond plane
    // eigenVectorsPCA.row(0).col(2) << 1.0f;
    // eigenVectorsPCA.row(2).col(0) << 1.0f;
    // // eigenVectorsPCA.row(2).col(2) << 0.0f;
    // eigenVectorsPCA.row(2).col(1) << 1.0f;
    // eigenVectorsPCA.row(1).col(2) << 1.0f;

    // Print Matrix
    std::cout << "eigenVectorsPCA: " << std::endl;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::cout << eigenVectorsPCA.row(i).col(j) << ", ";
        }
        std::cout << std::endl;
    }
        

    /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
    // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
    */

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // Form the box
    BoxQ box;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;
    box.bboxQuaternion = bboxQuaternion;
    box.bboxTransform = bboxTransform;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

// ########################## Start of Project Code ##################################
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	while (maxIterations--)
	{
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto iter = inliers.begin();
		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		z1 = cloud->points[*iter].z;
	
		iter++;
		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;
		z2 = cloud->points[*iter].z;

		iter++;
		x3 = cloud->points[*iter].x;
		y3 = cloud->points[*iter].y;
		z3 = cloud->points[*iter].z;

		float A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		float B = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		float C = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		float D = -(A*x1 + B*y1 + C*z1);

		// Measure distance between every point and fitted plane
		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0) 
				continue;

			PointT point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;
			float z3 = point.z;

			// If distance is smaller than threshold count it as inlier
			float d = fabs(A*x3 + B*y3 + C*z3 + D)/sqrt(A*A + B*B + C*C);
			if (d <= distanceTol)
				inliers.insert(index);
		}

		// Return indicies of inliers from fitted plane with most inliers
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomizedSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers = Ransac3D(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new typename pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new typename pcl::PointCloud<PointT>());
    
    if (!inliers.empty())
    {
        for(int index = 0; index < cloud->points.size(); index++)
        {
            PointT point = cloud->points[index];
            if(inliers.count(index))
                cloudInliers->points.push_back(point);
            else
                cloudOutliers->points.push_back(point); 
        }
    }
    else
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr points, std::vector<int>& clusterIndices, std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTol)
{
	processed[indice] = true;
	clusterIndices.push_back(indice);

	std::vector<int> nearest = tree->search(points->points[indice], distanceTol);

	for (int id : nearest)
	{
		if (!processed[id])
			clusterHelper(id, points, clusterIndices, processed, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	std::vector<bool> processed(cloud->points.size(), false);
	
	int i = 0;
	while (i < cloud->points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}
		
		std::vector<int> clusterIndices;
		clusterHelper(i, cloud, clusterIndices, processed, tree, distanceTol);
        
        typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
        for (auto index : clusterIndices)
            cluster->points.push_back(cloud->points[index]);
        
        if (cluster->points.size() >= minSize && cluster->points.size() <= maxSize)
        {
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }
        
		i++;
	}

	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomizedClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Create a KD Tree and insert the points
    KdTree<PointT>* tree = new KdTree<PointT>;
    for (int i = 0; i < cloud->points.size(); i++) 
    	tree->insert(cloud->points[i], i); 

  	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);

  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    return clusters;
}

// ************************* Tracking ***************************
template<typename PointT>
std::vector<float> ProcessPointClouds<PointT>::getCentroid(const Box& a)
{
    return {(a.x_max + a.x_min) / 2, (a.y_max + a.y_min) / 2, (a.z_max + a.z_min) / 2};
}

template<typename PointT>
std::vector<float> ProcessPointClouds<PointT>::getDimension(const Box& a)
{
    return {a.x_max - a.x_min, a.y_max - a.y_min, a.z_max - a.z_min};
}

template<typename PointT>
float ProcessPointClouds<PointT>::getVolume(const Box& a)
{
    return (a.x_max - a.x_min)*(a.y_max - a.y_min)*(a.z_max - a.z_min);
}

template<typename PointT>
bool ProcessPointClouds<PointT>::compareBoxes(const Box& a, const Box& b, float& displacementTol, float& volumeTol)
{
    const std::vector<float> a_ctr = this->getCentroid(a);
    const std::vector<float> b_ctr = this->getCentroid(b);
    const std::vector<float> a_dim = this->getDimension(a);
    const std::vector<float> b_dim = this->getDimension(b);

    // Percetage Displacements ranging between [0.0, 1.0]
    const float x_dis = fabs(a_ctr[0] - b_ctr[0]) / (a_dim[0] + b_dim[0]) * 2;
    const float y_dis = fabs(a_ctr[1] - b_ctr[1]) / (a_dim[1] + b_dim[1]) * 2;
    const float z_dis = fabs(a_ctr[2] - b_ctr[2]) / (a_dim[2] + b_dim[2]) * 2;

    // Volume similiarity value between [0.0, 1.0]
    float vlm_diff = fabs(this->getVolume(a) - this->getVolume(b)) / (this->getVolume(a) + this->getVolume(b)) * 2;

    if (x_dis <= displacementTol && y_dis <= displacementTol && z_dis <= displacementTol && vlm_diff <= volumeTol)
    {
        return true;
    }
    else
    {
        return false;
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::associateBoxes(const std::vector<Box>& preBoxes, const std::vector<Box>& curBoxes, float displacementTol, float volumeTol)
{
    std::vector<std::vector<int>> connectionPairs;

    for (auto& curBox : curBoxes)
    {
        for (auto& preBox : preBoxes)
        {
            // Add the indecies of a pair of similiar boxes to the matrix
            if (this->compareBoxes(curBox, preBox, displacementTol, volumeTol))
            {
                connectionPairs.push_back({preBox.id, curBox.id});
            }
        }
    }

    return connectionPairs;
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::connectionMatrix(const std::vector<std::vector<int>>& connectionPairs, std::vector<int>& left, std::vector<int>& right)
{
    // Hash the box ids in the connectionPairs to two vectors(sets), left and right
    for (auto& pair : connectionPairs)
    {
        bool left_found = false;
        for (auto i : left)
        {
            if (i == pair[0]) left_found = true;
        }
        if (!left_found) left.push_back(pair[0]);

        bool right_found = false;
        for (auto j : right)
        {
            if (j == pair[1]) right_found = true;
        }
        if (!right_found) right.push_back(pair[1]);

    }

    std::vector<std::vector<int>> connectionMatrix(left.size(), std::vector<int>(right.size(), 0));

    for (auto& pair : connectionPairs)
    {
        int left_index = -1;
        for (int i = 0; i < left.size(); ++i)
        {
            if (pair[0] == left[i]) left_index = i;
        }

        int right_index = -1;
        for (int i = 0; i < right.size(); ++i)
        {
            if (pair[1] == right[i]) right_index = i;
        }

        connectionMatrix[left_index][right_index] = 1;
    }
    
    return connectionMatrix;
}

template<typename PointT>
bool ProcessPointClouds<PointT>::hungarianFind(const int i, const std::vector<std::vector<int>>& connectionMatrix, std::vector<bool>& right_connected, std::vector<int>& right_pair)
{
    for (int j = 0; j < connectionMatrix[0].size(); ++j)
    {
        if (connectionMatrix[i][j] == 1 && right_connected[j] == false)
        {
            right_connected[j] = true;

            if (right_pair[j] == -1 || hungarianFind(right_pair[j], connectionMatrix, right_connected, right_pair))
            {
                right_pair[j] = i;
                return true;
            }
        }
    }
}

template<typename PointT>
std::vector<int> ProcessPointClouds<PointT>::hungarian(const std::vector<std::vector<int>>& connectionMatrix)
{
    std::vector<bool> right_connected(connectionMatrix[0].size(), false);
    std::vector<int> right_pair(connectionMatrix[0].size(), -1);

    int count = 0;
    for (int i = 0; i < connectionMatrix.size(); ++i)
    {
        if (hungarianFind(i, connectionMatrix, right_connected, right_pair)) count++;
    }

    return right_pair;
}

// ########################## End of Project Code ###################################