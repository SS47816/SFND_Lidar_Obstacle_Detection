// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
    PointT point;
    int id;
    Node* left;
    Node* right;

    Node(PointT setPoint, int setId) : point(setPoint), id(setId), left(NULL), right(NULL) {};
};

template<typename PointT>
struct KdTree
{
    Node<PointT>* root;

    KdTree() : root(NULL) {};

    void insertHelper(Node<PointT>** node, uint depth, const typename PointT& point, int id)
    {
        // Tree is empty
        if (*node == NULL)
            *node = new Node(point, id);
        else
        {
            // Calulate current dimension
            uint cd = depth % 3;
            // if(point[cd] < ((*node)->point[cd]))
            if (point.data[cd] < ((*node)->point.data[cd]))
                insertHelper(&(*node)->left, depth + 1, point, id);
            else
                insertHelper(&(*node)->right, depth + 1, point, id);
        }
    }

    void insert(const typename PointT& point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root 
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(const typename PointT& target, typename Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if (node != NULL)
        {
            if (node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol)
				&& node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol)
                && node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol))
			{
				float distance = sqrt((node->point.x - target.x)*(node->point.x - target.x) 
									+ (node->point.y - target.y)*(node->point.y - target.y)
                                    + (node->point.z - target.z)*(node->point.z - target.z));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

            // check accross boundary
            if ((target.data[depth%3] - distanceTol) < node->point.data[depth%3])
                searchHelper(target, node->left, depth + 1, distanceTol, ids);
            if ((target.data[depth%3] + distanceTol) > node->point.data[depth%3])
                searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
    }
    
    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const typename PointT& target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);

        return ids;
    }
};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    BoxQ MinimumBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
    
    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    
    // Start of Project Code
    std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomizedSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr points, std::vector<int>& cluster, std::vector<bool>& processed, typename KdTree<PointT>* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree<PointT>* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> CustomizedClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
    // End of Project Code
};
#endif /* PROCESSPOINTCLOUDS_H_ */