#ifndef PCLPROCESSOR_H_
#define PCLPROCESSOR_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "kdtree.h"

class pclProcessor {
    public:
        pclProcessor();
        ~pclProcessor();
        pcl::PointCloud<pcl::PointXYZ>::Ptr FilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceThreshold);
        void Proximity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index, std::vector<bool> &proc_status, KdTree &tree, float distanceTol, std::vector<int> &cluster);
        std::vector<std::vector<int>> euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, KdTree* tree, float distanceTol);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
};

#endif