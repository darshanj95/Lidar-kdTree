#include "pclProcessor.h"

pclProcessor::pclProcessor() {}
pclProcessor::~pclProcessor() {}


//Filter point cloud data
pcl::PointCloud<pcl::PointXYZ>::Ptr pclProcessor::FilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){
    auto startTime = std::chrono::steady_clock::now();

    //Downsample points - Voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> vgrid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    vgrid.setInputCloud(cloud);
    vgrid.setLeafSize(filterRes, filterRes, filterRes);
    vgrid.filter(*cloud_filtered);

    //Remove points outside box (too far...)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRegion (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::CropBox<pcl::PointXYZ> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloudRegion);


    //Remove roof points inside box (too close... like roof of a car)
    std::vector<int> indices;

    pcl::CropBox<pcl::PointXYZ> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point: indices) {
      inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

//Assign indices to point cloud, seperate road from objects
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> pclProcessor::SeparateClouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    for (int idx: inliers->indices)
        plane_cloud->points.push_back(cloud->points[idx]);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obs_cloud);

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult(obs_cloud, plane_cloud);
    return segResult;
}

//PCL - RANSAC
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> pclProcessor::SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceThreshold){
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "PCL-RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

void pclProcessor::Proximity( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index, std::vector<bool> &proc_status, KdTree &tree, float distanceTol, std::vector<int> &cluster)
{
    cluster.push_back(index);
    proc_status.at(index) = true;
    std::vector<int> nearby_points = tree.search(cloud->points.at(index), distanceTol);
    for(auto np : nearby_points)
    {
        if(!proc_status.at(np))
            Proximity(cloud, np, proc_status, tree, distanceTol, cluster);
    }
}

std::vector<std::vector<int>> pclProcessor::euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, KdTree* tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<int> cluster;
    std::vector<bool> proc_status(cloud->points.size(), false);
    for(int i {0}; i < cloud->points.size(); i++)
    {
        if(!proc_status.at(i))
        {
            Proximity(cloud, i, proc_status, *tree, distanceTol, cluster);
            clusters.push_back(cluster);
            cluster.clear();
        }
    }
    return clusters;

}


std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pclProcessor::Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){

    auto startTime = std::chrono::steady_clock::now();

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    std::vector<pcl::PointIndices> cluster_indices;

    KdTree* tree = new KdTree;
    tree->dim = 3;
    for (int i=0; i < cloud->points.size(); i++)
        tree->insert(cloud->points[i],i);
    std::vector<std::vector<int>> cluster_vec = euclideanCluster(cloud, tree, clusterTolerance);
    for(auto clus: cluster_vec)
    {
        pcl::PointIndices pInd;
        pInd.indices = clus;
        if(clus.size() > minSize && clus.size() < maxSize)
            cluster_indices.push_back(pInd);
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            tempCloud->points.push_back (cloud->points[*pit]);
        tempCloud->width = tempCloud->points.size ();
        tempCloud->height = 1;
        tempCloud->is_dense = true;
        clusters.push_back(tempCloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}