#include "pclProcessor.h"

int main(int arg, char ** argv){

    pclProcessor* processor = new pclProcessor();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer viewer ("Viewer");

    //Load .pcd file
    if(pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/data0.pcd", *cloud) == -1) {
        PCL_ERROR("Could not load file \n");
        return -1;
    }

    std::cout <<"LOADED: " << cloud->width * cloud->height << " data points. \n";

    // preprocessing filter paramenter
    float filterRes = 0.2;
    Eigen::Vector4f minPoint(-20, -8, -5, 1);
    Eigen::Vector4f maxPoint(40, 8, 5, 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud = processor->FilterCloud(cloud, filterRes, minPoint, maxPoint);

    //RANSAC Parameter
    int maxIter = 150;
    float distanceThreshold = 0.2;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processor->SegmentPlane(filterCloud, maxIter, distanceThreshold);

    //Clustering Parameter
    float clusterTolerance = 0.4;
    int minSize = 100;
    int maxSize = 500;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor->Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize);

    // Visualize
    //Road
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorG(segmentCloud.second,0,255,0);
    viewer.addPointCloud(segmentCloud.second, colorG, "planeCloud");

    //Objects | Clusters
    int clusterId = 0;
    int i = 255;
    int k = 0;

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(cluster,i,0,k);
        viewer.addPointCloud(cluster,col,"clusters"+std::to_string(clusterId));
        if (i == 255) {
            i = 0; k = 255;
        }
        else {
            i = 255; k = 0;
        }
        ++clusterId;
    }

    //Viewer
    while(!viewer.wasStopped())
        viewer.spinOnce();

    return 0; 
}
