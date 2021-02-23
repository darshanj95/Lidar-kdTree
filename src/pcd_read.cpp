#include "pclProcessor.h"

int main(int arg, char ** argv){

    pclProcessor* processor = new pclProcessor();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer viewer ("Viewer");

    float filterRes = 0.2;
    Eigen::Vector4f minPoint(-20, -8, -5, 1);
    Eigen::Vector4f maxPoint(40, 8, 5, 1);
    int maxIter = 150;
    float distanceThreshold = 0.2;

    if(pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/data0.pcd", *cloud) == -1) {
        PCL_ERROR("Could not load file \n");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud = processor->FilterCloud(cloud, filterRes, minPoint, maxPoint);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processor->SegmentPlane(filterCloud, maxIter, distanceThreshold);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorG(segmentCloud.second,0,255,0);

    std::cout <<"LOADED: " << cloud->width * cloud->height << " data points. \n";

    viewer.addPointCloud(segmentCloud.second, colorG, "planeCloud");

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor->Clustering(segmentCloud.first, 0.4, 100, 500);
    int clusterId = 0;
    int i = 255;
    int k = 0;

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
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

    while(!viewer.wasStopped())
        viewer.spinOnce();

    return 0; 
}
