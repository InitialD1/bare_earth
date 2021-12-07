#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
using namespace pcl;
using namespace std;
int main() {
    cout << "Hello, World!" << std::endl;
    PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud<pcl::PointXYZ>::Ptr pmf_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud<pcl::PointXYZ>::Ptr voxel_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud<pcl::PointXYZ>::Ptr statistical_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);
    pcl::PCDReader reader;
    pcl::PCDWriter writer;

    //读取点云
    reader.read<pcl::PointXYZ> ("../data/zhangzisong_cut1.pcd", *cloud);
    //双窗口
    pcl::visualization::PCLVisualizer douViewer("双窗口学习");

    //设置左右窗口
    int v1(0);
    int v2(0);
    douViewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
    douViewer.setBackgroundColor(0,0,0,v1);
    douViewer.createViewPort(0.5,0,1,1,v2);
    douViewer.setBackgroundColor(0.5,0.5,0.5,v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_src_blue(cloud, 0,0,255);
    douViewer.addPointCloud(cloud, cloud_src_blue, "cloud_src",v1);



    //下采样 体素化滤波
    cerr << "下采样前的点云信息" << cloud->width * cloud->height
         << "data points (" << getFieldsList(*cloud) << ")."
         << endl;
    //创建滤波对象
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*voxel_filtered);
    cerr << "下采样后的点云信息" << voxel_filtered->width * voxel_filtered->height
         << "data points (" << getFieldsList(*voxel_filtered) << ")."
         << endl;

    writer.write<pcl::PointXYZ> ("zhangzisong_cut1_downsampled.pcd", *voxel_filtered, false);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_orange(voxel_filtered, 250,128,10);
    douViewer.addPointCloud(voxel_filtered, cloud_out_orange, "voxel", v2);
    while(!douViewer.wasStopped())
    {
        douViewer.spinOnce();
    }
    //孤立点去噪(统计学方法)statistical_filtered
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisical;
    statisical.setInputCloud(voxel_filtered);
    statisical.setMeanK(50);
    statisical.setStddevMulThresh(1.0);
    statisical.setNegative(false);
    statisical.filter(*statistical_filtered);
    writer.write("zhangzisong_cut1_statistical.pcd", *statistical_filtered, false);
    cerr << "去噪后的点云信息" << statistical_filtered->width * statistical_filtered->height
         << "data points (" << getFieldsList(*statistical_filtered) << ")."
         << endl;
    cerr << "开始渐进形态学滤波" << endl;
    cerr << *statistical_filtered << endl;

    //创建滤波器
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(statistical_filtered);
    pmf.setMaxWindowSize(20);
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(0.5f);
    pmf.setMaxDistance(3.0f);
    pmf.extract(ground->indices);

    //Create the filter object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(statistical_filtered);
    extract.setIndices(ground);
    extract.filter(*pmf_filtered);

    cerr << "地面点云生成" << endl;
    cerr << *pmf_filtered << endl;

    //地面点写入及查看

    writer.write<pcl::PointXYZ> ("zhangzisong_cut1_ground.pcd", *pmf_filtered, false);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("地面点云查看"));
    viewer->setBackgroundColor(28,28,28);
    viewer->addPointCloud(pmf_filtered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pmf_filtered, 0, 0, 255), "ground");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "ground");
    viewer->spin();
    cout << "地面点云写入成功" << endl;

    //提取植被点及查看
    extract.setNegative(true);
    extract.filter(*pmf_filtered);
    cerr << "Object cloud after filtering: " << endl;
    cerr << *pmf_filtered << endl;
    writer.write<pcl::PointXYZ> ("zhangzisong_cut1_object.pcd", *pmf_filtered, false);
    pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("植被点云查看"));
    viewer1->setBackgroundColor(28,28,28);
    viewer1->addPointCloud(pmf_filtered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pmf_filtered, 0, 0, 255), "object");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "object");
    viewer1->spin();
    return 0;
}
