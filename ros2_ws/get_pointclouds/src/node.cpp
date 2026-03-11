#include <iostream>
#include <memory>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>

#include <pcl_conversions/pcl_conversions.h>

using PointRGB = pcl::PointXYZRGB;        // nube con color
using PointI = pcl::PointXYZI;            // nube con intensidad
using PointXYZ = pcl::PointXYZ;           // nube solo geométrica
using SIFTPoint = pcl::PointWithScale;    // tipo de punto que devuelve SIFT
using SHOTDesc = pcl::SHOT352;            // tipo de descriptor SHOT

struct FeaturesResult
{
    pcl::PointCloud<PointXYZ>::Ptr keypoints;               // keypoints detectados
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors; // descriptores FPFH
};

struct FeaturesResultSHOT
{
    pcl::PointCloud<PointXYZ>::Ptr keypoints;   // keypoints detectados
    pcl::PointCloud<SHOTDesc>::Ptr descriptors; // descriptores SHOT
};

pcl::PointCloud<PointI>::Ptr convertRGBToIntensity(
    const pcl::PointCloud<PointRGB>::Ptr& cloud_rgb)
{
    auto cloud_i = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);

    cloud_i->points.reserve(cloud_rgb->points.size());

    for (const auto& p : cloud_rgb->points)
    {
        if (!pcl::isFinite(p)) continue;

        PointI q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;

        q.intensity = 0.299f * p.r + 0.587f * p.g + 0.114f * p.b;

        cloud_i->points.push_back(q);
    }

    cloud_i->width = cloud_i->points.size();
    cloud_i->height = 1;
    cloud_i->is_dense = false;

    return cloud_i;
}

pcl::PointCloud<PointXYZ>::Ptr convertRGBToXYZ(
    const pcl::PointCloud<PointRGB>::Ptr& cloud_rgb)
{
    auto cloud_xyz = pcl::PointCloud<PointXYZ>::Ptr(new pcl::PointCloud<PointXYZ>);

    cloud_xyz->points.reserve(cloud_rgb->points.size());

    for (const auto& p : cloud_rgb->points)
    {
        if (!pcl::isFinite(p)) continue;

        PointXYZ q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;

        cloud_xyz->points.push_back(q);
    }

    cloud_xyz->width = cloud_xyz->points.size();
    cloud_xyz->height = 1;
    cloud_xyz->is_dense = false;

    return cloud_xyz;
}

pcl::PointCloud<PointXYZ>::Ptr convertSIFTToXYZ(
    const pcl::PointCloud<SIFTPoint>::Ptr& keypoints_sift)
{
    auto keypoints_xyz = pcl::PointCloud<PointXYZ>::Ptr(new pcl::PointCloud<PointXYZ>);

    keypoints_xyz->points.reserve(keypoints_sift->points.size());

    for (const auto& p : keypoints_sift->points)
    {
        PointXYZ q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;

        keypoints_xyz->points.push_back(q);
    }

    keypoints_xyz->width = keypoints_xyz->points.size();
    keypoints_xyz->height = 1;
    keypoints_xyz->is_dense = false;

    return keypoints_xyz;
}

pcl::PointCloud<PointXYZ>::Ptr convertHARRISToXYZ(
    const pcl::PointCloud<PointI>::Ptr& keypoints_harris)
{
    auto keypoints_xyz = pcl::PointCloud<PointXYZ>::Ptr(new pcl::PointCloud<PointXYZ>);

    keypoints_xyz->points.reserve(keypoints_harris->points.size());

    for (const auto& p : keypoints_harris->points)
    {
        PointXYZ q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;

        keypoints_xyz->points.push_back(q);
    }

    keypoints_xyz->width = keypoints_xyz->points.size();
    keypoints_xyz->height = 1;
    keypoints_xyz->is_dense = false;

    return keypoints_xyz;
}

pcl::PointCloud<SIFTPoint>::Ptr detectSIFTKeypoints(
    const pcl::PointCloud<PointI>::Ptr& cloud_i)
{
    auto keypoints = pcl::PointCloud<SIFTPoint>::Ptr(new pcl::PointCloud<SIFTPoint>);

    pcl::SIFTKeypoint<PointI, SIFTPoint> sift;
    pcl::search::KdTree<PointI>::Ptr tree(new pcl::search::KdTree<PointI>());

    sift.setSearchMethod(tree);
    sift.setInputCloud(cloud_i);

    sift.setScales(0.01f, 3, 2);
    sift.setMinimumContrast(0.001f);

    sift.compute(*keypoints);

    return keypoints;
}

pcl::PointCloud<PointI>::Ptr detectHARRISKeypoints(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz)
{
    auto keypoints = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);

    pcl::HarrisKeypoint3D<PointXYZ, PointI> harris;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    harris.setInputCloud(cloud_xyz);
    harris.setSearchMethod(tree);
    harris.setNonMaxSupression(true);
    //harris.setRadius(0.02f);
    //harris.setThreshold(1e-6f);
    harris.setRadius(0.02f);
    harris.setThreshold(1e-6f);
    harris.setRefine(true);

    harris.compute(*keypoints);

    return keypoints;
}

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz)
{
    auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    ne.setInputCloud(cloud_xyz);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);

    ne.compute(*normals);

    return normals;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz,
    const pcl::PointCloud<PointXYZ>::Ptr& keypoints_xyz,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    auto descriptors = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(
        new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::FPFHEstimation<PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    fpfh.setInputCloud(keypoints_xyz);
    fpfh.setSearchSurface(cloud_xyz);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.05);

    fpfh.compute(*descriptors);

    return descriptors;
}

pcl::PointCloud<SHOTDesc>::Ptr computeSHOT(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz,
    const pcl::PointCloud<PointXYZ>::Ptr& keypoints_xyz,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    auto descriptors = pcl::PointCloud<SHOTDesc>::Ptr(new pcl::PointCloud<SHOTDesc>);

    pcl::SHOTEstimation<PointXYZ, pcl::Normal, SHOTDesc> shot;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    shot.setInputCloud(keypoints_xyz);
    shot.setSearchSurface(cloud_xyz);
    shot.setInputNormals(normals);
    shot.setSearchMethod(tree);
    shot.setRadiusSearch(0.05);

    shot.compute(*descriptors);

    return descriptors;
}

FeaturesResult extractFeaturesSIFT_FPFH(
    const pcl::PointCloud<PointRGB>::Ptr& cloud_rgb)
{
    FeaturesResult result;

    auto cloud_i = convertRGBToIntensity(cloud_rgb);              // RGB -> intensidad
    auto cloud_xyz = convertRGBToXYZ(cloud_rgb);                  // RGB -> XYZ

    auto keypoints_sift = detectSIFTKeypoints(cloud_i);           // detector SIFT
    auto keypoints_xyz = convertSIFTToXYZ(keypoints_sift);        // convertir a XYZ

    auto normals = estimateNormals(cloud_xyz);                    // normales
    auto descriptors = computeFPFH(cloud_xyz, keypoints_xyz, normals); // descriptor FPFH

    result.keypoints = keypoints_xyz;
    result.descriptors = descriptors;

    std::cout << "Puntos nube original: " << cloud_rgb->size() << std::endl;
    std::cout << "Keypoints SIFT: " << result.keypoints->size() << std::endl;
    std::cout << "Descriptores FPFH: " << result.descriptors->size() << std::endl;

    return result;
}

FeaturesResult extractFeaturesHARRIS_SHOT(
    const pcl::PointCloud<PointRGB>::Ptr& cloud_rgb)
{
    FeaturesResult result;

    auto cloud_xyz = convertRGBToXYZ(cloud_rgb);                      // RGB -> XYZ

    auto keypoints_harris = detectHARRISKeypoints(cloud_xyz);         // detector Harris
    auto keypoints_xyz = convertHARRISToXYZ(keypoints_harris);        // convertir a XYZ

    auto normals = estimateNormals(cloud_xyz);                        // normales
    //auto descriptors = computeSHOT(cloud_xyz, keypoints_xyz, normals); // descriptor SHOT
    auto descriptors = computeFPFH(cloud_xyz, keypoints_xyz, normals); // descriptor FPFH

    result.keypoints = keypoints_xyz;
    result.descriptors = descriptors;

    std::cout << "Puntos nube original: " << cloud_rgb->size() << std::endl;
    std::cout << "Keypoints HARRIS: " << result.keypoints->size() << std::endl;
    std::cout << "Descriptores FPFH: " << result.descriptors->size() << std::endl;

    return result;
}

class PclSubNode : public rclcpp::Node
{
public:
    PclSubNode() : Node("get_pointclouds_node"), counter_(0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points",
            rclcpp::SensorDataQoS(),
            std::bind(&PclSubNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nodo de extraccion SIFT + FPFH y HARRIS + SHOT iniciado");
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        counter_++;

        if (counter_ % 30 != 0) return;  // procesar solo 1 de cada 30 nubes

        pcl::PointCloud<PointRGB>::Ptr cloud(new pcl::PointCloud<PointRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        pcl::PointCloud<PointRGB>::Ptr filtered(new pcl::PointCloud<PointRGB>);

        pcl::VoxelGrid<PointRGB> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        vg.filter(*filtered);

        if (filtered->empty()) return;

        FeaturesResult features_sift = extractFeaturesSIFT_FPFH(filtered);
        FeaturesResult features_harris = extractFeaturesHARRIS_SHOT(filtered);

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::size_t counter_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclSubNode>());
    rclcpp::shutdown();
    return 0;
}