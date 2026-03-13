#include <iostream>
#include <memory>
#include <vector>
#include <functional>
#include <limits>
#include <string>
#include <cmath>
#include <algorithm>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>

#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl_conversions/pcl_conversions.h>

using PointRGB = pcl::PointXYZRGB;
using PointI = pcl::PointXYZI;
using PointXYZ = pcl::PointXYZ;
using SHOTDesc = pcl::SHOT352;

struct FeaturesResultSHOT
{
    pcl::PointCloud<PointXYZ>::Ptr keypoints;
    pcl::PointCloud<SHOTDesc>::Ptr descriptors;
};

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

pcl::PointCloud<PointI>::Ptr detectHARRISKeypoints(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz)
{
    auto keypoints = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);

    pcl::HarrisKeypoint3D<PointXYZ, PointI> harris;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    harris.setInputCloud(cloud_xyz);
    harris.setSearchMethod(tree);
    harris.setNonMaxSupression(true);
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
    ne.setRadiusSearch(0.05);

    ne.compute(*normals);

    return normals;
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

bool isValidSHOTDescriptor(const SHOTDesc& desc)
{
    for (int i = 0; i < 352; ++i)
    {
        if (!std::isfinite(desc.descriptor[i]))
        {
            return false;
        }
    }
    return true;
}

FeaturesResultSHOT filterValidSHOTFeatures(const FeaturesResultSHOT& input)
{
    FeaturesResultSHOT output;
    output.keypoints.reset(new pcl::PointCloud<PointXYZ>);
    output.descriptors.reset(new pcl::PointCloud<SHOTDesc>);

    std::size_t n = std::min(input.keypoints->size(), input.descriptors->size());

    for (std::size_t i = 0; i < n; ++i)
    {
        if (isValidSHOTDescriptor(input.descriptors->points[i]))
        {
            output.keypoints->points.push_back(input.keypoints->points[i]);
            output.descriptors->points.push_back(input.descriptors->points[i]);
        }
    }

    output.keypoints->width = output.keypoints->points.size();
    output.keypoints->height = 1;
    output.keypoints->is_dense = false;

    output.descriptors->width = output.descriptors->points.size();
    output.descriptors->height = 1;
    output.descriptors->is_dense = false;

    return output;
}

FeaturesResultSHOT extractFeaturesHARRIS_SHOT(
    const pcl::PointCloud<PointRGB>::Ptr& cloud_rgb)
{
    FeaturesResultSHOT result;

    auto cloud_xyz = convertRGBToXYZ(cloud_rgb);
    auto keypoints_harris = detectHARRISKeypoints(cloud_xyz);
    auto keypoints_xyz = convertHARRISToXYZ(keypoints_harris);
    auto normals = estimateNormals(cloud_xyz);
    auto descriptors = computeSHOT(cloud_xyz, keypoints_xyz, normals);

    result.keypoints = keypoints_xyz;
    result.descriptors = descriptors;

    std::cout << "Keypoints HARRIS brutos: " << result.keypoints->size() << std::endl;
    std::cout << "Descriptores SHOT brutos: " << result.descriptors->size() << std::endl;

    return result;
}

pcl::CorrespondencesPtr findCorrespondencesSHOT(
    const pcl::PointCloud<SHOTDesc>::Ptr& prev,
    const pcl::PointCloud<SHOTDesc>::Ptr& curr)
{
    pcl::CorrespondencesPtr corr(new pcl::Correspondences);

    if (prev->empty() || curr->empty())
        return corr;

    pcl::registration::CorrespondenceEstimation<SHOTDesc, SHOTDesc> est;
    est.setInputSource(prev);
    est.setInputTarget(curr);
    est.determineCorrespondences(*corr, std::numeric_limits<float>::max());

    return corr;
}

pcl::CorrespondencesPtr runRANSAC(
    const pcl::PointCloud<PointXYZ>::Ptr& keypoints_prev,
    const pcl::PointCloud<PointXYZ>::Ptr& keypoints_curr,
    const pcl::CorrespondencesPtr& input_corr,
    Eigen::Matrix4f& transformation)
{
    pcl::CorrespondencesPtr inliers(new pcl::Correspondences);

    if (keypoints_prev->empty() || keypoints_curr->empty() || input_corr->empty())
    {
        transformation = Eigen::Matrix4f::Identity();
        return inliers;
    }

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZ> ransac;
    ransac.setInputSource(keypoints_prev);
    ransac.setInputTarget(keypoints_curr);
    ransac.setInputCorrespondences(input_corr);
    ransac.setInlierThreshold(0.05);
    ransac.setMaximumIterations(1000);
    ransac.getCorrespondences(*inliers);

    transformation = ransac.getBestTransformation();

    return inliers;
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
    FeaturesResultSHOT prev_features_;
    bool has_prev_ = false;
    std::size_t global_map_size_ = 0;
    std::size_t counter_;

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        counter_++;

        if (counter_ % 30 != 0) return;

        pcl::PointCloud<PointRGB>::Ptr cloud(new pcl::PointCloud<PointRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        pcl::PointCloud<PointRGB>::Ptr filtered(new pcl::PointCloud<PointRGB>);
        pcl::VoxelGrid<PointRGB> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        vg.filter(*filtered);

        if (filtered->empty()) return;

        std::cout << "Puntos nube filtrada: " << filtered->size() << std::endl;

        FeaturesResultSHOT raw_features = extractFeaturesHARRIS_SHOT(filtered);
        FeaturesResultSHOT features = filterValidSHOTFeatures(raw_features);

        std::cout << "Keypoints/descriptores validos: "
                  << features.keypoints->size() << std::endl;

        if (!has_prev_)
        {
            prev_features_ = features;
            global_map_size_ = filtered->size();
            has_prev_ = true;
            return;
        }

        auto correspondences = findCorrespondencesSHOT(
            prev_features_.descriptors,
            features.descriptors);

        RCLCPP_INFO(this->get_logger(),
            "Correspondencias encontradas entre nubes: %zu",
            correspondences->size());

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        auto inliers = runRANSAC(
            prev_features_.keypoints,
            features.keypoints,
            correspondences,
            T);

        global_map_size_ += filtered->size();

        RCLCPP_INFO(this->get_logger(),
            "Original: %zu | Filtrada: %zu | Keypoints validos: %zu | Corr: %zu | Inliers: %zu | Mapa global: %zu",
            cloud->size(),
            filtered->size(),
            features.keypoints->size(),
            correspondences->size(),
            inliers->size(),
            global_map_size_);

        prev_features_ = features;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclSubNode>());
    rclcpp::shutdown();
    return 0;
}