#include <iostream>
#include <memory>
#include <vector>
#include <functional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/keypoints/harris_3d.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/features/fpfh.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/correspondence.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

using PointRGB = pcl::PointXYZRGB;   // punto coordenadas + color
using PointI   = pcl::PointXYZI;     // punto coordenadas + intensidad
using PointXYZ = pcl::PointXYZ;      // punto geométrico
using SHOTDesc = pcl::SHOT352;       // descriptor SHOT
using FPFHDesc = pcl::FPFHSignature33; // descriptor FPFH

struct FrameData
{
    pcl::PointCloud<PointRGB>::Ptr cloud_rgb;
    pcl::PointCloud<PointXYZ>::Ptr cloud_xyz;
    pcl::PointCloud<PointXYZ>::Ptr keypoints;
    pcl::PointCloud<SHOTDesc>::Ptr shot_descriptors;
    pcl::PointCloud<FPFHDesc>::Ptr fpfh_descriptors;
};

// Convierte nube RGB a nube XYZ

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

// Convierte los keypoints Harris a nube XYZ

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

// Busca puntos interesantes dentro de la nube

pcl::PointCloud<PointI>::Ptr detectHARRISKeypoints(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz)
{
    auto keypoints = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);

    pcl::HarrisKeypoint3D<PointXYZ, PointI> harris;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    harris.setInputCloud(cloud_xyz);
    harris.setSearchMethod(tree);

    // Evita keypoints muy cercanos entre sí
    harris.setNonMaxSupression(true);

    // Radio del vecindario para detectar salientes
    harris.setRadius(0.08f);

    // Umbral de respuesta del detector
    harris.setThreshold(1e-7f);

    // Mejora la posición del keypoint
    harris.setRefine(true);

    harris.compute(*keypoints);

    return keypoints;
}

// Estimación de normales

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz)
{
    auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    ne.setInputCloud(cloud_xyz);
    ne.setSearchMethod(tree);

    // Radio usado para calcular la normal local
    ne.setRadiusSearch(0.08);

    ne.compute(*normals);

    return normals;
}

// ---- SHOT ----

pcl::PointCloud<SHOTDesc>::Ptr computeSHOT(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz,
    const pcl::PointCloud<PointXYZ>::Ptr& keypoints_xyz,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    auto descriptors = pcl::PointCloud<SHOTDesc>::Ptr(new pcl::PointCloud<SHOTDesc>);

    pcl::SHOTEstimation<PointXYZ, pcl::Normal, SHOTDesc> shot;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    shot.setInputCloud(keypoints_xyz);      // dónde calcular descriptor
    shot.setSearchSurface(cloud_xyz);       // nube completa como entorno
    shot.setInputNormals(normals);          // normales calculadas
    shot.setSearchMethod(tree);

    // Radio del entorno usado por SHOT
    shot.setRadiusSearch(0.18);

    shot.compute(*descriptors);

    return descriptors;
}

// Filtrado de descriptores SHOT inválidos (NaN)

void removeInvalidSHOTDescriptors(
    const pcl::PointCloud<PointXYZ>::Ptr& input_keypoints,
    const pcl::PointCloud<SHOTDesc>::Ptr& input_descriptors,
    pcl::PointCloud<PointXYZ>::Ptr& filtered_keypoints,
    pcl::PointCloud<SHOTDesc>::Ptr& filtered_descriptors)
{
    filtered_keypoints.reset(new pcl::PointCloud<PointXYZ>);
    filtered_descriptors.reset(new pcl::PointCloud<SHOTDesc>);

    for (std::size_t i = 0; i < input_descriptors->size(); ++i)
    {
        const auto& desc = input_descriptors->points[i];
        bool valid = true;

        for (int j = 0; j < 352; ++j)
        {
            if (!std::isfinite(desc.descriptor[j]))
            {
                valid = false;
                break;
            }
        }

        if (valid)
        {
            filtered_keypoints->points.push_back(input_keypoints->points[i]);
            filtered_descriptors->points.push_back(desc);
        }
    }

    filtered_keypoints->width = filtered_keypoints->points.size();
    filtered_keypoints->height = 1;
    filtered_keypoints->is_dense = false;

    filtered_descriptors->width = filtered_descriptors->points.size();
    filtered_descriptors->height = 1;
    filtered_descriptors->is_dense = false;
}

pcl::CorrespondencesPtr findCorrespondencesSHOT(
    const pcl::PointCloud<SHOTDesc>::Ptr& source_desc,
    const pcl::PointCloud<SHOTDesc>::Ptr& target_desc)
{
    auto correspondences = pcl::CorrespondencesPtr(new pcl::Correspondences);

    pcl::registration::CorrespondenceEstimation<SHOTDesc, SHOTDesc> est;
    est.setInputSource(source_desc);
    est.setInputTarget(target_desc);

    est.determineReciprocalCorrespondences(*correspondences);

    return correspondences;
}

// ---- FPFH ----

pcl::PointCloud<FPFHDesc>::Ptr computeFPFH(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz,
    const pcl::PointCloud<PointXYZ>::Ptr& keypoints_xyz,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    auto descriptors = pcl::PointCloud<FPFHDesc>::Ptr(new pcl::PointCloud<FPFHDesc>);

    pcl::FPFHEstimation<PointXYZ, pcl::Normal, FPFHDesc> fpfh;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    fpfh.setInputCloud(keypoints_xyz);
    fpfh.setSearchSurface(cloud_xyz);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);

    // Radio del entorno usado por FPFH
    fpfh.setRadiusSearch(0.18);

    fpfh.compute(*descriptors);

    return descriptors;
}

// Filtrado de descriptores FPFH inválidos (NaN)

void removeInvalidFPFHDescriptors(
    const pcl::PointCloud<PointXYZ>::Ptr& input_keypoints,
    const pcl::PointCloud<FPFHDesc>::Ptr& input_descriptors,
    pcl::PointCloud<PointXYZ>::Ptr& filtered_keypoints,
    pcl::PointCloud<FPFHDesc>::Ptr& filtered_descriptors)
{
    filtered_keypoints.reset(new pcl::PointCloud<PointXYZ>);
    filtered_descriptors.reset(new pcl::PointCloud<FPFHDesc>);

    for (std::size_t i = 0; i < input_descriptors->size(); ++i)
    {
        const auto& desc = input_descriptors->points[i];
        bool valid = true;

        for (int j = 0; j < 33; ++j)
        {
            if (!std::isfinite(desc.histogram[j]))
            {
                valid = false;
                break;
            }
        }

        if (valid)
        {
            filtered_keypoints->points.push_back(input_keypoints->points[i]);
            filtered_descriptors->points.push_back(desc);
        }
    }

    filtered_keypoints->width = filtered_keypoints->points.size();
    filtered_keypoints->height = 1;
    filtered_keypoints->is_dense = false;

    filtered_descriptors->width = filtered_descriptors->points.size();
    filtered_descriptors->height = 1;
    filtered_descriptors->is_dense = false;
}

pcl::CorrespondencesPtr findCorrespondencesFPFH(
    const pcl::PointCloud<FPFHDesc>::Ptr& source_desc,
    const pcl::PointCloud<FPFHDesc>::Ptr& target_desc)
{
    auto correspondences = pcl::CorrespondencesPtr(new pcl::Correspondences);

    pcl::registration::CorrespondenceEstimation<FPFHDesc, FPFHDesc> est;
    est.setInputSource(source_desc);
    est.setInputTarget(target_desc);

    est.determineReciprocalCorrespondences(*correspondences);

    return correspondences;
}

// --------------------------------------------------------------------------------------------------------------------
// Procesado de una nube: HARRIS + SHOT

FrameData processFrameHARRIS_SHOT(const pcl::PointCloud<PointRGB>::Ptr& cloud_rgb)
{
    FrameData frame;

    frame.cloud_rgb = cloud_rgb;
    frame.cloud_xyz = convertRGBToXYZ(cloud_rgb);

    auto keypoints_harris = detectHARRISKeypoints(frame.cloud_xyz);
    auto raw_keypoints = convertHARRISToXYZ(keypoints_harris);

    auto normals = estimateNormals(frame.cloud_xyz);

    auto raw_descriptors = computeSHOT(frame.cloud_xyz, raw_keypoints, normals);

    removeInvalidSHOTDescriptors(
        raw_keypoints,
        raw_descriptors,
        frame.keypoints,
        frame.shot_descriptors);

    std::cout << "Puntos nube filtrada: " << frame.cloud_rgb->size() << std::endl;
    std::cout << "Keypoints HARRIS brutos: " << raw_keypoints->size() << std::endl;
    std::cout << "Descriptores SHOT brutos: " << raw_descriptors->size() << std::endl;
    std::cout << "Keypoints/descriptores validos: " << frame.keypoints->size() << std::endl;

    return frame;
}

// Procesado de una nube: HARRIS + FPFH

FrameData processFrameHARRIS_FPFH(const pcl::PointCloud<PointRGB>::Ptr& cloud_rgb)
{
    FrameData frame;

    frame.cloud_rgb = cloud_rgb;
    frame.cloud_xyz = convertRGBToXYZ(cloud_rgb);

    auto keypoints_harris = detectHARRISKeypoints(frame.cloud_xyz);
    auto raw_keypoints = convertHARRISToXYZ(keypoints_harris);

    auto normals = estimateNormals(frame.cloud_xyz);

    auto raw_descriptors = computeFPFH(frame.cloud_xyz, raw_keypoints, normals);

    removeInvalidFPFHDescriptors(
        raw_keypoints,
        raw_descriptors,
        frame.keypoints,
        frame.fpfh_descriptors);

    std::cout << "Puntos nube filtrada: " << frame.cloud_rgb->size() << std::endl;
    std::cout << "Keypoints HARRIS brutos: " << raw_keypoints->size() << std::endl;
    std::cout << "Descriptores FPFH brutos: " << raw_descriptors->size() << std::endl;
    std::cout << "Keypoints/descriptores validos: " << frame.keypoints->size() << std::endl;

    return frame;
}

// --------------------------------------------------------------------------------------------------------------------
// Transformación con RANSAC

bool estimateTransformationRANSAC(
    const pcl::PointCloud<PointXYZ>::Ptr& source_keypoints,
    const pcl::PointCloud<PointXYZ>::Ptr& target_keypoints,
    const pcl::CorrespondencesPtr& correspondences,
    Eigen::Matrix4f& transform,
    pcl::Correspondences& inliers)
{
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZ> ransac;

    ransac.setInputSource(source_keypoints);
    ransac.setInputTarget(target_keypoints);
    ransac.setInputCorrespondences(correspondences);

    // Distancia máxima para considerar una correspondencia correcta
    ransac.setInlierThreshold(0.03);

    // Máximo número de iteraciones
    ransac.setMaximumIterations(10000);

    // Obtener inliers tras RANSAC
    ransac.getCorrespondences(inliers);

    // Con menos de 3 correspondencias no hay transformación fiable
    if (inliers.size() < 3)
    {
        return false;
    }

    transform = ransac.getBestTransformation();
    return true;
}

// ICP refina alineación de RANSAC

bool refineWithICP(
    const pcl::PointCloud<PointXYZ>::Ptr& source,
    const pcl::PointCloud<PointXYZ>::Ptr& target,
    const Eigen::Matrix4f& initial_guess,
    Eigen::Matrix4f& refined_transform)
{
    pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;

    icp.setInputSource(source);
    icp.setInputTarget(target);

    // Distancia máxima entre puntos correspondientes
    icp.setMaxCorrespondenceDistance(0.15);

    // Criterios de convergencia
    icp.setMaximumIterations(80);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);

    pcl::PointCloud<PointXYZ> aligned;
    icp.align(aligned, initial_guess);

    if (icp.hasConverged() && icp.getFitnessScore() < 0.1)
    {
        refined_transform = icp.getFinalTransformation();
        return true;
    }

    // Si ICP no converge bien, usamos la transformación original de RANSAC
    refined_transform = initial_guess;
    return false;
}

// Nodo ROS2

class PclSubNode : public rclcpp::Node
{
public:
    PclSubNode() : Node("get_pointclouds_node"), counter_(0)
    {
        // Parámetro para seleccionar pipeline: HARRIS_SHOT o HARRIS_FPFH
        this->declare_parameter<std::string>("pipeline", "HARRIS_SHOT");
        pipeline_ = this->get_parameter("pipeline").as_string();

        // Suscripción a la nube de la cámara
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points",
            rclcpp::SensorDataQoS(),
            std::bind(&PclSubNode::topic_callback, this, std::placeholders::_1));

        // Publicador del mapa global acumulado
        publisher_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/global_map", 10);

        // Timer para republicar el mapa cada 2 segundos
        map_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PclSubNode::publish_map, this));

        RCLCPP_INFO(this->get_logger(), "Nodo de registro de nubes iniciado");
        RCLCPP_INFO(this->get_logger(), "Pipeline activo: %s + Correspondencias + RANSAC + ICP + Mapa global",
            pipeline_.c_str());
    }

private:
    void publish_map()
    {
        if (global_map_->empty()) return;

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "odom";
        publisher_map_->publish(map_msg);
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Procesar solo una de cada N nubes
        counter_++;
        if (counter_ % 5 != 0) return;

        // Convertir mensaje ROS a nube PCL

        pcl::PointCloud<PointRGB>::Ptr cloud(new pcl::PointCloud<PointRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Nube vacia recibida");
            return;
        }

        // Filtrado inicial con VoxelGrid

        pcl::PointCloud<PointRGB>::Ptr filtered(new pcl::PointCloud<PointRGB>);

        pcl::VoxelGrid<PointRGB> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        vg.filter(*filtered);

        // Eliminar puntos fuera del rango útil del Kinect

        pcl::PointCloud<PointRGB>::Ptr cropped(new pcl::PointCloud<PointRGB>);

        pcl::PassThrough<PointRGB> pass_z;
        pass_z.setInputCloud(filtered);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.4f, 3.0f);
        pass_z.filter(*cropped);

        pcl::PointCloud<PointRGB>::Ptr cropped_x(new pcl::PointCloud<PointRGB>);
        pcl::PassThrough<PointRGB> pass_x;
        pass_x.setInputCloud(cropped);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-2.5f, 2.5f);
        pass_x.filter(*cropped_x);

        pcl::PointCloud<PointRGB>::Ptr cropped_xy(new pcl::PointCloud<PointRGB>);
        pcl::PassThrough<PointRGB> pass_y;
        pass_y.setInputCloud(cropped_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-2.5f, 2.5f);
        pass_y.filter(*cropped_xy);

        // Eliminar outliers estadísticos
        pcl::PointCloud<PointRGB>::Ptr inlier_cloud(new pcl::PointCloud<PointRGB>);
        pcl::StatisticalOutlierRemoval<PointRGB> sor;
        sor.setInputCloud(cropped_xy);
        sor.setMeanK(30);
        sor.setStddevMulThresh(0.3);
        sor.filter(*inlier_cloud);
        filtered = inlier_cloud;

        if (filtered->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Nube filtrada vacia");
            return;
        }

        // Rotar de frame óptico (z-adelante, y-abajo) a frame suelo (x-adelante, z-arriba)
        // x_suelo = z_cam, y_suelo = -x_cam, z_suelo = -y_cam
        // Altura de la cámara del turtlebot waffle ~0.19m
        Eigen::Matrix4f camera_to_ground = Eigen::Matrix4f::Identity();
        camera_to_ground <<  0,  0,  1,  0,
                            -1,  0,  0,  0,
                             0, -1,  0,  0.19f,
                             0,  0,  0,  1;

        pcl::PointCloud<PointRGB>::Ptr ground_cloud(new pcl::PointCloud<PointRGB>);
        pcl::transformPointCloud(*filtered, *ground_cloud, camera_to_ground);
        filtered = ground_cloud;

        // Pipeline de registro según parámetro

        FrameData current_frame;

        if (pipeline_ == "HARRIS_FPFH")
        {
            current_frame = processFrameHARRIS_FPFH(filtered);
        }
        else
        {
            current_frame = processFrameHARRIS_SHOT(filtered);
        }

        std::size_t n_kp = current_frame.keypoints ? current_frame.keypoints->size() : 0;
        std::size_t n_corr = 0;
        std::size_t n_inliers = 0;

        if (!has_previous_frame_)
        {
            // Primera nube: añadir directamente al mapa
            pcl::PointCloud<PointRGB>::Ptr clean_cloud(new pcl::PointCloud<PointRGB>);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*filtered, *clean_cloud, indices);
            *global_map_ += *clean_cloud;
            global_map_xyz_ = convertRGBToXYZ(global_map_);

            has_previous_frame_ = true;
            previous_frame_ = current_frame;

            RCLCPP_INFO(this->get_logger(),
                "Primera nube añadida | Puntos: %zu", clean_cloud->size());
            return;
        }

        // Buscar correspondencias según pipeline
        pcl::CorrespondencesPtr correspondences;
        bool has_enough_descriptors = false;

        if (pipeline_ == "HARRIS_FPFH")
        {
            has_enough_descriptors =
                current_frame.fpfh_descriptors && current_frame.fpfh_descriptors->size() >= 5 &&
                previous_frame_.fpfh_descriptors && previous_frame_.fpfh_descriptors->size() >= 5;

            if (has_enough_descriptors)
            {
                correspondences = findCorrespondencesFPFH(
                    current_frame.fpfh_descriptors,
                    previous_frame_.fpfh_descriptors);
            }
        }
        else
        {
            has_enough_descriptors =
                current_frame.shot_descriptors && current_frame.shot_descriptors->size() >= 5 &&
                previous_frame_.shot_descriptors && previous_frame_.shot_descriptors->size() >= 5;

            if (has_enough_descriptors)
            {
                correspondences = findCorrespondencesSHOT(
                    current_frame.shot_descriptors,
                    previous_frame_.shot_descriptors);
            }
        }

        if (has_enough_descriptors)
        {
            n_corr = correspondences->size();

            if (correspondences->size() >= 5)
            {
                Eigen::Matrix4f relative_transform;
                pcl::Correspondences inliers;

                bool ok = estimateTransformationRANSAC(
                    current_frame.keypoints,
                    previous_frame_.keypoints,
                    correspondences,
                    relative_transform,
                    inliers);

                n_inliers = inliers.size();

                if (ok)
                {
                    // Estimación inicial: acumular RANSAC relativo
                    Eigen::Matrix4f initial_global = global_transform_ * relative_transform;

                    // Refinar contra el mapa global con ICP (reduce drift)
                    Eigen::Matrix4f refined_global;
                    bool icp_ok = refineWithICP(
                        current_frame.cloud_xyz,
                        global_map_xyz_,
                        initial_global,
                        refined_global);

                    if (icp_ok)
                    {
                        global_transform_ = refined_global;
                    }
                    else
                    {
                        global_transform_ = initial_global;
                    }

                    // Transformar nube actual al frame global y añadir al mapa
                    pcl::PointCloud<PointRGB>::Ptr transformed_cloud(new pcl::PointCloud<PointRGB>);
                    pcl::transformPointCloud(*filtered, *transformed_cloud, global_transform_);

                    pcl::PointCloud<PointRGB>::Ptr clean_cloud(new pcl::PointCloud<PointRGB>);
                    std::vector<int> indices;
                    pcl::removeNaNFromPointCloud(*transformed_cloud, *clean_cloud, indices);

                    *global_map_ += *clean_cloud;

                    // Reducir mapa con VoxelGrid
                    pcl::PointCloud<PointRGB>::Ptr reduced_map(new pcl::PointCloud<PointRGB>);
                    pcl::VoxelGrid<PointRGB> vg_map;
                    vg_map.setInputCloud(global_map_);
                    vg_map.setLeafSize(0.02f, 0.02f, 0.02f);
                    vg_map.filter(*reduced_map);
                    global_map_ = reduced_map;

                    // Actualizar mapa XYZ para ICP
                    global_map_xyz_ = convertRGBToXYZ(global_map_);

                    RCLCPP_INFO(this->get_logger(),
                        "Registro [%s]: Kp=%zu | Corr=%zu | Inliers=%zu | ICP=%s",
                        pipeline_.c_str(), n_kp, n_corr, n_inliers,
                        icp_ok ? "OK" : "fallback");
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                        "RANSAC fallido: Kp=%zu | Corr=%zu | Inliers=%zu",
                        n_kp, n_corr, n_inliers);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                    "Pocas correspondencias: %zu", n_corr);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                "Pocos descriptores para registro");
        }

        previous_frame_ = current_frame;

        RCLCPP_INFO(this->get_logger(),
            "Mapa: %zu puntos", global_map_->size());
    }

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_map_;
    rclcpp::TimerBase::SharedPtr map_timer_;

    // Pipeline seleccionado
    std::string pipeline_;

    // Procesar solo 1 de cada N nubes
    std::size_t counter_;

    // Estado del registro
    FrameData previous_frame_;
    bool has_previous_frame_ = false;

    // Transformación acumulada (registro sin TF)
    Eigen::Matrix4f global_transform_ = Eigen::Matrix4f::Identity();

    // Mapa global acumulado
    pcl::PointCloud<PointRGB>::Ptr global_map_{new pcl::PointCloud<PointRGB>};
    pcl::PointCloud<PointXYZ>::Ptr global_map_xyz_{new pcl::PointCloud<PointXYZ>};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclSubNode>());
    rclcpp::shutdown();
    return 0;
}
