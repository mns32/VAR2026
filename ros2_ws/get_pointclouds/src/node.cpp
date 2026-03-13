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

#include <pcl/keypoints/harris_3d.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/correspondence.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

// ============================================================
// Tipos de datos que vamos a usar en PCL
// ============================================================

using PointRGB = pcl::PointXYZRGB;   // punto con coordenadas + color
using PointI   = pcl::PointXYZI;     // punto con coordenadas + intensidad
using PointXYZ = pcl::PointXYZ;      // punto solo geométrico
using SHOTDesc = pcl::SHOT352;       // descriptor SHOT

// ============================================================
// Estructura para guardar una nube ya procesada
// ============================================================
// Guardamos:
// - la nube RGB filtrada
// - la nube en XYZ
// - los keypoints
// - los descriptores de esos keypoints
// ============================================================

struct FrameData
{
    pcl::PointCloud<PointRGB>::Ptr cloud_rgb;
    pcl::PointCloud<PointXYZ>::Ptr cloud_xyz;
    pcl::PointCloud<PointXYZ>::Ptr keypoints;
    pcl::PointCloud<SHOTDesc>::Ptr descriptors;
};

// ============================================================
// Convierte una nube RGB a una nube XYZ
// ============================================================
// Muchos algoritmos de PCL no necesitan color, solo geometría.
// Aquí copiamos x, y, z de cada punto válido.
// ============================================================

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

// ============================================================
// Convierte los keypoints Harris a nube XYZ
// ============================================================
// Harris3D devuelve puntos con tipo PointXYZI.
// Para trabajar cómodamente con ellos luego, los pasamos a XYZ.
// ============================================================

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

// ============================================================
// Detección de keypoints con Harris 3D
// ============================================================
// Busca puntos geométricamente interesantes dentro de la nube.
// ============================================================

pcl::PointCloud<PointI>::Ptr detectHARRISKeypoints(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz)
{
    auto keypoints = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);

    pcl::HarrisKeypoint3D<PointXYZ, PointI> harris;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    harris.setInputCloud(cloud_xyz);
    harris.setSearchMethod(tree);

    // Evita keypoints repetidos muy cercanos entre sí
    harris.setNonMaxSupression(true);

    // Radio del vecindario para detectar esquinas/salientes 3D
    harris.setRadius(0.02f);

    // Umbral de respuesta del detector
    harris.setThreshold(1e-6f);

    // Mejora ligeramente la posición del keypoint
    harris.setRefine(true);

    harris.compute(*keypoints);

    return keypoints;
}

// ============================================================
// Estimación de normales
// ============================================================
// SHOT necesita conocer la orientación local de la superficie.
// Para ello calculamos una normal en cada punto.
// ============================================================

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<PointXYZ>::Ptr& cloud_xyz)
{
    auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());

    ne.setInputCloud(cloud_xyz);
    ne.setSearchMethod(tree);

    // Radio usado para calcular la normal local
    ne.setRadiusSearch(0.03);

    ne.compute(*normals);

    return normals;
}

// ============================================================
// Cálculo de descriptores SHOT
// ============================================================
// Se calculan solo sobre los keypoints.
// La nube completa sirve como superficie de contexto.
// ============================================================

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
    shot.setRadiusSearch(0.05);

    shot.compute(*descriptors);

    return descriptors;
}

// ============================================================
// Filtrado de descriptores SHOT inválidos
// ============================================================
// Problema:
// algunos descriptores SHOT pueden contener NaN o Inf si la
// referencia local no se puede calcular correctamente.
//
// Solución:
// eliminar esos descriptores Y también eliminar el keypoint
// correspondiente, para que ambos sigan alineados por índice.
// ============================================================

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

        // SHOT352 tiene 352 componentes
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

// ============================================================
// Procesado completo de una nube: HARRIS + SHOT
// ============================================================
// Este es el paso 1 del enunciado:
// - detectar características
// - describirlas
// ============================================================

FrameData processFrameHARRIS_SHOT(const pcl::PointCloud<PointRGB>::Ptr& cloud_rgb)
{
    FrameData frame;

    frame.cloud_rgb = cloud_rgb;
    frame.cloud_xyz = convertRGBToXYZ(cloud_rgb);

    // 1) Detectar keypoints Harris
    auto keypoints_harris = detectHARRISKeypoints(frame.cloud_xyz);
    auto raw_keypoints = convertHARRISToXYZ(keypoints_harris);

    // 2) Calcular normales de la nube
    auto normals = estimateNormals(frame.cloud_xyz);

    // 3) Calcular descriptores SHOT sobre los keypoints
    auto raw_descriptors = computeSHOT(frame.cloud_xyz, raw_keypoints, normals);

    // 4) Eliminar descriptores no válidos y sus keypoints asociados
    removeInvalidSHOTDescriptors(
        raw_keypoints,
        raw_descriptors,
        frame.keypoints,
        frame.descriptors);

    std::cout << "Puntos nube filtrada: " << frame.cloud_rgb->size() << std::endl;
    std::cout << "Keypoints HARRIS brutos: " << raw_keypoints->size() << std::endl;
    std::cout << "Descriptores SHOT brutos: " << raw_descriptors->size() << std::endl;
    std::cout << "Keypoints/descriptores validos: " << frame.keypoints->size() << std::endl;

    return frame;
}

// ============================================================
// Búsqueda de correspondencias entre dos conjuntos de descriptores
// ============================================================
// source = nube actual
// target = nube anterior
//
// Se usan correspondencias recíprocas para reducir emparejamientos malos.
// ============================================================

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

// ============================================================
// Estimación de transformación con RANSAC
// ============================================================
// Paso 3 del enunciado:
// a partir de las correspondencias, RANSAC elimina emparejamientos
// erróneos y calcula la mejor transformación rígida.
// ============================================================

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
    ransac.setInlierThreshold(0.05);

    // Máximo número de iteraciones
    ransac.setMaximumIterations(1000);

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

// ============================================================
// Nodo ROS2
// ============================================================

class PclSubNode : public rclcpp::Node
{
public:
    PclSubNode() : Node("get_pointclouds_node"), counter_(0)
    {
        // Suscripción a la nube de la cámara
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points",
            rclcpp::SensorDataQoS(),
            std::bind(&PclSubNode::topic_callback, this, std::placeholders::_1));

        // Publicador del mapa global acumulado
        publisher_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/global_map", 10);

        RCLCPP_INFO(this->get_logger(), "Nodo de registro de nubes iniciado");
        RCLCPP_INFO(this->get_logger(), "Pipeline activo: HARRIS + SHOT + Correspondencias + RANSAC + Mapa global");
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ========================================================
        // 1. Procesar solo una de cada N nubes
        // ========================================================
        counter_++;
        if (counter_ % 30 != 0) return;

        // ========================================================
        // 2. Convertir mensaje ROS a nube PCL
        // ========================================================
        pcl::PointCloud<PointRGB>::Ptr cloud(new pcl::PointCloud<PointRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Nube vacia recibida");
            return;
        }

        // ========================================================
        // 3. Filtrado inicial con VoxelGrid
        // ========================================================
        // Reduce el número de puntos para acelerar el procesamiento.
        // ========================================================
        pcl::PointCloud<PointRGB>::Ptr filtered(new pcl::PointCloud<PointRGB>);

        pcl::VoxelGrid<PointRGB> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);  // 2 cm
        vg.filter(*filtered);

        if (filtered->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Nube filtrada vacia");
            return;
        }

        // ========================================================
        // 4. Extraer keypoints + descriptores
        // ========================================================
        FrameData current_frame = processFrameHARRIS_SHOT(filtered);

        if (current_frame.keypoints->empty() || current_frame.descriptors->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No hay suficientes caracteristicas validas");
            return;
        }

        // ========================================================
        // 5. Caso especial: primera nube
        // ========================================================
        // La primera nube inicializa el mapa y el sistema global.
        // ========================================================
        if (!has_previous_frame_)
        {
            *global_map_ = *filtered;
            previous_frame_ = current_frame;
            has_previous_frame_ = true;
            global_transform_ = Eigen::Matrix4f::Identity();

            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*global_map_, map_msg);
            map_msg.header = msg->header;
            publisher_map_->publish(map_msg);

            RCLCPP_INFO(this->get_logger(), "Mapa inicializado con la primera nube");
            return;
        }

        // ========================================================
        // 6. Comprobar que hay suficientes descriptores válidos
        // ========================================================
        if (current_frame.descriptors->size() < 5 || previous_frame_.descriptors->size() < 5)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Muy pocos descriptores validos: actual=%zu anterior=%zu",
                current_frame.descriptors->size(),
                previous_frame_.descriptors->size());

            previous_frame_ = current_frame;
            return;
        }

        // ========================================================
        // 7. Buscar correspondencias entre nube actual y anterior
        // ========================================================
        auto correspondences = findCorrespondencesSHOT(
            current_frame.descriptors,
            previous_frame_.descriptors);

        RCLCPP_INFO( //Sacar por pantalla el número se correspondencias encontradas entre 2 nubes
            this->get_logger(),
            "Correspondencias encontradas entre nubes: %zu",
            correspondences->size());

        if (correspondences->size() < 10)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Muy pocas correspondencias: %zu. Se actualiza referencia y se sigue.",
                correspondences->size());

            previous_frame_ = current_frame;
            return;
        }

        // ========================================================
        // 8. Usar RANSAC para eliminar malas correspondencias
        //    y calcular la transformación relativa
        // ========================================================
        Eigen::Matrix4f relative_transform;
        pcl::Correspondences inliers;

        bool ok = estimateTransformationRANSAC(
            current_frame.keypoints,
            previous_frame_.keypoints,
            correspondences,
            relative_transform,
            inliers);

        if (!ok)
        {
            RCLCPP_WARN(this->get_logger(), "RANSAC no ha encontrado una transformacion valida");
            previous_frame_ = current_frame;
            return;
        }

        // ========================================================
        // 9. Acumular transformación global
        // ========================================================
        // Cada nube nueva se lleva al sistema de la primera nube.
        // ========================================================
        global_transform_ = global_transform_ * relative_transform;

        // ========================================================
        // 10. Transformar la nube actual al sistema global
        // ========================================================
        pcl::PointCloud<PointRGB>::Ptr transformed_cloud(new pcl::PointCloud<PointRGB>);
        pcl::transformPointCloud(*filtered, *transformed_cloud, global_transform_);

        // ========================================================
        // 11. Añadir nube transformada al mapa global
        // ========================================================
        *global_map_ += *transformed_cloud;

        // ========================================================
        // 12. Reducir el mapa global con VoxelGrid
        // ========================================================
        // Esto evita que el mapa crezca demasiado en número de puntos.
        // ========================================================
        pcl::PointCloud<PointRGB>::Ptr reduced_map(new pcl::PointCloud<PointRGB>);

        pcl::VoxelGrid<PointRGB> vg_map;
        vg_map.setInputCloud(global_map_);
        vg_map.setLeafSize(0.03f, 0.03f, 0.03f);  // 3 cm
        vg_map.filter(*reduced_map);

        global_map_ = reduced_map;

        // ========================================================
        // 13. Publicar mapa global para verlo en RViz
        // ========================================================
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header = msg->header;
        publisher_map_->publish(map_msg);

        // ========================================================
        // 14. Mostrar información de depuración
        // ========================================================
        RCLCPP_INFO(
            this->get_logger(),
            "Original: %zu | Filtrada: %zu | Keypoints validos: %zu | Corr: %zu | Inliers: %zu | Mapa global: %zu",
            cloud->size(),
            filtered->size(),
            current_frame.keypoints->size(),
            correspondences->size(),
            inliers.size(),
            global_map_->size());

        // ========================================================
        // 15. La nube actual pasa a ser la anterior
        // ========================================================
        previous_frame_ = current_frame;
    }

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_map_;

    // Procesar solo 1 de cada N nubes
    std::size_t counter_;

    // Estado del registro
    FrameData previous_frame_;
    bool has_previous_frame_ = false;

    // Mapa global acumulado
    pcl::PointCloud<PointRGB>::Ptr global_map_{new pcl::PointCloud<PointRGB>};

    // Transformación global acumulada
    Eigen::Matrix4f global_transform_ = Eigen::Matrix4f::Identity();
};

// ============================================================
// main
// ============================================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclSubNode>());
    rclcpp::shutdown();
    return 0;
}