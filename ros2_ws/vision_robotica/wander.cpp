#include <cmath>
#include <cstdlib>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Usamos std::chrono para definir tiempos (ms, s)
using namespace std::chrono_literals;

// Heredamos de rclcpp::Node
class Wander : public rclcpp::Node 
{
private:
    // Publishers/Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandPub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub;
    rclcpp::TimerBase::SharedPtr timer_; 

    // Variables de control
    double forwardVel;
    double rotateVel;
    double closestRange;

public:
    Wander() : Node("wander_node") {
        // Inicializar variables
        forwardVel = 1.0;
        rotateVel = 0.0;
        closestRange = 0.0;

        // Crear Publicador
        // Este método nos permite indicar al sistema que vamos a publicar 
        // mensajes de cmd_vel
		// El valor de 1 indica que si acumulamos varios mensajes, solo el 
        // último será enviado (tamaño de la cola).
		// El método devuelve el Publisher que recibirá los mensajes.
        commandPub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

        // Crear Suscriptores
        // Suscribe el método commandCallback al tópico base_scan (el láser 
        // proporcionado por Stage)
		// El método commandCallback será llamado cada vez que el emisor 
        // (stage) publique datos 
        // Necesitamos std::bind para vincular la función callback a esta clase
        laserSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "base_scan", 1, std::bind(&Wander::commandCallback, this, std::placeholders::_1));

        // El Timer 
        // Se ejecutará cada 100ms (10 Hz)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Wander::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Nodo Wander iniciado!");
    }

    // Procesa los datos de láser
    void commandCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // get_logger() es necesario para saber quién imprime el mensaje
        // Mínimo valor angular del láser
        RCLCPP_INFO_STREAM(this->get_logger(), "AngleMin: " << msg->angle_min);
        // Máximo valor angular del láser
        RCLCPP_INFO_STREAM(this->get_logger(), "AngleMax: " << msg->angle_max);
        // Incremento angular entre dos beams
        RCLCPP_INFO_STREAM(this->get_logger(), "AngleIncrement: " << msg->angle_increment);
        // Mínimo valor que devuelve el láser
        RCLCPP_INFO_STREAM(this->get_logger(), "RangeMin: " << msg->range_min);
        // Máximo valor que devuelve el láser. Valores por debajo y por encima
        // de estos rangos no deben ser tenidos en cuenta.
        RCLCPP_INFO_STREAM(this->get_logger(), "RangeMax: " << msg->range_max);

        // Total de valores que devuelve el láser
        int totalValues = static_cast<int>(std::ceil((msg->angle_max - msg->angle_min) / msg->angle_increment));
        totalValues = std::min(totalValues, static_cast<int>(msg->ranges.size()));

        for (int i = 0; i < totalValues; i++) {
            // Acceso a los valores de rango
            RCLCPP_INFO_STREAM(this->get_logger(), "Values[" << i << "]: " << msg->ranges[i]);
        }

        // TODO: a partir de los datos del láser se tiene que modificar las variables forwardVel y rotateVel para hacer que el robot no choque.
	};

    // “Bucle” por timer
    // Esta función se llama automáticamente 10 veces por segundo
    void timerCallback() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = forwardVel;
        msg.angular.z = rotateVel;
        commandPub->publish(msg);
    }
};

int main(int argc, char **argv) {
    // Inicializa ROS 2
    rclcpp::init(argc, argv);
    
    // Crear el nodo y comenzar su procesamiento 
    rclcpp::spin(std::make_shared<Wander>());
    
    rclcpp::shutdown();
    return 0;
}