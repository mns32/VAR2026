import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Directorios
    pkg_share = get_package_share_directory('turtlebot_gazebo_multiple')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Archivo del Mundo y Modelo
    sdf_file = os.path.join(pkg_share, 'worlds', 'Actors_GrannyAnnie.world')
    
    # Obtener modelo del robot de variable de entorno o argumento
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    
    # Ruta al URDF del Turtlebot3 oficial
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        f'turtlebot3_{turtlebot3_model}.urdf'
    )

    # Ruta al modelo SDF (incluye plugins de movimiento para gz-sim)
    custom_model_sdf_file = os.path.join(
        pkg_share,
        'models',
        'turtlebot3_waffle_kinect',
        'model.sdf'
    )
    if os.path.exists(custom_model_sdf_file):
        model_sdf_file = custom_model_sdf_file
    else:
        model_sdf_file = os.path.join(
            pkg_turtlebot3_gazebo,
            'models',
            f'turtlebot3_{turtlebot3_model}',
            'model.sdf'
        )
        if not os.path.exists(model_sdf_file):
            model_sdf_file = os.path.join(
                pkg_turtlebot3_gazebo,
                'models',
                'turtlebot3_waffle',
                'model.sdf'
            )

    # Configurar Gazebo (Servidor y Cliente)
    # Le pasamos el archivo SDF directamente al simulador
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items(),
    )

    # Publicar el Estado del Robot (Robot State Publisher)
    # Procesa el URDF con xacro (estos URDFs de TurtleBot3 usan ${namespace})
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro', ' ', urdf_file])
        }]
    )

    # Publica /joint_states para que robot_state_publisher pueda publicar TF
    # de juntas NO fijas (p.ej. ruedas)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    # Spawnear el Robot (Entity Creation)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3',
            '-file', model_sdf_file,
            '-x', '-6.0',
            '-y', '-3.0',
            '-z', '0.2',
            '-Y', '0.0' # Rotación (yaw)
        ],
        output='screen'
    )

    # El Puente (Bridge) entre ROS 2 y Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            # RViz TurtleBot3 default config subscribes /scan as BEST_EFFORT
            'qos_overrides./scan.publisher.reliability': 'best_effort',
        }],
        arguments=[
            # Reloj de simulación
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Odometría y TF
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Comandos de velocidad
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Sensores 
            # -- Láser
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # -- Cámara RGB original del turtlebot3
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # -- KINECT 
            '/kinect/rgb@sensor_msgs/msg/Image[gz.msgs.Image',
            '/kinect/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/kinect/depth@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        # Remappings se usa para cambiar el nombre de tópicos de salida
        remappings=[
            ('/kinect/rgb', '/camera/rgb/image_raw'),
            ('/kinect/camera_info', '/camera/rgb/camera_info'),
            ('/kinect/depth', '/camera/depth/image_raw'),
        ],
        output='screen'
    )

    # Publica CameraInfo de profundidad con timestamp/frame_id del depth image
    depth_camera_info = Node(
        package='turtlebot_gazebo_multiple',
        executable='depth_camera_info_publisher',
        name='depth_camera_info_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'depth_image_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'width': 640,
            'height': 480,
            'hfov': 1.047,
            # Use TurtleBot3 URDF optical frame (no extra static TF)
            'frame_id': 'camera_rgb_optical_frame',
        }],
    )

    # Genera PointCloud2 con color (XYZRGB) desde depth + RGB image
    depth_to_points = ComposableNodeContainer(
        name='point_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                parameters=[{
                    'use_sim_time': True,
                }],
                remappings=[
                    ('rgb/image_rect_color', '/camera/rgb/image_raw'),
                    ('rgb/camera_info', '/camera/rgb/camera_info'),
                    ('depth_registered/image_rect', '/camera/depth/image_raw'),
                    ('points', '/camera/depth/points'),
                ]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        joint_state_publisher,
        robot_state_publisher,
        spawn_entity,
        bridge,
        depth_camera_info,
        depth_to_points,
    ])