from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetbot_pro_ros2',
            node_executable='jetbot',
            name='jetbot',
            parameters=[
                {"serial_port": "/dev/ttyACM0"},
                {"linear_correction": 1.0},
                {"angular_correction": 1.0},
                {"publish_odom_transform": False},
            ],
            output='screen',
        ),
        Node(
            package='robot_localization',
            node_executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                # Configuation for robot odometry EKF
                {"frequency": 30.0},
                {"sensor_timeout": 0.1},
                {"two_d_mode": True},
                {"transform_time_offset": 0.1},
                {"transform_timeout": 0.1},
                {"print_diagnostics": True},
                {"debug": False},
                {"map_frame": "map"},
                {"odom_frame": "odom"},
                {"base_link_frame": "base_link"},
                {"world_frame": "odom"},
                {"publish_tf": True},
                {"use_control": False},
    
                {"process_noise_covariance": [0.05, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0.05, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0.03, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.02, 0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015]},


                {"initial_estimate_covariance": [1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    1e-9, 0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    1e-9, 0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    1e-9, 0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    1e-9, 0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    1e-9, 0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    1e-9, 0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    1e-9,  0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     1e-9,  0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     1e-9,  0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     1e-9, 0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    1e-9, 0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    1e-9]},
    
                # Configuration for odometry sensors
                {"odom0": "/wheel/odometry"},
                {"odom0_config": [False, False, False, False, False, False, True, True, True, False, False, False, False, False, False]},
                {"odom0_queue_size": 10},
                {"odom0_nodelay": True},
                {"odom0_differential": False},
                {"odom0_relative": False},
                
                # Configuration for IMU sensors
                {"imu0": "/imu/data"},
                {"imu0_config": [False, False, False, True, True, False, False, False, False, True, True, True, True, True, True]},
                {"imu0_nodelay": False},
                {"imu0_differential": False},
                {"imu0_relative": False},
                {"imu0_queue_size": 10},
                {"imu0_remove_gravitational_acceleration": True},
            ]
        ),
        Node(
            package='gscam',
            node_executable='gscam_node',
            name='csi_cam_0',
            parameters=[
                {"camera_name": "csi_cam_0"},
                {"camera_info_url": "package://jetbot_pro_ros2/config/camera_calibration/cam_640x480.yaml"},
                {"gscam_config": "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! videoconvert"},
                {"frame_id": "/csi_cam_0_link"},
                {"sync_sink": False},               
            ],
            output='screen',
        ),
        Node(
            package='rplidar_ros',
            node_executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': "/dev/ttyACM1",
                         'serial_baudrate': 115200,
                         'frame_id': "/laser_frame",
                         'inverted': False,
                         'angle_compensate': True}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='base_footprint_to_imu',
            arguments=["0", "0", "0.07", "0", "0", "0", "/base_link", "/base_imu_link"]
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=["0", "0", "0.15", "3.14", "0", "0", "/base_link", "/laser_frame"]
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='base_footprint_to_imu',
            arguments=["0", "0", "0.07", "0", "0", "0", "/base_link", "/csi_cam_0_link"]
        ),
    ])

'''
<node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf" output="screen">
    <param name="output_frame" value="odom"/>
	<param name="base_footprint_frame" value="base_footprint"/>
    <param name="freq" value="30.0"/>
    <param name="sensor_timeout" value="0.5"/>
    <param name="odom_used" value="true"/>
    <param name="imu_used" value="true"/>
    <param name="vo_used" value="false"/>
    <param name="debug" value="false"/>
    <param name="self_diagnose" value="false"/>
    <remap from="odom" to="/odom_raw"/>
    <remap from="/imu_data" to="/imu"/>
	<remap from="/robot_pose_ekf/odom_combined" to="/odom_combined"/>
</node>
'''
