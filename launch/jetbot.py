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
            remappings=[
                ("/odom", "/odom_raw")
            ],
            output='screen',
        ),
        Node(
            package='jetbot_pro_ros2',
            node_executable='odom_ekf',
            name='odom_ekf_node',
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
            remappings=[
                ("/camera/image_raw", "/csi_cam_0/image_raw"),
                ("/set_camera_info", "/csi_cam_0/set_camera_info")
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
                         'inverted': 'false',
                         'angle_compensate': 'true'}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='base_footprint_to_imu',
            arguments=["0", "0", "0.07", "0", "0", "0", "/base_footprint", "/base_imu_link"]
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=["0", "0", "0.15", "3.14", "0", "0", "/base_footprint", "/laser_frame"]
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='base_footprint_to_imu',
            arguments=["0", "0", "0.07", "0", "0", "0", "/base_footprint", "/csi_cam_0_link"]
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
