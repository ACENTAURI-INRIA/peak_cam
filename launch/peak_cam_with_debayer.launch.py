import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_arg = launch.actions.DeclareLaunchArgument("config", default_value='peak_cam_params.yaml')
    camera_info_arg = launch.actions.DeclareLaunchArgument("camera_info", default_value='default_camera_info.yaml')

    parameters_file = PathJoinSubstitution([FindPackageShare('peak_cam'), 'params', 'settings', LaunchConfiguration('config')])
    camera_info_file = [ TextSubstitution(text='file://'), PathJoinSubstitution([FindPackageShare('peak_cam'), 'params', 'intrinsics', LaunchConfiguration('camera_info')]) ]

    container = ComposableNodeContainer(
        name='peak_cam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='peak_cam',
                plugin='peak_cam::PeakCamNode',
                name='peak_cam',
                parameters=[
                    parameters_file,
                    {'PixelFormat': 'BayerRG8',
                     'ImagePixelFormat': 'BayerRG8',
                     'camera_info_url': camera_info_file}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer_node',
                remappings=[('image_raw', '/peak_cam/image_raw')],
                parameters=[{'debayer': 3}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen'
    )

    return launch.LaunchDescription(
        [config_arg, camera_info_arg, container]
    )
