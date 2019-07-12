from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        # launch_ros.actions.Node(
        #     package='tf2_ros', node_executable='static_transform_broadcaster', node_name='base2laser',
        #     arguments=[0, 0, 0, 0, 0, 0, 'base_link', 'laser']
        # ),
        launch_ros.actions.Node(
            package='alego2', node_executable='IP'
        ),
        launch_ros.actions.Node(
            package='alego2', node_executable='LO'
        ),
        launch_ros.actions.Node(
            package='lol2', node_executable='lol', node_name='lol', remappings=[('/corner', '/corner_less'), ('/surf', '/surf_less'), ('/odom', '/odom/lidar')],
            parameters=[{
                'tf_b2l_x': 0.,
                'tf_b2l_y': 0.,
                'tf_b2l_z': 0.,
                'tf_b2l_roll': 0.,
                'tf_b2l_pitch': 0.,
                'tf_b2l_yaw': 0.,
                'fn_poses': '/home/zh/workspace/catkin_ws/src/bag_file/pcd/keypose.pcd',
                'fn_surf': '/home/zh/workspace/catkin_ws/src/bag_file/pcd/surf.pcd',
                'fn_corner': '/home/zh/workspace/catkin_ws/src/bag_file/pcd/corner.pcd',
                'fn_outlier': '/home/zh/workspace/catkin_ws/src/bag_file/pcd/outlier.pcd',
            }],
            output='screen'
        )
    ])
