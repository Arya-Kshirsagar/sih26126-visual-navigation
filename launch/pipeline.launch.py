"""
SIH UGV pipeline: everything after the simulator, in one command.

  Bridge (stereo cameras Gazebo -> ROS)
  -> stereo_image_proc (rectify, disparity, point cloud)
  -> RTAB-Map stereo visual odometry (official odom -> base_link)
  -> RTAB-Map SLAM (map -> odom, /map)          [only with slam:=true]
  -> obstacle classifier + Nav2                  [only with nav:=true, needs slam:=true]

Run (after the sim is up):
  ros2 launch ~/SIH/launch/pipeline.launch.py
  ros2 launch ~/SIH/launch/pipeline.launch.py slam:=true
  ros2 launch ~/SIH/launch/pipeline.launch.py slam:=true nav:=true
RViz opens with ~/SIH/launch/sih.rviz (all maps preloaded); add rviz:=false to skip it.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')
    rviz = LaunchConfiguration('rviz')
    seg = LaunchConfiguration('seg')

    stereo_remaps = [
        ('left/image_rect', '/stereo/left/image_rect'),
        ('right/image_rect', '/stereo/right/image_rect'),
        ('left/camera_info', '/stereo/left/camera_info'),
        ('right/camera_info', '/stereo/right/camera_info'),
        ('odom', '/vo'),
    ]

    # 1. Gazebo -> ROS bridge for the stereo pair (was T4)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='stereo_bridge',
        output='screen',
        arguments=[
            '/stereo/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/stereo/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/stereo/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/stereo/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # 2. Stereo matching - disparity and point cloud (was T5)
    stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('stereo_image_proc'), 'launch', 'stereo_image_proc.launch.py'])),
        launch_arguments={
            'left_namespace': 'stereo/left',
            'right_namespace': 'stereo/right',
            'stereo_algorithm': '1',          # SGBM
            'disparity_range': '128',         # sees down to ~0.33 m
            'correlation_window_size': '7',
        }.items(),
    )

    # 3. Stereo visual odometry - the robot's official position (was T6)
    #    Logs only warnings, so the terminal stays readable. Check it with:
    #    ros2 topic hz /vo
    visual_odometry = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'use_sim_time': True,
            'approx_sync': False,
        }],
        remappings=stereo_remaps,
        ros_arguments=['--log-level', 'warn'],
    )

    # 4. SLAM - map and loop closure (was T7). Starts a fresh map each time (-d).
    slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(slam),
        arguments=['-d'],
        parameters=[{
            'subscribe_stereo': True,
            'subscribe_depth': False,
            'frame_id': 'base_link',
            'use_sim_time': True,
            'approx_sync': False,
            # 2D map settings (RTAB-Map expects these as strings)
            'Grid/MaxObstacleHeight': '0.4',   # ignore canopy; robot is ~0.25 m tall
            'Grid/RangeMax': '5.0',            # far stereo depth is noisy
            'Grid/RayTracing': 'true',         # fill free space up to obstacles
            'Grid/CellSize': '0.1',            # 10 cm cells
            'Grid/MaxGroundAngle': '25',       # steeper than 25 deg (roots, boulders) = obstacle
        }],
        remappings=stereo_remaps,
    )

    # 5. Obstacle classifier: splits the stereo point cloud into ground and obstacles,
    #    measured relative to the robot (works on slopes). Feeds Nav2.
    obstacles = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        condition=IfCondition(nav),
        parameters=[{
            'frame_id': 'base_link',
            'use_sim_time': True,
            'Grid/MaxObstacleHeight': '0.4',
            'Grid/RangeMax': '5.0',            # Nav2 marks obstacles only to 3.5 m; ground clears to 5 m
            'Grid/CellSize': '0.05',           # 5 cm cubes: small stones keep enough points
            'Grid/MaxGroundAngle': '25',       # steeper than 25 deg (roots, boulders) = obstacle
            'Grid/NormalK': '10',              # judge surface tilt over a smaller patch (was 20)
            'Grid/MinClusterSize': '3',        # keep small obstacle groups (default 10 dropped stones)
            'Grid/NoiseFilteringRadius': '0.15',      # drop isolated points (stereo noise)...
            'Grid/NoiseFilteringMinNeighbors': '5',   # ...that have fewer than 5 neighbours within 15 cm
        }],
        remappings=[
            ('cloud', '/points2'),
            ('obstacles', '/obstacles_cloud'),
            ('ground', '/ground_cloud'),
        ],
    )

    # 6. Nav2 (planner, controller, behaviors, collision monitor)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
        condition=IfCondition(nav),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.expanduser('~/SIH/launch/nav2_params.yaml'),
        }.items(),
    )

    # 9. Coloured disparity image for RViz (RViz cannot show stereo_msgs/DisparityImage directly)
    disparity_color = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/SIH/launch/disparity_color.py'), '--ros-args',
             '-p', 'use_sim_time:=true'],
        output='log',
    )

    # 8. Live terrain segmentation (HRNet+OCR on the GPU) - display only, Nav2 unaffected
    segmentation = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/SIH/launch/seg_node.py'), '--ros-args',
             '-p', 'use_sim_time:=true', '-p', 'rate:=3.0'],
        output='screen',
        condition=IfCondition(seg),
    )

    # 7. RViz with every display preloaded
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.expanduser('~/SIH/launch/sih.rviz')],
        parameters=[{'use_sim_time': True}],
        # Render on the RTX 3050 (the 3D map is heavy)
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='false',
                              description='Also start RTAB-Map SLAM (true/false)'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Also start Nav2; needs slam:=true (true/false)'),
        DeclareLaunchArgument('seg', default_value='false',
                              description='Run live terrain segmentation (true/false)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz with the SIH layout (true/false)'),
        bridge,
        stereo,
        visual_odometry,
        slam_node,
        obstacles,
        nav2,
        segmentation,
        disparity_color,
        rviz_node,
    ])