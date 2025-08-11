#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, LogInfo

def generate_launch_description():
    # 패키지 경로
    pkg_share   = get_package_share_directory('human_tracker')
    nav2_share  = get_package_share_directory('nav2_bringup')
    slam_share  = get_package_share_directory('slam_toolbox')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # 0) Nav2 파라미터 파일 인자 선언
    declare_nav2_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Nav2 파라미터 파일 경로'
    )

    # 1) SLAM Toolbox (online_async) → /map 토픽 퍼블리시
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', 'False'),
        ]
    )

    # # 2) Nav2 Bringup (SLAM 모드)
    # nav2_bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_share, 'launch', 'bringup_launch.py')
    #     ),
    #     launch_arguments=[
    #         ('slam',         'True'),                # SLAM 모드 활성화
    #         ('map',          ''),                    # SLAM 모드 시 빈 문자열
    #         ('use_sim_time', 'False'),
    #         ('autostart',    'True'),
    #         ('params_file',  LaunchConfiguration('params_file')),
    #     ]
    # )

    # 3) Azure Kinect 드라이버
    azure_kinect = Node(
        package='azure_kinect_ros2_driver',
        executable='azure_kinect_node',
        name='k4a_ros2_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'color_enabled': True},
            {'color_resolution': '720P'},
            {'color_format': 'bgra'},
            {'fps': 30},
            {'depth_enabled': True},
            {'depth_mode': 'NFOV_UNBINNED'},
            {'point_cloud': True},
            {'point_cloud_in_depth_frame': True},
        ],
    )

    # 4) PointCloud 필터링 노드
    filter_node = Node(
        package='depth_map',
        executable='filter_node',
        name='point_cloud_filter_node',
        output='screen',
        parameters=[
            {'input_cloud_topic': '/k4a/points2'},
            {'filtered_cloud_topic': '/filtered_points'},
        ],
    )

    # 5) 2D 맵핑 노드
    mapping_node = Node(
        package='depth_map',
        executable='pointcloud_2d_map_node',
        name='pointcloud_2d_map_node',
        output='screen',
        parameters=[
            {'input_cloud_topic': '/filtered_points'},
            {'map_topic': '/pointcloud_2d_map'},
            {'frame_id': 'camera_base'},
            {'resolution': 0.05},
            {'width': 200},
            {'height': 200},
            {'z_min': 0.05},
            {'z_max': 5.0},
        ],
    )

    # 6) Human Tracker 노드
    human_tracker = Node(
        package='human_tracker',
        executable='human_tracker_node',
        name='human_tracker',
        output='screen',
        parameters=[
            {'rgb_topic': '/k4a/rgb/image_raw'},
            {'depth_topic': '/k4a/depth_to_rgb/image_raw'},
            {'camerainfo_topic': '/k4a/depth/camera_info'},
            {'user_topic': '/user_tracking'},
            {'frame_id': 'camera_base'},
            {'fx': 525.0},
            {'fy': 525.0},
            {'cx': 320.0},
            {'cy': 240.0},
        ],
    )

    # 7) User Goal Planner 노드
    user_goal_planner = Node(
        package='human_tracker',
        executable='user_goal_planner',
        name='user_goal_planner',
        output='screen',
        parameters=[
            {'map_topic': '/pointcloud_2d_map'},
            {'goal_topic': '/user_tracking'},
            {'path_topic': '/global_plan'},
            {'base_frame': 'camera_base'},
            {'start_x': 0.0},
            {'start_y': 0.0},
        ],
    )

    # # 5초 뒤에 info 출력 + user_goal_planner 실행
    # delayed_goal_planner = TimerAction(
    #     period=3.0,
    #     actions=[
    #         LogInfo(msg='[TimerAction] 3초 경과: user_goal_planner 실행!'),
    #         user_goal_planner,
    #     ],
    # )

    return LaunchDescription([
        declare_nav2_arg,
        slam_toolbox,
        #nav2_bringup,
        azure_kinect,
        filter_node,
        mapping_node,
        human_tracker,
        user_goal_planner,
        #delayed_goal_planner,
    ])
