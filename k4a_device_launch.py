# launch/combined_octomap.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) Azure Kinect 드라이버 (포인트클라우드 활성화)
        Node(
            package="azure_kinect_ros2_driver",
            executable="azure_kinect_node",
            name="k4a_ros2_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"color_enabled":             True},
                {"color_resolution":          "720P"},
                {"color_format":              "bgra"},
                {"fps":                       30},
                {"depth_enabled":             True},
                {"depth_mode":                "NFOV_UNBINNED"},
                {"point_cloud":               True},
                {"point_cloud_in_depth_frame": True},
            ],
        ),

        # 2) PointCloud 필터링 노드
        Node(
            package="depth_map",            # depth_map 패키지 이름
            executable="filter_node",       # ros2 run depth_map filter_node 로 실행하던 이름
            name="point_cloud_filter_node",
            output="screen",
            # parameters=[{
            #     # 필요하다면 여기서 XYZ 범위 등 설정
            #     "x_min": -1.0,
            #     "x_max":  1.0,
            #     "y_min": -1.0,
            #     "y_max":  1.0,
            #     "z_min":  0.1,
            #     "z_max":  3.0,
            # }],
            remappings=[
                ("/input_cloud", "/k4a/points2"),
                ("/filtered_cloud", "/filtered_points"),
            ],
        ),

        # 3) OctoMap 서버 (필터된 포인트 구독)
        Node(
            package="octomap_server",
            executable="octomap_server_node",
            name="octomap_server",
            output="screen",
            parameters=[
                {"frame_id": "camera_base"},   # RViz Fixed Frame 과 동일
                {"resolution": 0.1},
                {"publish_2d_map": True},            # 2D 맵도 함께 퍼블리시
            ],
            remappings=[
                ("cloud_in", "/filtered_points"),
            ],
        ),
    ])
