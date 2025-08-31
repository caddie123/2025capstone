#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class CmdVelToRpm(Node):
    """
    /cmd_vel(geometry_msgs/Twist) -> /cmd_rpm_left(Int32), /cmd_rpm_right(Int32)

    변환식 (차동 구동):
      v_l = v  - w * (W/2)
      v_r = v  + w * (W/2)
      RPM  = (v / (2*pi*r)) * 60

    파라미터:
      - wheel_radius_m: 바퀴 반지름 [m]
      - track_width_m : 좌우 바퀴 간 트랙 폭 [m]
      - max_wheel_rpm: 휠 RPM 한계 (클램프)
      - cmd_timeout_s : cmd_vel 수신 타임아웃(초) -> 타임아웃 시 0rpm
      - output_hz     : 출력 주기(Hz)
      - cmd_vel_topic : (/cmd_vel 기본)
      - left_topic    : (/cmd_rpm_left 기본)
      - right_topic   : (/cmd_rpm_right 기본)
      - rpm_scale     : 계산된 RPM에 추가 스케일(보정용, 기본 1.0)

    주의:
      - 실제 모터 구동 방향 반전은 md_controller 파라미터(invert_left/right)로 처리 권장.
      - md_controller에서 wheel RPM * GearRatio = 모터 RPM 으로 변환해 보냄.
    """

    def __init__(self):
        super().__init__("cmdvel_to_rpm")

        # ---- 파라미터 선언 ----
        self.declare_parameter("wheel_radius_m", 0.125)     # 바퀴 반지름 [m] (예: 0.10)
        self.declare_parameter("track_width_m",  0.76)     # 트랙 폭 [m]     (예: 0.50)
        self.declare_parameter("max_wheel_rpm",  1000)     # 휠 RPM 제한
        self.declare_parameter("cmd_timeout_s",  0.50)     # cmd_vel 타임아웃
        self.declare_parameter("output_hz",      30.0)     # 출력 주기
        self.declare_parameter("cmd_vel_topic",  "/cmd_vel")
        self.declare_parameter("left_topic",     "/cmd_rpm_left")
        self.declare_parameter("right_topic",    "/cmd_rpm_right")
        self.declare_parameter("rpm_scale",      1.0)      # 보정 스케일

        # ---- 파라미터 로드 ----
        self.r = float(self.get_parameter("wheel_radius_m").value)
        self.W = float(self.get_parameter("track_width_m").value)
        self.max_rpm = int(self.get_parameter("max_wheel_rpm").value)
        self.timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.output_hz = float(self.get_parameter("output_hz").value)
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.left_topic = self.get_parameter("left_topic").value
        self.right_topic = self.get_parameter("right_topic").value
        self.rpm_scale = float(self.get_parameter("rpm_scale").value)

        # ---- Pub/Sub ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_left = self.create_publisher(Int32, self.left_topic, qos)
        self.pub_right = self.create_publisher(Int32, self.right_topic, qos)
        self.sub = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, qos)

        # ---- 상태 ----
        self.last_cmd_time = 0.0
        self.last_v = 0.0       # m/s
        self.last_w = 0.0       # rad/s

        # ---- 타이머 ----
        period = 1.0 / max(1e-6, self.output_hz)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"[cmdvel_to_rpm] r={self.r:.3f} m, W={self.W:.3f} m, "
            f"max_rpm={self.max_rpm}, timeout={self.timeout_s}s, out_hz={self.output_hz}"
        )
        self.get_logger().info(
            f"[cmdvel_to_rpm] cmd_vel='{self.cmd_vel_topic}', left='{self.left_topic}', right='{self.right_topic}', rpm_scale={self.rpm_scale}"
        )

    # ----------------- 콜백 -----------------
    def _on_cmd_vel(self, msg: Twist):
        self.last_v = float(msg.linear.x)    # m/s
        self.last_w = float(msg.angular.z)   # rad/s
        self.last_cmd_time = time.time()

    # ----------------- 로직 -----------------
    def _tick(self):
        now = time.time()
        v = 0.0
        w = 0.0
        if now - self.last_cmd_time <= self.timeout_s:
            v = self.last_v
            w = self.last_w
        # else: 타임아웃 -> 0 명령
        
        w = -w
        
        # 차동 구동: 휠 선속도
        v_l = v - w * (self.W * 0.5)
        v_r = v + w * (self.W * 0.5)

        # 선속도 -> 휠 RPM
        # RPM = (v / (2*pi*r)) * 60
        two_pi_r = 2.0 * math.pi * self.r
        if two_pi_r <= 1e-9:
            self.get_logger().error("wheel_radius_m가 0 또는 너무 작습니다.")
            return

        rpm_l = (v_l / two_pi_r) * 60.0
        rpm_r = (v_r / two_pi_r) * 60.0

        # 보정 스케일
        rpm_l *= self.rpm_scale
        rpm_r *= self.rpm_scale

        # 정수/클램프
        rpm_l = int(max(-self.max_rpm, min(self.max_rpm, round(rpm_l))))
        rpm_r = int(max(-self.max_rpm, min(self.max_rpm, round(rpm_r))))

        # 퍼블리시
        ml, mr = Int32(), Int32()
        ml.data, mr.data = rpm_l, rpm_r
        self.pub_left.publish(ml)
        self.pub_right.publish(mr)

    # ----------------- 엔트리 -----------------
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRpm()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
