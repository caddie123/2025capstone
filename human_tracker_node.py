#!/usr/bin/env python3
import os
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import cv2
import numpy as np
import torch
import torch.nn.functional as F
import torch.nn as nn
from torchvision import models
from torchvision.models import MobileNet_V2_Weights
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
from dataclasses import dataclass
import tf2_geometry_msgs

@dataclass
class Detection:
    track_id: int
    bbox: tuple    # (x, y, w, h)
    center: tuple  # (cx, cy)
    depth: float   # meters

class MobileNetEmbedder(nn.Module):
    """Re-ID 용 MobileNet 임베더"""
    def __init__(self, device='cpu'):
        super().__init__()
        mb = models.mobilenet_v2(
            weights=MobileNet_V2_Weights.DEFAULT
        ).features
        self.features = mb.to(device)
        self.pool     = nn.AdaptiveAvgPool2d(1)
        self.device   = device

    def forward(self, x):
        x = x.to(self.device)
        f = self.features(x)
        return self.pool(f).view(f.size(0), -1)

class HumanTracker(Node):
    def __init__(self):
        super().__init__('human_tracker')

        # --- parameters ---
        self.declare_parameter('rgb_topic',   '/k4a/rgb/image_raw')
        self.declare_parameter('depth_topic', '/k4a/depth_to_rgb/image_raw')
        self.rgb_topic   = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value

        # re-ID params
        self.max_users      = 3
        self.max_missed     = 80
        self.gate_dx        = 300
        self.gate_dy        = 200
        self.alpha_hist     = 0.4
        self.feat_scale     = 100.0
        self.reid_threshold = 0.6

        # performance & smoothing
        self.frame_count     = 0
        self.skip_frames     = 1
        self.last_detections = []
        self.last_frame      = None
        self.prev_point      = None
        self.alpha_smooth    = 0.4

        # 고유 사용자 색상 지정 (BGR)
        self.user_color  = (0, 0, 255)  # 빨간
        self.other_color = (0, 255, 0)  # 초록

        # init
        self.bridge      = CvBridge()
        self.depth_frame = None
        self.device      = 'cuda' if torch.cuda.is_available() else 'cpu'

        # 카메라 내부 파라미터
        self.fx = None; self.fy = None
        self.cx_cam = None; self.cy_cam = None

        # YOLOv11s 모델 로드 (경량, FP16)
        model_file = os.path.join(os.path.dirname(__file__), 'yolo11s.pt')
        if not os.path.exists(model_file):
            self.get_logger().warn("Downloading yolo11s.pt …")
            urllib.request.urlretrieve(
                "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s.pt",
                model_file
            )
        self.yolo = YOLO(model_file).to(self.device)
        self.yolo.model.half()

        # DeepSort tracker
        self.tracker = DeepSort(
            max_age=90,
            n_init=4,
            max_cosine_distance=0.2,
            nn_budget=100,
            embedder='mobilenet',
            embedder_gpu=True,
            half=False,
            bgr=True,
        )

        # manual re-ID embedder
        self.embedder = MobileNetEmbedder(self.device).eval()

        # re-ID 상태
        self.user_ids     = []
        self.last_centers = {}
        self.orig_hists   = {}
        self.orig_feats   = {}
        self.missed       = {}

        # A1 strategy: last published goal
        self.last_goal = None

        # ROS pubs/subs
        self.create_subscription(Image,      self.rgb_topic,   self.rgb_cb,   10)
        self.create_subscription(Image,      self.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/k4a/rgb/camera_info', self.caminfo_cb, 10)
        self.user_pub = self.create_publisher(PointStamped, '/user_tracking', 10)

        # debug window
        cv2.namedWindow("Human Tracker", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Human Tracker", 640, 480)
        self.get_logger().info("✅ HumanTracker initialized")

    def depth_cb(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def caminfo_cb(self, msg: CameraInfo):
        self.fx = msg.k[0]; self.fy = msg.k[4]
        self.cx_cam = msg.k[2]; self.cy_cam = msg.k[5]

    def compute_hist(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h = cv2.calcHist([hsv],[0,1],None,[30,32],[0,180,0,256])
        cv2.normalize(h,h,0,1,cv2.NORM_MINMAX)
        return h

    def extract_feat(self, img, bbox):
        x,y,w,h = map(int,bbox)
        roi = img[y:y+h, x:x+w]
        t = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        t = torch.from_numpy(t).permute(2,0,1).unsqueeze(0).float()/255.0
        t = F.interpolate(t, size=(224,224), mode='bilinear').to(self.device)
        with torch.no_grad():
            f = self.embedder(t)
        return f.cpu().numpy().reshape(-1)

    def try_reid(self, uid, img, dets):
        best_score,best_id = float('inf'),None
        orig_h = self.orig_hists.get(uid); orig_f = self.orig_feats.get(uid)
        if orig_h is None or orig_f is None: return None
        lx,ly = self.last_centers[uid]
        for d in dets:
            cx,cy = d.center
            if abs(cx-lx)>self.gate_dx or abs(cy-ly)>self.gate_dy: continue
            x,y,w,h = map(int,d.bbox); roi = img[y:y+h, x:x+w]
            h2 = cv2.calcHist([cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)],[0,1],None,[30,32],[0,180,0,256])
            cv2.normalize(h2,h2,0,1,cv2.NORM_MINMAX)
            hd = cv2.compareHist(orig_h,h2,cv2.HISTCMP_BHATTACHARYYA)
            fd = np.linalg.norm(self.extract_feat(img,d.bbox)-orig_f)/self.feat_scale
            score = self.alpha_hist*hd + (1-self.alpha_hist)*fd
            if score<best_score: best_score,best_id = score,d.track_id
        return best_id if best_score<self.reid_threshold else None

    def smooth_point(self, new_pt: PointStamped) -> PointStamped:
        if self.prev_point is None:
            sm_pt = new_pt
        else:
            sm_pt = PointStamped(); sm_pt.header=new_pt.header
            sm_pt.point.x = self.alpha_smooth*self.prev_point.point.x + (1-self.alpha_smooth)*new_pt.point.x
            sm_pt.point.y = self.alpha_smooth*self.prev_point.point.y + (1-self.alpha_smooth)*new_pt.point.y
            sm_pt.point.z = self.alpha_smooth*self.prev_point.point.z + (1-self.alpha_smooth)*new_pt.point.z
        self.prev_point = sm_pt
        return sm_pt

    def rgb_cb(self, msg):
        try:
            if self.depth_frame is None or self.fx is None:
                return
            self.frame_count += 1
            if self.frame_count % (self.skip_frames+1) != 0 and self.last_frame is not None:
                dets = self.last_detections
                frame = self.last_frame.copy()
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                results = self.yolo(frame, classes=[0],imgsz=320, device=self.device, half=True, conf=0.5)[0]
                dets_ds = []
                for box in results.boxes:
                    conf = float(box.conf[0])
                    if conf < 0.5: continue
                    x1,y1,x2,y2 = map(int, box.xyxy[0].cpu().numpy())
                    dets_ds.append(((x1,y1,x2-x1,y2-y1), conf, None))
                tracks = self.tracker.update_tracks(dets_ds, frame=frame)
                dets = []
                for tr in tracks:
                    if not tr.is_confirmed(): continue
                    x1,y1,x2,y2 = map(int, tr.to_tlbr()); w,h = x2-x1, y2-y1
                    cx,cy = x1 + w/2, y1 + h/2
                    # depth gate
                    y0,y1_i = int(max(0,cy-h/4)), int(min(self.depth_frame.shape[0],cy+h/4))
                    x0,x1_i = int(max(0,cx-w/4)), int(min(self.depth_frame.shape[1],cx+w/4))
                    window = self.depth_frame[y0:y1_i, x0:x1_i].flatten()
                    valid = window[window>0]
                    if valid.size == 0: continue
                    scale = 0.001 if self.depth_frame.dtype==np.uint16 else 1.0
                    depth_m = float(np.percentile(valid,95)) * scale
                    if not (0.5 < depth_m < 5.0): continue
                    dets.append(Detection(tr.track_id, (x1,y1,w,h), (cx,cy), depth_m))
                self.last_detections = dets
                self.last_frame = frame.copy()
            if not dets:
                #disp = cv2.resize(frame, (960,540))
                cv2.imshow('Human Tracker', frame)
                cv2.waitKey(1)
                if self.last_goal: self.user_pub.publish(self.last_goal)
                return
            # re-ID & missed
            for uid in list(self.user_ids):
                tgt = next((d for d in dets if d.track_id==uid), None)
                if tgt:
                    self.missed[uid] = 0
                    self.last_centers[uid] = tgt.center
                else:
                    self.missed[uid] = self.missed.get(uid,0) + 1
                    if self.missed[uid] > self.max_missed:
                        new = self.try_reid(uid, frame, dets)
                        if new == uid:
                            self.missed[uid] = 0
                        else:
                            self.user_ids.remove(uid)
                            for d in (self.last_centers, self.orig_hists, self.orig_feats, self.missed): d.pop(uid, None)
            # 신규 할당
            for d in dets:
                if len(self.user_ids) >= self.max_users: break
                if d.track_id in self.user_ids: continue
                if any(abs(d.center[0]-self.last_centers[u][0])<self.gate_dx and abs(d.center[1]-self.last_centers[u][1])<self.gate_dy for u in self.user_ids): continue
                uid = d.track_id
                self.user_ids.append(uid)
                self.last_centers[uid] = d.center
                self.missed[uid] = 0
                x,y,w,h = map(int, d.bbox)
                roi = frame[y:y+h, x:x+w]
                self.orig_hists[uid] = self.compute_hist(roi)
                self.orig_feats[uid] = self.extract_feat(frame, d.bbox)
            # publish & visualize
            for d in dets:
                X = (d.center[0] - self.cx_cam) * d.depth / self.fx
                Y = (d.center[1] - self.cy_cam) * d.depth / self.fy
                pt = PointStamped()
                pt.header.stamp = msg.header.stamp
                pt.header.frame_id = 'camera_base'
                pt.point.x = float(d.depth)
                pt.point.y = float(-X)
                pt.point.z = float(-Y)
                if d.track_id in self.user_ids:
                    sm_pt = self.smooth_point(pt)
                    self.user_pub.publish(sm_pt)
                    self.last_goal = sm_pt
                color = self.user_color if d.track_id in self.user_ids else self.other_color
                l,t = int(d.bbox[0]), int(d.bbox[1])
                r,b = l+int(d.bbox[2]), t+int(d.bbox[3])
                cv2.rectangle(frame,(l,t),(r,b),color,2)
                cv2.putText(frame, f'ID{d.track_id} {d.depth:.2f}m', (l, t-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            disp = cv2.resize(frame, (960,540))
            cv2.imshow('Human Tracker', disp)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"[rgb_cb] exception: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
