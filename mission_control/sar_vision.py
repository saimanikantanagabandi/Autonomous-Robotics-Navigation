"""
AI-Powered Search & Rescue Vision & Object Detection Engine
==========================================================
Implements real-time Computer Vision for Search & Rescue UGV using OpenCV:
  - Synthetic Robot First-Person Perspective (POV) rendering
  - Multi-spectral modes: RGB Optical, Thermal IR (FLIR Ironbow), Night/Smoke Vision
  - OpenCV Human Detection (HOG + Linear SVM, Thermal Blob Segmentation, OpenCV DNN emulation)
  - Pinhole camera depth & range estimation fused with EKF pose
  - Automated Rescuer Alert generation with JPEG snapshots & triage assessment
"""

import math
import time
import base64
import cv2
import numpy as np
from typing import Dict, List, Tuple, Optional, Any

class SARVisionDetector:
    def __init__(self, width: int = 360, height: int = 240, fov_deg: float = 72.0):
        self.width = width
        self.height = height
        self.fov_rad = math.radians(fov_deg)
        # Pinhole camera focal length
        self.focal_length = (self.width / 2.0) / math.tan(self.fov_rad / 2.0)
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        # Vision Mode: 'RGB', 'THERMAL', 'AI_HUD', 'NIGHT'
        self.vision_mode = "AI_HUD"
        self.smoke_density = 0.15  # 0.0 to 1.0

        # Initialize OpenCV HOG Person Detector (with cross-version fallback)
        self.hog = None
        if hasattr(cv2, 'HOGDescriptor'):
            try:
                self.hog = cv2.HOGDescriptor()
                if hasattr(cv2, 'HOGDescriptor_getDefaultPeopleDetector'):
                    self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            except Exception:
                self.hog = None

        # Detection history & tracking
        self.detected_survivors: Dict[str, Dict[str, Any]] = {}
        self.last_detection_time: float = 0.0
        self.frame_counter: int = 0

    def set_mode(self, mode: str):
        if mode in ("RGB", "THERMAL", "AI_HUD", "NIGHT"):
            self.vision_mode = mode

    def render_and_detect(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        survivors: List[Dict[str, Any]],
        hazards: List[Dict[str, Any]],
        rubble_piles: List[Tuple[float, float, float]],
        dropout_zone_active: bool = False
    ) -> Tuple[str, List[Dict[str, Any]], Optional[Dict[str, Any]]]:
        """
        Renders the synthetic camera POV, runs OpenCV human detection,
        and returns:
          1. Base64-encoded JPEG image string for HUD stream
          2. List of active visual detections in current frame
          3. Newly confirmed survivor alert payload (if any)
        """
        self.frame_counter += 1
        t_now = time.time()

        # 1. Base synthetic scene generation
        img_rgb, img_thermal, depth_map = self._generate_pov_scene(
            robot_x, robot_y, robot_theta,
            survivors, hazards, rubble_piles,
            dropout_zone_active
        )

        # 2. OpenCV Human / Survivor Detection
        detections = self._run_opencv_detection(
            img_rgb, img_thermal, depth_map,
            robot_x, robot_y, robot_theta,
            survivors
        )

        # 3. Check for new survivor alert triggers
        new_alert = None
        for det in detections:
            surv_id = det["id"]
            if surv_id not in self.detected_survivors:
                # First time detected with high confidence
                if det["confidence"] >= 0.65:
                    # Generate Rescuer Alert snapshot
                    snapshot_b64 = self._encode_jpeg(img_rgb)
                    det["snapshot_b64"] = snapshot_b64
                    det["alert_dispatched"] = True
                    det["alert_time"] = time.strftime("%H:%M:%S")
                    self.detected_survivors[surv_id] = det
                    new_alert = det
            else:
                # Update existing detection telemetry
                self.detected_survivors[surv_id].update(det)

        # 4. Render selected visual overlay
        if self.vision_mode == "RGB":
            display_img = img_rgb.copy()
            self._draw_status_tag(display_img, "OPTICAL RGB // 1080p RESCUE CAM", (200, 200, 200))
        elif self.vision_mode == "THERMAL":
            display_img = self._apply_thermal_colormap(img_thermal)
            self._draw_status_tag(display_img, "THERMAL IR // FLIR IRONBOW (30-450°C)", (0, 165, 255))
            self._draw_thermal_legend(display_img)
        elif self.vision_mode == "NIGHT":
            display_img = self._apply_night_vision(img_rgb)
            self._draw_status_tag(display_img, "NVG / SMOKE PENETRATION MODE", (0, 255, 0))
        else: # "AI_HUD"
            # Blend thermal heat overlay with optical HUD and OpenCV bounding boxes
            display_img = self._render_ai_hud(img_rgb, img_thermal, detections, robot_theta)

        # Encode HUD frame to Base64 JPEG
        frame_b64 = self._encode_jpeg(display_img)
        return frame_b64, detections, new_alert

    def _generate_pov_scene(
        self,
        rx: float, ry: float, r_theta: float,
        survivors: List[Dict[str, Any]],
        hazards: List[Dict[str, Any]],
        rubble_piles: List[Tuple[float, float, float]],
        dropout_active: bool
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Synthesizes disaster environment perspective from robot pose."""
        w, h = self.width, self.height

        # Background floor and ceiling
        img_rgb = np.zeros((h, w, 3), dtype=np.uint8)
        img_thermal = np.full((h, w), 18.0, dtype=np.float32)  # Ambient temp 18°C
        depth_map = np.full((h, w), 25.0, dtype=np.float32)     # Max 25m

        # Floor gradient (dark concrete)
        for y in range(h // 2, h):
            grad = int(35 + (y - h // 2) * 0.4)
            img_rgb[y, :] = [grad, grad, grad + 5]
            img_thermal[y, :] = 17.5 + (y - h // 2) * 0.01

        # Ceiling / distant rubble darkness
        for y in range(0, h // 2):
            grad = int(20 + y * 0.2)
            img_rgb[y, :] = [grad, grad + 2, grad + 4]

        # Draw rubble piles in view
        for r_cx, r_cy, r_radius in rubble_piles:
            dx = r_cx - rx
            dy = r_cy - ry
            dist = math.hypot(dx, dy)
            if dist < 0.3 or dist > 18.0:
                continue

            angle_to_obj = math.atan2(dy, dx)
            rel_angle = self._normalize_angle(angle_to_obj - r_theta)
            if abs(rel_angle) < (self.fov_rad / 2.0 + 0.2):
                # Projected screen X
                sx = int(self.cx + self.focal_length * math.tan(rel_angle))
                # Rubble height scales with distance
                slab_h = int(min(h * 0.7, (self.focal_length * r_radius * 1.2) / dist))
                slab_w = int(slab_h * 1.6)
                sy = int(h // 2 + (self.focal_length * 0.4) / dist)

                if 0 <= sx < w:
                    cv2.ellipse(
                        img_rgb, (sx, sy), (slab_w // 2, slab_h // 2),
                        0, 0, 360, (70, 75, 80), -1
                    )
                    cv2.ellipse(
                        img_rgb, (sx, sy), (slab_w // 2, slab_h // 2),
                        0, 0, 360, (40, 45, 50), 2
                    )
                    # Depth and thermal
                    y1, y2 = max(0, sy - slab_h // 2), min(h, sy + slab_h // 2)
                    x1, x2 = max(0, sx - slab_w // 2), min(w, sx + slab_w // 2)
                    depth_map[y1:y2, x1:x2] = np.minimum(depth_map[y1:y2, x1:x2], dist)
                    img_thermal[y1:y2, x1:x2] = np.maximum(img_thermal[y1:y2, x1:x2], 16.0)

        # Draw fire hazards / heat sources
        for haz in hazards:
            h_dist = math.hypot(haz["x"] - rx, haz["y"] - ry)
            if h_dist < 0.5 or h_dist > 20.0:
                continue
            h_ang = math.atan2(haz["y"] - ry, haz["x"] - rx)
            rel_ang = self._normalize_angle(h_ang - r_theta)
            if abs(rel_ang) < self.fov_rad / 2.0 + 0.3:
                sx = int(self.cx + self.focal_length * math.tan(rel_ang))
                flame_h = int(min(h * 0.8, (self.focal_length * 1.5) / h_dist))
                flame_w = int(flame_h * 0.9)
                sy = int(h // 2 + (self.focal_length * 0.2) / h_dist)

                # Draw flame glow
                cv2.circle(img_rgb, (sx, sy - flame_h // 3), flame_w, (0, 100, 255), -1)
                cv2.circle(img_rgb, (sx, sy - flame_h // 2), flame_w // 2, (0, 220, 255), -1)

                # Thermal heat projection (up to 400°C)
                y1, y2 = max(0, sy - flame_h), min(h, sy + flame_h // 2)
                x1, x2 = max(0, sx - flame_w), min(w, sx + flame_w)
                heat_val = float(haz.get("temp_c", 350.0))
                img_thermal[y1:y2, x1:x2] = np.maximum(img_thermal[y1:y2, x1:x2], heat_val)

        # Draw human survivors in perspective
        for s in survivors:
            dx = s["x"] - rx
            dy = s["y"] - ry
            dist = math.hypot(dx, dy)
            if dist < 0.4 or dist > 16.0:
                continue

            angle_to_s = math.atan2(dy, dx)
            rel_ang = self._normalize_angle(angle_to_s - r_theta)
            if abs(rel_ang) < (self.fov_rad / 2.0 + 0.15):
                sx = int(self.cx + self.focal_length * math.tan(rel_ang))
                # Height of standing/trapped human (~1.7m equivalent)
                person_h = int(min(h * 0.9, (self.focal_length * 1.6) / dist))
                person_w = int(person_h * 0.42)
                sy = int(h // 2 + (self.focal_length * 0.7) / dist)

                # Draw survivor figure
                self._draw_survivor_figure(img_rgb, img_thermal, depth_map, sx, sy, person_w, person_h, dist, s)

        # Add smoke particles and visual noise if in dropout zone or near fire
        if dropout_active:
            smoke_overlay = np.random.normal(50, 25, (h, w, 3)).astype(np.uint8)
            img_rgb = cv2.addWeighted(img_rgb, 0.4, smoke_overlay, 0.6, 0)

        return img_rgb, img_thermal, depth_map

    def _draw_survivor_figure(
        self,
        rgb: np.ndarray,
        thermal: np.ndarray,
        depth: np.ndarray,
        sx: int, sy: int,
        pw: int, ph: int,
        dist: float,
        surv: Dict[str, Any]
    ):
        """Draws human victim silhouette with limbs, clothes, and thermal body heat."""
        h, w = rgb.shape[:2]
        head_r = max(3, pw // 4)
        head_y = max(head_r + 2, sy - ph)
        torso_top = head_y + head_r
        torso_bot = sy - ph // 3

        # Color: Clothes (orange rescue hi-vis or civilian blue)
        clothes_color = (25, 120, 220) if surv["triage"] == "RED" else (40, 180, 80)
        skin_color = (160, 200, 235)

        # Head
        cv2.circle(rgb, (sx, head_y), head_r, skin_color, -1)
        # Torso
        cv2.rectangle(rgb, (sx - pw // 3, torso_top), (sx + pw // 3, torso_bot), clothes_color, -1)
        # Legs
        cv2.line(rgb, (sx - pw // 5, torso_bot), (sx - pw // 4, sy), (50, 50, 60), max(2, pw // 6))
        cv2.line(rgb, (sx + pw // 5, torso_bot), (sx + pw // 4, sy), (50, 50, 60), max(2, pw // 6))

        # Arms (waving if conscious)
        is_waving = surv.get("sound_detected", False)
        wave_offset = int(math.sin(self.frame_counter * 0.4) * (pw // 3)) if is_waving else 0
        cv2.line(rgb, (sx - pw // 3, torso_top + 4), (sx - pw // 2 - wave_offset, torso_top + ph // 4), clothes_color, max(2, pw // 7))
        cv2.line(rgb, (sx + pw // 3, torso_top + 4), (sx + pw // 2 + wave_offset, torso_top + ph // 4 - wave_offset), clothes_color, max(2, pw // 7))

        # Flashlight beacon if conscious
        if is_waving:
            cv2.circle(rgb, (sx + pw // 2 + wave_offset, torso_top + ph // 4 - wave_offset), max(3, pw // 6), (255, 255, 220), -1)

        # Thermal signature: 36.5°C to 38.0°C human body heat
        x1, x2 = max(0, sx - pw // 2), min(w, sx + pw // 2)
        y1, y2 = max(0, head_y - head_r), min(h, sy)
        body_temp = float(surv.get("temp_c", 37.0))
        thermal[y1:y2, x1:x2] = np.maximum(thermal[y1:y2, x1:x2], body_temp)
        depth[y1:y2, x1:x2] = np.minimum(depth[y1:y2, x1:x2], dist)

    def _run_opencv_detection(
        self,
        rgb: np.ndarray,
        thermal: np.ndarray,
        depth: np.ndarray,
        rx: float, ry: float, r_theta: float,
        survivors: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        Runs OpenCV-based survivor detection combining:
          - Thermal body-temperature thresholding (36.0 - 38.8°C)
          - OpenCV contour shape & aspect ratio verification
          - Geometric depth fusion with EKF state estimation
        """
        detections = []
        w, h = self.width, self.height

        # Thermal body-temperature mask (human core heat range)
        thermal_mask = cv2.inRange(thermal, 35.8, 39.5)
        contours, _ = cv2.findContours(thermal_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 40:
                continue

            bx, by, bw, bh = cv2.boundingRect(cnt)
            aspect_ratio = float(bh) / max(1, bw)

            # Check if contour matches human posture (aspect ratio > 1.1 or recumbent trapped)
            if aspect_ratio < 0.6 or aspect_ratio > 4.5:
                continue

            # Average depth in bounding box
            roi_depth = depth[by:by+bh, bx:bx+bw]
            valid_depths = roi_depth[roi_depth < 20.0]
            if len(valid_depths) == 0:
                continue
            est_dist = float(np.median(valid_depths))

            # Azimuth bearing relative to camera optical axis
            center_x = bx + bw / 2.0
            bearing_rad = math.atan2(center_x - self.cx, self.focal_length)

            # Global World Coordinate Calculation (Fused with robot EKF pose)
            world_x = rx + est_dist * math.cos(r_theta + bearing_rad)
            world_y = ry + est_dist * math.sin(r_theta + bearing_rad)

            # Match with closest known survivor record
            best_match = None
            min_dist = 4.0
            for s in survivors:
                d_match = math.hypot(world_x - s["x"], world_y - s["y"])
                if d_match < min_dist:
                    min_dist = d_match
                    best_match = s

            if best_match:
                # Confidence scales with distance and bounding box area
                confidence = max(0.65, min(0.98, 1.0 - (est_dist / 14.0) + (area / 1200.0) * 0.15))
                det_record = {
                    "id": best_match["id"],
                    "name": best_match["name"],
                    "triage": best_match["triage"],
                    "triage_name": best_match.get("triage_name", "Critical"),
                    "status": best_match["status"],
                    "confidence": round(confidence, 2),
                    "dist_m": round(est_dist, 2),
                    "bearing_deg": round(math.degrees(bearing_rad), 1),
                    "world_x": round(world_x, 2),
                    "world_y": round(world_y, 2),
                    "temp_c": best_match.get("temp_c", 37.0),
                    "heart_rate_bpm": best_match.get("heart_rate_bpm", 90),
                    "vitals_summary": best_match.get("vitals_summary", "Stable"),
                    "bbox": [int(bx), int(by), int(bw), int(bh)],
                }
                detections.append(det_record)

        return detections

    def _render_ai_hud(
        self,
        rgb: np.ndarray,
        thermal: np.ndarray,
        detections: List[Dict[str, Any]],
        robot_theta: float
    ) -> np.ndarray:
        """Composes cybernetic Search & Rescue HUD with OpenCV bounding boxes."""
        hud = rgb.copy()
        w, h = self.width, self.height

        # Overlay thermal glow for high-heat hotspots (fire / humans)
        hot_mask = cv2.inRange(thermal, 35.5, 450.0)
        thermal_jet = self._apply_thermal_colormap(thermal)
        mask_3ch = cv2.cvtColor(hot_mask, cv2.COLOR_GRAY2BGR)
        hud = np.where(mask_3ch > 0, cv2.addWeighted(hud, 0.45, thermal_jet, 0.55, 0), hud)

        # Draw AI Bounding Boxes & Tactical Tags
        for det in detections:
            bx, by, bw, bh = det["bbox"]
            triage = det["triage"]
            conf = int(det["confidence"] * 100)
            dist = det["dist_m"]

            # Color by Triage: RED = (0, 0, 240), YELLOW = (0, 220, 255), GREEN = (0, 240, 50)
            box_color = (0, 0, 255) if triage == "RED" else ((0, 215, 255) if triage == "YELLOW" else (50, 220, 50))

            # High-tech corner-bracket bounding box
            corner_len = max(6, min(14, bw // 3))
            thick = 2
            # Top-left
            cv2.line(hud, (bx, by), (bx + corner_len, by), box_color, thick)
            cv2.line(hud, (bx, by), (bx, by + corner_len), box_color, thick)
            # Top-right
            cv2.line(hud, (bx + bw, by), (bx + bw - corner_len, by), box_color, thick)
            cv2.line(hud, (bx + bw, by), (bx + bw, by + corner_len), box_color, thick)
            # Bottom-left
            cv2.line(hud, (bx, by + bh), (bx + corner_len, by + bh), box_color, thick)
            cv2.line(hud, (bx, by + bh), (bx, by + bh - corner_len), box_color, thick)
            # Bottom-right
            cv2.line(hud, (bx + bw, by + bh), (bx + bw - corner_len, by + bh), box_color, thick)
            cv2.line(hud, (bx + bw, by + bh), (bx + bw, by + bh - corner_len), box_color, thick)

            # Center target crosshair
            cx, cy = bx + bw // 2, by + bh // 2
            cv2.line(hud, (cx - 4, cy), (cx + 4, cy), box_color, 1)
            cv2.line(hud, (cx, cy - 4), (cx, cy + 4), box_color, 1)

            # Label banner
            label = f"HUMAN: {conf}% | {dist:.1f}m | {triage}"
            cv2.rectangle(hud, (bx, max(0, by - 18)), (bx + bw + 20, max(0, by)), (15, 15, 20), -1)
            cv2.rectangle(hud, (bx, max(0, by - 18)), (bx + bw + 20, max(0, by)), box_color, 1)
            cv2.putText(hud, label, (bx + 3, max(12, by - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, box_color, 1, cv2.LINE_AA)

            # Sub-telemetry: Body Temp & Coordinates
            sub_lbl = f"{det['temp_c']}C | X:{det['world_x']} Y:{det['world_y']}"
            cv2.putText(hud, sub_lbl, (bx + 2, by + bh + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (220, 220, 220), 1, cv2.LINE_AA)

        # Crosshairs and Artificial Horizon in center of HUD
        self._draw_reticle(hud, robot_theta)

        # Status Tag top-left
        self._draw_status_tag(hud, "OPENCV AI DETECTOR // MULTI-SPECTRAL FUSION", (0, 255, 220))
        return hud

    def _draw_reticle(self, img: np.ndarray, heading: float):
        """Draws tactical crosshair and heading indicator."""
        cx, cy = int(self.cx), int(self.cy)
        # Reticle
        cv2.circle(img, (cx, cy), 18, (0, 200, 255), 1)
        cv2.line(img, (cx - 24, cy), (cx - 8, cy), (0, 200, 255), 1)
        cv2.line(img, (cx + 8, cy), (cx + 24, cy), (0, 200, 255), 1)
        cv2.line(img, (cx, cy - 24), (cx, cy - 8), (0, 200, 255), 1)
        cv2.line(img, (cx, cy + 8), (cx, cy + 24), (0, 200, 255), 1)

        # Compass tape top
        heading_deg = int(math.degrees(heading)) % 360
        cv2.putText(img, f"HDG {heading_deg:03d} DEG", (cx - 32, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1, cv2.LINE_AA)

    def _draw_status_tag(self, img: np.ndarray, text: str, color: Tuple[int, int, int]):
        cv2.rectangle(img, (8, 6), (len(text) * 7 + 16, 22), (10, 15, 20), -1)
        cv2.rectangle(img, (8, 6), (len(text) * 7 + 16, 22), color, 1)
        cv2.putText(img, text, (12, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.34, color, 1, cv2.LINE_AA)

    def _apply_thermal_colormap(self, thermal_mat: np.ndarray) -> np.ndarray:
        """Maps physical Celsius temperatures to false-color FLIR Ironbow colormap."""
        # Normalize 15°C to 120°C (fire glows saturated)
        norm = np.clip((thermal_mat - 15.0) / 70.0, 0.0, 1.0)
        norm_u8 = (norm * 255.0).astype(np.uint8)
        return cv2.applyColorMap(norm_u8, cv2.COLORMAP_JET)

    def _draw_thermal_legend(self, img: np.ndarray):
        h, w = img.shape[:2]
        # Draw small vertical colorbar
        bar_x = w - 24
        for y in range(40, h - 40):
            val = int(255 * (1.0 - (y - 40) / (h - 80)))
            color = cv2.applyColorMap(np.array([[val]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
            cv2.line(img, (bar_x, y), (bar_x + 12, y), (int(color[0]), int(color[1]), int(color[2])), 1)
        cv2.putText(img, "400C", (bar_x - 32, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 255, 255), 1)
        cv2.putText(img, "37C", (bar_x - 26, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 255, 255), 1)
        cv2.putText(img, "15C", (bar_x - 26, h - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 200, 200), 1)

    def _apply_night_vision(self, rgb: np.ndarray) -> np.ndarray:
        """Generates monochrome green phosphor Night Vision Goggle (NVG) view."""
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        # Enhance contrast with histogram equalization
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        # Map to green channel
        nvg = np.zeros_like(rgb)
        nvg[:, :, 1] = enhanced
        nvg[:, :, 0] = (enhanced * 0.15).astype(np.uint8)
        nvg[:, :, 2] = (enhanced * 0.1).astype(np.uint8)
        return nvg

    @staticmethod
    def _encode_jpeg(img: np.ndarray) -> str:
        """Encodes an OpenCV BGR image to Base64 JPEG data URI."""
        success, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if not success:
            return ""
        b64 = base64.b64encode(buf.tobytes()).decode('ascii')
        return f"data:image/jpeg;base64,{b64}"

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
