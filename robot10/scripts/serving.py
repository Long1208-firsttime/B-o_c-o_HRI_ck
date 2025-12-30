#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox, font
import cv2
import numpy as np
import time
import random
import math
from PIL import Image, ImageTk
from datetime import datetime
import os

# =========================================================================
# CẤU HÌNH GIAO DIỆN (THEME KIOSK HIỆN ĐẠI)
# =========================================================================
C_BG_MAIN  = "#1E1E2E"       
C_BG_CARD  = "#2B2B40"       
C_ACCENT   = "#F39C12"       
C_TEXT     = "#ECF0F1"       
C_TEXT_SEC = "#95A5A6"       
C_DANGER   = "#E74C3C"       
C_SUCCESS  = "#2ECC71"       
C_TABLE_FREE = "#3498DB"     
C_VIP      = "#F1C40F"       

# =========================================================================
# KIỂM TRA THƯ VIỆN ROS
# =========================================================================
ROS_AVAILABLE = False
try:
    import rospy
    import actionlib 
    from geometry_msgs.msg import PoseStamped, Quaternion
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from actionlib_msgs.msg import GoalStatus
    ROS_AVAILABLE = True
except ImportError:
    print("Warning: ROS libraries not found. Running in UI simulation mode.")

# =========================================================================
# PHẦN 1: ROBOT FACE ANIMATOR (GIỮ NGUYÊN)
# =========================================================================
class RobotFaceAnimator:
    def __init__(self, width=1024, height=220):
        self.width = width
        self.height = height
        self.bg_color = (30, 30, 46) 
        self.default_eye_color = (255, 255, 0)
        self.current_eye_color = self.default_eye_color
        self.mouth_color = (255, 255, 0)

        self.eye_radius = int(height * 0.22)
        self.base_eye_distance = int(width * 0.15)
        self.center_y_eyes = int(height * 0.4)
        self.center_x = self.width // 2
        self.center_y_mouth = int(height * 0.8)

        self.cur_eyelid_left_top = 0.0
        self.cur_eyelid_right_top = 0.0
        self.cur_eyelid_bottom = 0.0
        self.cur_eye_angle = 0.0
        self.cur_pupil_offset_x = 0.0
        self.cur_mouth_width = 60.0
        self.cur_mouth_height = 10.0
        self.cur_mouth_curvature = 0.0 

        self.target_emotion = "neutral"
        self.next_blink_time = time.time() + 2.0
        self.is_blinking = False
        self.blink_start_time = 0
        self.last_interaction = time.time()

    def set_emotion(self, emotion_name):
        self.last_interaction = time.time()
        if self.target_emotion != emotion_name:
            self.target_emotion = emotion_name
            self.is_blinking = False 
            
            if emotion_name == "love": self.current_eye_color = (203, 192, 255) 
            elif emotion_name == "thinking": self.current_eye_color = (0, 165, 255) 
            elif emotion_name == "sleep": self.current_eye_color = (100, 100, 100) 
            elif emotion_name == "occupied": self.current_eye_color = (50, 50, 255) 
            elif emotion_name == "whistle": self.current_eye_color = (255, 255, 0)
            else: self.current_eye_color = self.default_eye_color

    def get_target_params(self):
        l_top, r_top, bot = 0.1, 0.1, 0.1
        angle, p_x = 0.0, 0.0
        m_w, m_h, m_curve = 80.0, 15.0, 0.0 

        emo = self.target_emotion
        if emo == "happy":
            l_top, r_top, bot = 0.0, 0.0, 0.6
            m_w, m_h, m_curve = 120.0, 60.0, 1.0 
        elif emo == "satisfy": 
            l_top, r_top, bot = 0.0, 0.0, 0.75 
            m_w, m_h, m_curve = 90.0, 30.0, 0.8
        elif emo == "laugh": 
            l_top, r_top, bot = 0.0, 0.0, 0.85 
            m_w, m_h, m_curve = 140.0, 100.0, 2.0 
        elif emo == "look_left":
            l_top, r_top = 0.0, 0.2
            p_x = -int(self.width * 0.12)
            m_w, m_h, m_curve = 60.0, 15.0, 0.2
        elif emo == "look_right":
            l_top, r_top = 0.0, 0.2
            p_x = int(self.width * 0.12)
            m_w, m_h, m_curve = 60.0, 15.0, 0.2
        elif emo == "confused":
            l_top, r_top = 0.6, 0.0
            angle = -8.0
            m_w, m_h, m_curve = 50.0, 20.0, -0.5
        elif emo == "wink":
            l_top, r_top, bot = 1.0, 0.0, 0.1
            m_w, m_h, m_curve = 100.0, 40.0, 0.5
        elif emo == "sleep":
            l_top, r_top, bot = 1.0, 1.0, 0.0
            m_w, m_h, m_curve = 40.0, 10.0, 0.0
        elif emo == "love":
            l_top, r_top, bot = 0.0, 0.0, 0.5
            m_w, m_h, m_curve = 100.0, 80.0, 1.5
        elif emo == "thinking":
            l_top, r_top, bot = 0.5, 0.5, 0.1
            p_x = int(self.width * 0.05)
            angle = 5.0
            m_w, m_h, m_curve = 40.0, 10.0, -0.3
        elif emo == "whistle":
            l_top, r_top, bot = 0.1, 0.1, 0.2
            m_w, m_h, m_curve = 25.0, 25.0, 0.0 
        
        return (l_top, r_top, bot, angle, p_x, m_w, m_h, m_curve)

    def update_physics(self):
        targets = self.get_target_params()
        alpha = 0.15
        self.cur_eyelid_left_top += (targets[0] - self.cur_eyelid_left_top) * alpha
        self.cur_eyelid_right_top += (targets[1] - self.cur_eyelid_right_top) * alpha
        self.cur_eyelid_bottom += (targets[2] - self.cur_eyelid_bottom) * alpha
        self.cur_eye_angle += (targets[3] - self.cur_eye_angle) * alpha
        self.cur_pupil_offset_x += (targets[4] - self.cur_pupil_offset_x) * alpha
        self.cur_mouth_width += (targets[5] - self.cur_mouth_width) * alpha
        self.cur_mouth_height += (targets[6] - self.cur_mouth_height) * alpha
        self.cur_mouth_curvature += (targets[7] - self.cur_mouth_curvature) * alpha

    def check_idle(self):
        if time.time() - self.last_interaction > 5.0:
            if self.target_emotion not in ["sleep", "wink", "whistle"]:
                anim = random.choice(["wink", "neutral", "look_left", "look_right"])
                self.set_emotion(anim)
                self.last_interaction = time.time() - 2.0 

    def process_blink(self):
        now = time.time()
        if self.target_emotion in ["sleep", "wink"]: return
        if not self.is_blinking and now > self.next_blink_time:
            self.is_blinking = True
            self.blink_start_time = now
            self.next_blink_time = now + random.uniform(2.0, 6.0)
        if self.is_blinking:
            progress = (now - self.blink_start_time) / 0.15
            if progress < 0.5: 
                self.cur_eyelid_left_top = max(self.cur_eyelid_left_top, 1.0)
                self.cur_eyelid_right_top = max(self.cur_eyelid_right_top, 1.0)
            elif progress >= 1.0: 
                self.is_blinking = False

    def draw(self):
        self.check_idle()
        self.update_physics()
        self.process_blink()
        frame = np.zeros((self.height, self.width, 3), np.uint8)
        frame[:] = self.bg_color 
        
        for is_left in [True, False]:
            sign = 1 if is_left else -1
            center_x = self.center_x - (self.base_eye_distance * sign)
            draw_x = int(center_x + self.cur_pupil_offset_x)
            draw_y = self.center_y_eyes
            radius = self.eye_radius
            
            cv2.rectangle(frame, (int(draw_x-radius), int(draw_y-radius*1.2)), 
                          (int(draw_x+radius), int(draw_y+radius*1.2)), self.current_eye_color, -1)
            
            h_eye = radius * 2.4
            top_val = self.cur_eyelid_left_top if is_left else self.cur_eyelid_right_top
            angle = self.cur_eye_angle * sign
            
            mask_w = radius + 40
            offset_tilt = int(math.tan(math.radians(angle)) * mask_w)
            y_top = int(h_eye * top_val)
            pt_tl = (draw_x - mask_w, int(draw_y - radius*1.2))
            pt_tr = (draw_x + mask_w, int(draw_y - radius*1.2))
            pt_br = (draw_x + mask_w, int(pt_tr[1] + y_top + offset_tilt))
            pt_bl = (draw_x - mask_w, int(pt_tl[1] + y_top - offset_tilt))
            cv2.fillPoly(frame, [np.array([pt_tl, pt_tr, pt_br, pt_bl], np.int32)], self.bg_color)
            
            y_bot = int(h_eye * self.cur_eyelid_bottom)
            rect_bot_y1 = int((draw_y + radius*1.2) - y_bot)
            cv2.rectangle(frame, (draw_x-mask_w, rect_bot_y1), (draw_x+mask_w, int(draw_y+radius*2)), self.bg_color, -1)

        mx, my = self.center_x, self.center_y_mouth
        mw, mh = int(self.cur_mouth_width), int(self.cur_mouth_height)
        
        if self.target_emotion == "whistle":
            cv2.circle(frame, (mx, my), int(mw/2), self.mouth_color, 3) 
        elif self.cur_mouth_curvature > 1.5: 
            cv2.ellipse(frame, (mx, my-mh//4), (mw//2, mh), 0, 0, 180, self.mouth_color, -1)
        elif abs(self.cur_mouth_curvature) < 0.1:
            cv2.rectangle(frame, (mx-mw//2, my-mh//2), (mx+mw//2, my+mh//2), self.mouth_color, -1)
        else:
            start, end, off = (0, 180, -mh//2) if self.cur_mouth_curvature > 0 else (180, 360, mh//2)
            cv2.ellipse(frame, (mx, my+off), (mw//2, mh), 0, start, end, self.mouth_color, 8)
            
        return frame

# =========================================================================
# PHẦN 2: ỨNG DỤNG ĐIỀU KHIỂN ROBOT (UI/UX)
# =========================================================================
class RobotWaiterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboWaiter Pro OS v3.5 - Vision Integrated")
        self.root.geometry("1024x768") 
        self.root.configure(bg=C_BG_MAIN)
        
        # [VISION SETUP] CẤU HÌNH CAMERA & HUMAN DETECTION
        print("[VISION] Khởi tạo Camera cho Human Detect...")
        try:
            self.cap = cv2.VideoCapture(0) # Camera Laptop
            if not self.cap.isOpened():
                print("!!! Không thể mở Camera. Chế độ Vision bị vô hiệu hóa.")
            
            face_cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(face_cascade_path)
            self.last_human_detect_time = 0
            self.is_human_detected = False
        except Exception as e:
            print(f"Lỗi khởi tạo Vision: {e}")
            self.cap = None

        self.cam_preview_frame = tk.Frame(root, bg="#000", bd=2, relief=tk.RAISED)
        self.cam_preview_frame.place(relx=1.0, rely=1.0, x=-20, y=-20, anchor="se")
        
        tk.Label(self.cam_preview_frame, text="ROBOT VISION", fg="#0F0", bg="#000", 
                 font=("Consolas", 8, "bold")).pack(fill="x")
        self.cam_label = tk.Label(self.cam_preview_frame, bg="#000")
        self.cam_label.pack()

        if ROS_AVAILABLE:
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            print("Đang kết nối tới Navigation Server (move_base)...")
            is_connected = self.move_base_client.wait_for_server(timeout=rospy.Duration(3.0))
            if is_connected:
                print(">>> KẾT NỐI THÀNH CÔNG VỚI ROBOT!")
            else:
                print("!!! CẢNH BÁO: Không tìm thấy move_base. Navigation có thể không hoạt động.")

        # Fonts
        self.font_h1 = font.Font(family="Segoe UI", size=24, weight="bold")
        self.font_h2 = font.Font(family="Segoe UI", size=16, weight="bold")
        self.font_body = font.Font(family="Segoe UI", size=11)
        self.font_btn = font.Font(family="Segoe UI", size=12, weight="bold")

        # DỮ LIỆU THỰC ĐƠN
        self.menu_items = {
            "CÀ PHÊ & TRÀ": {
                "Cà Phê Đen": 29000, "Cà Phê Sữa Đá": 35000, "Bạc Xỉu": 39000,
                "Trà Đào Cam Sả": 45000, "Trà Vải Hoa Hồng": 49000,
                "Trà Sữa Trân Châu": 42000, "Matcha Latte": 55000
            },
            "ĐÁ XAY & SINH TỐ": {
                "Cookie Đá Xay": 59000, "Chocolate Freeze": 65000,
                "Sinh Tố Bơ": 55000, "Sinh Tố Xoài": 52000, "Mojito Việt Quất": 49000
            },
            "ĐỒ ĂN NHANH": {
                "Gà Rán Giòn": 38000, "Burger Bò": 65000,
                "Burger Gà Cay": 59000, "Khoai Tây Chiên": 25000,
                "Mực Vòng Chiên": 45000, "Pizza Pepperoni": 129000
            },
            "MÓN CHÍNH": {
                "Mì Ý Bò Bằm": 79000, "Mì Ý Carbonara": 85000,
                "Cơm Gà Xối Mỡ": 69000, "Bò Bít Tết": 159000,
                "Salad Cá Ngừ": 55000
            },
            "TRÁNG MIỆNG": {
                "Bánh Croissant": 35000, "Tiramisu": 45000, "Mousse Chanh": 42000,
                "Bánh Cheese": 49000, "Kem Vani": 29000
            },
            "COMBO": {
                "Combo Sáng": 59000, "Combo Burger": 79000, 
                "Combo Hẹn Hò": 199000, "Combo Gia Đình": 399000
            }
        }
        self.current_bill = {}
        self.current_table = None
        self.table_status = {i: "free" for i in range(1, 36)}

        # LAYOUT UI
        self.face_frame = tk.Frame(root, bg=C_BG_MAIN, height=200)
        self.face_frame.pack(side="top", fill="x")
        self.face_label = tk.Label(self.face_frame, bg=C_BG_MAIN)
        self.face_label.pack(expand=True, fill="both")
        
        self.animator = RobotFaceAnimator(width=1024, height=200)
        self.animator.set_emotion("wink") 

        self.ui_frame = tk.Frame(root, bg=C_BG_MAIN)
        self.ui_frame.pack(side="bottom", fill="both", expand=True, padx=20, pady=20)

        self.update_face_loop()
        self.show_table_selection()

    # =========================================================================
    # [LOGIC] NAVIGATION - XỬ LÝ ĐIỀU HƯỚNG
    # =========================================================================
    def get_table_location(self, location_id):
        table_coords = {
            1: (-13.8376, -15.1457, 0.0),
            2: (-13.9101, -13.9183, 3.14),
            3: (-10.345, -15.241, 0.0),
            4: (-10.27, -13.668, 3.14),
            5: (-6.8782, -15.241, 0.0),
            6: (-6.64, -13.668, 3.14),
            7: (-3.54, -15.241, 0.0),
            8: (-3.05, -13.69, 3.14),
            9: (0.0, -15.17, 0.0),
            10: (3.56, -15.17, 0.0),
            11: (-8.826, -2.01, -1.57),
            12: (-8.826, 1.253, -1.57),
            13: (-7.1805, -2.0, 1.57),
            14: (-7.1805, 1.2530, 1.57),
            15: (-2.056, 16.995, -1.570),
            16: (-2.056, 19.727, -1.57),
            17: (0.2420, 16.9950, 1.57),
            18: (0.2420, 19.7270, 1.57),
            19: (5.266, -6.893, -1.57),
            20: (5.266, -2.928, -1.57),
            21: (5.266, 1.317, -1.570),
            22: (5.266, 11.01, -1.570),
            23: (5.266, 16.29, -1.570),
            24: (5.266, 20.68, -1.57),
            25: (-2.07, -2.301, -1.57),
            26: (-2.07, 1.24, -1.57),
            27: (0.13, -2.15, 1.57),
            28: (0.13, 0.57, 1.57),
            29: (-9.94, -7.450, 0),
            30: (-6.86, -7.45, 0),
            31: (-3.81, -7.45, 0),
            32: (-3.76, 12.02, 0),
            33: (-6.22,12.02, 0),
            34: (-9.11, 12.52, 0),
            # Đây là tọa độ đích cuối cùng của bàn 35, các điểm trung gian sẽ xử lý riêng
            35: (-5.950, 26.008, 0), 
            "RETURN_POINT": (-15.498122, 4.955394, 0.0) 
        }
        return table_coords.get(location_id, (0.0, 0.0, 0.0))

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def send_goal_to_coords(self, x, y, yaw):
        """Gửi tọa độ trực tiếp (x, y, yaw) xuống ROS move_base"""
        if not ROS_AVAILABLE:
            print(f"[SIM - NO ROS] Robot moving to coords: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f}")
            return
        
        try:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.position.z = 0.0
            
            q = self.euler_to_quaternion(0, 0, yaw)
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
            
            print(f"[ROS ACTION] Sending goal: X={x}, Y={y}...")
            self.move_base_client.send_goal(goal)
        except Exception as e:
            print(f"Lỗi khi gửi Navigation Goal: {e}")

    def send_navigation_goal(self, location_id):
        """Hàm cũ: Tìm ID rồi gửi (dùng cho Return Point hoặc dọn bàn)"""
        x, y, yaw = self.get_table_location(location_id)
        self.send_goal_to_coords(x, y, yaw)

    def monitor_navigation(self, on_success_callback):
        """Hàm đệ quy kiểm tra trạng thái Action Client cho đến khi Robot tới nơi"""
        if not ROS_AVAILABLE:
            # Simulation: chờ 3 giây rồi giả vờ đã đến nơi
            self.root.after(3000, on_success_callback)
            return

        state = self.move_base_client.get_state()
        if state in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
            self.root.after(500, lambda: self.monitor_navigation(on_success_callback))
        elif state == GoalStatus.SUCCEEDED:
            print(">>> Goal Reached!")
            on_success_callback()
        else:
            messagebox.showerror("Lỗi Di Chuyển", "Robot không thể tới đích! (Có thể bị kẹt hoặc mất vị trí)")
            self.show_table_selection()

    def run_waypoint_sequence(self, waypoints, on_finished):
        """
        Hàm đệ quy xử lý danh sách điểm đến.
        waypoints: List các tuple [(x1, y1, yaw1), (x2, y2, yaw2), ...]
        """
        if not waypoints:
            # Nếu danh sách rỗng, tức là đã đi hết các điểm -> Gọi callback (Mở Menu)
            on_finished()
            return

        # Lấy điểm đầu tiên trong danh sách và loại bỏ nó khỏi danh sách
        current_target = waypoints.pop(0) 
        x, y, yaw = current_target
        
        print(f"--- Bắt đầu di chuyển đến điểm trong chuỗi: {current_target}")
        
        # Gửi lệnh di chuyển
        self.send_goal_to_coords(x, y, yaw)
        
        # Chờ đến khi đến nơi, sau đó gọi lại chính hàm này với danh sách còn lại
        self.monitor_navigation(lambda: self.run_waypoint_sequence(waypoints, on_finished))

    # =========================================================================
    # [VISION] HÀM XỬ LÝ NHẬN DIỆN
    # =========================================================================
    def process_human_interaction(self):
        if self.cap is None or not self.cap.isOpened(): return
        ret, frame = self.cap.read()
        if not ret: return
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 5, minSize=(30, 30))
        current_emo = self.animator.target_emotion
        busy_states = ["thinking", "whistle", "sleep", "occupied", "confused"]
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, "HUMAN", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if len(faces) > 0:
            self.last_human_detect_time = time.time()
            if not self.is_human_detected:
                self.is_human_detected = True
                if current_emo not in busy_states: self.animator.set_emotion("love")
        else:
            if self.is_human_detected:
                if time.time() - self.last_human_detect_time > 2.0:
                    self.is_human_detected = False
                    if current_emo == "love": self.animator.set_emotion("neutral")
        vis_frame = cv2.resize(frame, (240, 180))
        vis_rgb = cv2.cvtColor(vis_frame, cv2.COLOR_BGR2RGB)
        vis_pil = Image.fromarray(vis_rgb)
        vis_tk = ImageTk.PhotoImage(image=vis_pil)
        self.cam_label.configure(image=vis_tk)
        self.cam_label.image = vis_tk

    def update_face_loop(self):
        self.process_human_interaction()
        cv_img = self.animator.draw()
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        im_pil = Image.fromarray(rgb_img)
        img_tk = ImageTk.PhotoImage(image=im_pil)
        self.face_label.configure(image=img_tk)
        self.face_label.image = img_tk
        self.root.after(30, self.update_face_loop)

    # =========================================================================
    # UI LOGIC
    # =========================================================================
    def show_table_selection(self):
        self.clear_ui()
        if self.animator.target_emotion != "whistle":
            self.animator.set_emotion("happy")

        header = tk.Frame(self.ui_frame, bg=C_BG_MAIN)
        header.pack(fill="x", pady=(0, 15))
        tk.Label(header, text="SƠ ĐỒ NHÀ HÀNG", font=self.font_h1, fg=C_ACCENT, bg=C_BG_MAIN).pack(side="left")
        
        legend = tk.Frame(header, bg=C_BG_MAIN)
        legend.pack(side="right")
        self.create_dot_legend(legend, C_TABLE_FREE, "Bàn Trống")
        self.create_dot_legend(legend, C_DANGER, "Đang Phục Vụ")

        container = tk.Frame(self.ui_frame, bg=C_BG_MAIN)
        container.pack(fill="both", expand=True)

        self.create_zone_column(container, "KHU VỰC A (1-2 NGƯỜI)", 1, 28, 0, 4)
        self.create_zone_column(container, "KHU VỰC B (GIA ĐÌNH)", 29, 34, 1, 2)
        
        f_vip = tk.Frame(container, bg=C_BG_CARD, padx=10, pady=10)
        f_vip.grid(row=0, column=2, sticky="nsew", padx=10)
        tk.Label(f_vip, text="PHÒNG VIP", font=self.font_h2, fg=C_VIP, bg=C_BG_CARD).pack(pady=10)
        self.create_table_btn(f_vip, 35, is_vip=True)
        
        container.grid_columnconfigure(0, weight=2)
        container.grid_columnconfigure(1, weight=1)
        container.grid_columnconfigure(2, weight=1)

    def create_dot_legend(self, parent, color, text):
        f = tk.Frame(parent, bg=C_BG_MAIN)
        f.pack(side="left", padx=10)
        tk.Frame(f, bg=color, width=15, height=15).pack(side="left")
        tk.Label(f, text=text, fg=C_TEXT_SEC, bg=C_BG_MAIN, font=self.font_body).pack(side="left", padx=5)

    def create_zone_column(self, parent, title, start, end, col_idx, grid_cols):
        frame = tk.Frame(parent, bg=C_BG_CARD, padx=10, pady=10)
        frame.grid(row=0, column=col_idx, sticky="nsew", padx=10)
        tk.Label(frame, text=title, font=self.font_btn, fg=C_TEXT, bg=C_BG_CARD).pack(pady=(0, 10), anchor="w")
        grid_f = tk.Frame(frame, bg=C_BG_CARD)
        grid_f.pack(fill="both", expand=True)
        count = 0
        for i in range(start, end + 1):
            r = count // grid_cols
            c = count % grid_cols
            self.create_table_btn(grid_f, i, r, c)
            grid_f.grid_columnconfigure(c, weight=1)
            count += 1

    def create_table_btn(self, parent, t_id, r=0, c=0, is_vip=False):
        status = self.table_status[t_id]
        bg_c = C_DANGER if status == "occupied" else C_TABLE_FREE
        if is_vip and status == "free": bg_c = C_VIP 
        
        txt = f"VIP {t_id}" if is_vip else f"{t_id}"
        h = 3 if is_vip else 2
        
        btn = tk.Button(parent, text=txt, font=("Segoe UI", 12, "bold"),
                        bg=bg_c, fg="white", activebackground="white", activeforeground=bg_c,
                        relief=tk.FLAT, bd=0, width=5, height=h,
                        command=lambda: self.process_table_choice(t_id))
        
        if is_vip: btn.pack(fill="both", expand=True, pady=10)
        else: btn.grid(row=r, column=c, padx=4, pady=4, sticky="nsew")
        btn.bind("<Enter>", lambda e: self.animator.set_emotion("look_left" if not is_vip else "look_right"))

    def process_table_choice(self, table_id):
        # 1. Kiểm tra trạng thái bàn (Dọn bàn nếu có khách)
        if self.table_status[table_id] == "occupied":
            self.animator.set_emotion("thinking")
            choice = messagebox.askyesno("Table Occupied", 
                                         f"Bàn {table_id} đang có khách.\nBạn muốn robot đến dọn bàn?", icon='question')
            if choice:
                self.table_status[table_id] = "free"
                self.send_navigation_goal(table_id) # Dọn bàn chỉ cần đi 1 điểm
                self.animator.set_emotion("happy")
                self.clear_ui()
                f = tk.Frame(self.ui_frame, bg=C_BG_MAIN)
                f.pack(expand=True)
                tk.Label(f, text=f"ĐANG ĐI DỌN BÀN {table_id}...", font=self.font_h1, fg=C_ACCENT, bg=C_BG_MAIN).pack(pady=20)
                self.monitor_navigation(self.show_table_selection)
            else:
                self.animator.set_emotion("neutral")
            return

        # 2. Xử lý logic chọn bàn (Di chuyển để Order)
        self.current_table = table_id
        self.current_bill = {}
        self.clear_ui()
        
        # Thiết lập UI chung "Đang di chuyển"
        self.animator.set_emotion("thinking")
        f = tk.Frame(self.ui_frame, bg=C_BG_MAIN)
        f.pack(expand=True)
        tk.Label(f, text=f"ROBOT ĐANG DI CHUYỂN ĐẾN BÀN {table_id}...", font=self.font_h1, fg=C_ACCENT, bg=C_BG_MAIN).pack(pady=20)
        tk.Label(f, text="(Vui lòng đợi Robot đến nơi)", font=self.font_body, fg=C_TEXT_SEC, bg=C_BG_MAIN).pack(pady=5)
        
        pb = ttk.Progressbar(f, orient="horizontal", length=400, mode="indeterminate")
        pb.pack(pady=10)
        pb.start(10)

        # 3. Logic Đa điểm (Multi-waypoints) cho Bàn 35
        target_waypoints = []

        if table_id == 35:
            # Định nghĩa 3 điểm cho bàn VIP 35
            p0 = (6.4, 6.5, 0.0)
            # Điểm 1: Gần cửa VIP (giả định)
            p1 = (2.11, 23.56, 0.0) 
            # Điểm 2: Giữa phòng VIP (giả định)
            p2 = (-1.37, 19.56, 1.57) 
            # Điểm 3: Tọa độ bàn chính xác (lấy từ dict)
            p3 = self.get_table_location(35)
            
            target_waypoints = [p0, p1, p2, p3]
            print(f"Xác nhận bàn VIP 35: Kích hoạt chuỗi 3 điểm di chuyển.")
        else:
            # Các bàn thường chỉ có 1 điểm
            target_waypoints = [self.get_table_location(table_id)]

        # 4. Gọi hàm xử lý chuỗi di chuyển (recursive)
        # Hàm run_waypoint_sequence sẽ chạy ngầm, UI vẫn giữ nguyên là "Đang di chuyển"
        # cho đến khi điểm cuối cùng hoàn tất, lúc đó mới gọi self.show_order_menu
        self.run_waypoint_sequence(target_waypoints, self.show_order_menu)

    def show_order_menu(self):
        self.clear_ui()
        self.animator.set_emotion("happy")

        sidebar = tk.Frame(self.ui_frame, bg=C_BG_CARD, width=220)
        sidebar.pack(side="left", fill="y", padx=(0, 10))
        sidebar.pack_propagate(False)

        tk.Button(sidebar, text="← QUAY LẠI", bg=C_DANGER, fg="white", relief=tk.FLAT, font=self.font_btn, height=2,
                  command=self.show_table_selection).pack(fill="x", pady=(0, 20))
        
        main_content = tk.Frame(self.ui_frame, bg=C_BG_MAIN)
        main_content.pack(side="left", fill="both", expand=True)
        
        self.lbl_cat_title = tk.Label(main_content, text="THỰC ĐƠN", font=self.font_h2, fg=C_TEXT, bg=C_BG_MAIN)
        self.lbl_cat_title.pack(anchor="w", pady=(0, 10))

        grid_frame = tk.Frame(main_content, bg=C_BG_MAIN)
        grid_frame.pack(fill="both", expand=True)

        bill_panel = tk.Frame(self.ui_frame, bg=C_BG_CARD, width=300)
        bill_panel.pack(side="right", fill="y", padx=(10, 0))
        bill_panel.pack_propagate(False)

        tk.Label(bill_panel, text=f"BÀN {self.current_table}", font=self.font_h2, fg=C_ACCENT, bg=C_BG_CARD).pack(pady=10)
        
        self.listbox = tk.Listbox(bill_panel, bg="#333", fg="white", font=("Consolas", 11), bd=0, highlightthickness=0)
        self.listbox.pack(fill="both", expand=True, padx=10)

        self.lbl_total = tk.Label(bill_panel, text="0 đ", font=self.font_h1, fg=C_SUCCESS, bg=C_BG_CARD)
        self.lbl_total.pack(pady=10)
        
        btn_checkout = tk.Button(bill_panel, text="XÁC NHẬN GỬI BẾP", bg=C_SUCCESS, fg="white", font=self.font_btn, relief=tk.FLAT, height=3,
                                 command=self.checkout)
        btn_checkout.pack(fill="x", side="bottom")
        
        def load_category(cat_name):
            for w in grid_frame.winfo_children(): w.destroy()
            self.lbl_cat_title.config(text=cat_name)
            items = self.menu_items.get(cat_name, {})
            r, c = 0, 0
            for name, price in items.items():
                f_item = tk.Frame(grid_frame, bg=C_BG_CARD, padx=5, pady=5)
                f_item.grid(row=r, column=c, padx=5, pady=5, sticky="nsew")
                btn = tk.Button(f_item, text=f"{name}\n{price:,}đ", font=self.font_body,
                                bg=C_BG_CARD, fg=C_TEXT, relief=tk.FLAT,
                                activebackground=C_ACCENT, activeforeground="white",
                                width=16, height=3,
                                command=lambda n=name, p=price: self.add_item(n, p))
                btn.pack(fill="both", expand=True)
                c += 1
                if c > 2: c=0; r+=1
            grid_frame.grid_columnconfigure(0, weight=1)
            grid_frame.grid_columnconfigure(1, weight=1)
            grid_frame.grid_columnconfigure(2, weight=1)

        categories = list(self.menu_items.keys())
        for cat in categories:
            btn = tk.Button(sidebar, text=cat, font=self.font_btn, bg=C_BG_CARD, fg=C_TEXT_SEC, 
                            relief=tk.FLAT, pady=10, activebackground=C_BG_MAIN, activeforeground=C_ACCENT,
                            anchor="w", padx=20,
                            command=lambda c=cat: load_category(c))
            btn.pack(fill="x")
        
        if categories: load_category(categories[0])

    def add_item(self, name, price):
        if name in self.current_bill:
            self.current_bill[name]['qty'] += 1
        else:
            self.current_bill[name] = {'price': price, 'qty': 1}
        
        total = sum(item['price'] * item['qty'] for item in self.current_bill.values())
        if price >= 100000 or "Combo" in name: self.animator.set_emotion("love")
        elif total > 200000: self.animator.set_emotion("laugh")
        else: self.animator.set_emotion("satisfy")
        self.root.after(1000, lambda: self.animator.set_emotion("happy"))
        self.refresh_bill_ui()

    def refresh_bill_ui(self):
        self.listbox.delete(0, tk.END)
        total = 0
        for name, info in self.current_bill.items():
            qty = info['qty']
            p = info['price']
            sub = p * qty
            total += sub
            self.listbox.insert(tk.END, f"{name}")
            self.listbox.insert(tk.END, f"  x{qty}   = {sub:,}")
            self.listbox.insert(tk.END, "-"*30)
        self.lbl_total.config(text=f"{total:,} đ")

    def checkout(self):
        if not self.current_bill:
            self.animator.set_emotion("confused")
            messagebox.showwarning("Oops", "Bạn chưa chọn món nào!")
            self.animator.set_emotion("happy")
            return
        
        self.animator.set_emotion("whistle")
        total = sum(item['price'] * item['qty'] for item in self.current_bill.values())
        self.table_status[self.current_table] = "occupied"
        
        messagebox.showinfo("Thành công", f"Đã gửi đơn! Tổng: {total:,} đ\nRobot sẽ quay về vị trí chờ.")
        
        # Quay về bếp (thường chỉ 1 điểm)
        self.send_navigation_goal("RETURN_POINT")
        
        self.clear_ui()
        f = tk.Frame(self.ui_frame, bg=C_BG_MAIN)
        f.pack(expand=True)
        tk.Label(f, text="ROBOT ĐANG QUAY VỀ BẾP...", font=self.font_h1, fg=C_ACCENT, bg=C_BG_MAIN).pack(pady=20)
        pb = ttk.Progressbar(f, orient="horizontal", length=400, mode="indeterminate")
        pb.pack()
        pb.start(10)
        
        self.monitor_navigation(self.show_table_selection)

    def clear_ui(self):
        for widget in self.ui_frame.winfo_children():
            widget.destroy()

if __name__ == "__main__":
    if ROS_AVAILABLE:
        try: 
            rospy.init_node('robot_waiter_kiosk_final', anonymous=True, disable_signals=True)
        except Exception as e: 
            print(f"ROS Init Error: {e}")
            
    root = tk.Tk()
    app = RobotWaiterApp(root)
    root.mainloop()
