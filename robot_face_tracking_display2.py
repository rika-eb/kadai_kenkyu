# robot_face_tracking.py
import os
os.environ["QT_QPA_PLATFORM"] = "offscreen"

import cv2
import numpy as np
import pygame
import time
import sys
import threading
from pdf2image import convert_from_path
from motor import tankMotor
from ultrasonic import Ultrasonic
from camera import Camera

# ===================================================
# 設定
# ===================================================
PDF_PATHS = [
    os.path.expanduser("~/kadai_kenkyu/happy.pdf"),
    os.path.expanduser("~/kadai_kenkyu/chou_happy.pdf"),
]
PNG_DIR = os.path.expanduser("~/kadai_kenkyu/png_cache/")

FACE_NORMAL = 0
FACE_HAPPY  = 1

# 設定に追加
APPROACH_TIME = 0.1  # 前進時間（秒）
TARGET_DISTANCE    = 30
TOLERANCE_DISTANCE = 5
APPROACH_SPEED     = 2000
TURN_SPEED         = 2000
TURN_STEP_TIME     = 0.28   # 左右に振る1回あたりの時間（秒）
WIGGLE_COUNT       = 3      # 左右に振る回数（片道1回としてカウント）
FACE_LOST_TIMEOUT  = 2.0

# ===================================================
# PDF → PNG 変換
# ===================================================
def convert_pdfs_to_png(pdf_paths, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    png_paths = []
    for pdf_path in pdf_paths:
        base_name = os.path.splitext(os.path.basename(pdf_path))[0]
        png_path  = os.path.join(out_dir, f"{base_name}.png")
        if os.path.exists(png_path):
            print(f"スキップ（変換済み）: {png_path}")
        else:
            print(f"変換中: {pdf_path} → {png_path}")
            pages = convert_from_path(pdf_path, dpi=150)
            pages[0].save(png_path, "PNG")
            print(f"変換完了: {png_path}")
        png_paths.append(png_path)
    return png_paths

# ===================================================
# pygame 初期化・画像読み込み
# ===================================================
def init_display(png_paths):
    pygame.init()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.mouse.set_visible(False)
    w, h = screen.get_size()
    images = []
    for path in png_paths:
        img = pygame.image.load(path).convert()
        img = pygame.transform.rotate(img, 90)
        img = pygame.transform.scale(img, (w, h))
        images.append(img)
    return screen, images

def show_face(screen, images, index):
    screen.blit(images[index], (0, 0))
    pygame.display.flip()

# ===================================================
# ロボット制御クラス
# ===================================================
class FaceTrackingRobot:
    def __init__(self, screen, images):
        self.screen = screen
        self.images = images

        self.motor      = tankMotor()
        self.ultrasonic = Ultrasonic()
        self.camera     = Camera()

        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        self.cam_w    = 640
        self.cam_h    = 480

        self.running       = True
        self.current_frame = None
        self.frame_lock    = threading.Lock()
        self.last_face_time = 0.0
        self.last_drawn     = -1

    # --------------------------------------------------
    # カメラスレッド
    # --------------------------------------------------
    def _capture_loop(self):
        self.camera.start_stream()
        while self.running:
            try:
                data = self.camera.get_frame()
                if data:
                    arr   = np.frombuffer(data, np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    with self.frame_lock:
                        self.current_frame = frame
            except Exception as e:
                print(f"[カメラ] {e}")
        self.camera.stop_stream()

    # --------------------------------------------------
    # 顔検出
    # --------------------------------------------------
    def _detect_face(self, frame):
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )
        if len(faces) > 0:
            x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
            return x + w // 2, y + h // 2, w, h
        return None, None, None, None

    # --------------------------------------------------
    # 左右に振る（ブロッキング）
    # --------------------------------------------------
    def _wiggle(self):
        """左右にWIGGLE_COUNT回振ってから停止する。"""
        print(f"振り動作開始（{WIGGLE_COUNT}回）")
        direction = "right"
        for i in range(WIGGLE_COUNT):
            if direction == "right":
                self.motor.setMotorModel(TURN_SPEED, -TURN_SPEED)
                print(f"  振り {i+1}/{WIGGLE_COUNT}: 右")
                direction = "left"
            else:
                self.motor.setMotorModel(-TURN_SPEED, TURN_SPEED)
                print(f"  振り {i+1}/{WIGGLE_COUNT}: 左")
                direction = "right"
            time.sleep(TURN_STEP_TIME)

        self.motor.setMotorModel(0, 0)
        print("振り動作完了")

    # --------------------------------------------------
    # 前進（距離確認つき）
    # --------------------------------------------------
    def _approach(self):
        """APPROACH_TIME秒だけ前進する。"""
        dist = self.ultrasonic.get_distance()

        if dist != 0 and dist < TARGET_DISTANCE - TOLERANCE_DISTANCE:
            print(f"近すぎるため前進しない: 距離={dist:.1f}cm")
            self.motor.setMotorModel(0, 0)
            return

        print(f"前進: {APPROACH_TIME}秒 (距離={dist:.1f}cm)")
        self.motor.setMotorModel(APPROACH_SPEED, APPROACH_SPEED)
        time.sleep(APPROACH_TIME)
        self.motor.setMotorModel(0, 0)
        
        # """目標距離になるまで前進する。"""
        # dist = self.ultrasonic.get_distance()
        # if dist == 0:
        #     print("距離取得失敗: 停止")
        #     self.motor.setMotorModel(0, 0)
        #     return

        # if dist > TARGET_DISTANCE + TOLERANCE_DISTANCE:
        #     print(f"前進: 距離={dist:.1f}cm → 目標={TARGET_DISTANCE}cm")
        #     self.motor.setMotorModel(APPROACH_SPEED, APPROACH_SPEED)
        #     time.sleep(0.3)
        #     self.motor.setMotorModel(0, 0)
        # elif dist < TARGET_DISTANCE - TOLERANCE_DISTANCE:
        #     print(f"後退: 距離={dist:.1f}cm → 目標={TARGET_DISTANCE}cm")
        #     self.motor.setMotorModel(-APPROACH_SPEED, -APPROACH_SPEED)
        #     time.sleep(0.3)
        #     self.motor.setMotorModel(0, 0)
        # else:
        #     print(f"適切な距離: {dist:.1f}cm → 停止")
        #     self.motor.setMotorModel(0, 0)

    # --------------------------------------------------
    # pygameイベント確認（ループ中断チェック用）
    # --------------------------------------------------
    def _check_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                raise KeyboardInterrupt

    # --------------------------------------------------
    # メインループ
    # --------------------------------------------------
    def start(self):
        t = threading.Thread(target=self._capture_loop, daemon=True)
        t.start()
        time.sleep(2)

        show_face(self.screen, self.images, FACE_NORMAL)
        self.last_drawn = FACE_NORMAL

        print("顔追跡開始。ESC で終了。")

        try:
            while self.running:
                self._check_events()

                # ── フレーム取得 ──
                with self.frame_lock:
                    frame = self.current_frame.copy() \
                        if self.current_frame is not None else None

                if frame is None:
                    time.sleep(0.05)
                    continue

                # ── 顔検出 ──
                face_x, face_y, face_w, face_h = self._detect_face(frame)
                now = time.time()

                if face_x is not None:
                    # ========== 顔あり ==========
                    self.last_face_time = now
                    target_face = FACE_HAPPY

                    # 表示切り替え
                    if target_face != self.last_drawn:
                        show_face(self.screen, self.images, target_face)
                        self.last_drawn = target_face

                    print(f"顔検出: x={face_x}")

                    # ① 左右に振る
                    self._wiggle()
                    self._check_events()

                    # ② 前進
                    self._approach()
                    self._check_events()

                else:
                    # ========== 顔なし ==========
                    elapsed     = now - self.last_face_time
                    target_face = FACE_NORMAL if elapsed > FACE_LOST_TIMEOUT \
                                  else FACE_HAPPY

                    self.motor.setMotorModel(0, 0)
                    print("顔なし: 待機中...")

                    # 表示切り替え
                    if target_face != self.last_drawn:
                        show_face(self.screen, self.images, target_face)
                        self.last_drawn = target_face

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n終了します...")
        finally:
            self._cleanup()

    # --------------------------------------------------
    # 終了処理
    # --------------------------------------------------
    def _cleanup(self):
        self.running = False
        self.motor.setMotorModel(0, 0)
        self.motor.close()
        self.ultrasonic.close()
        self.camera.close()
        pygame.quit()
        sys.exit()

# ===================================================
# エントリーポイント
# ===================================================
if __name__ == '__main__':
    png_paths      = convert_pdfs_to_png(PDF_PATHS, PNG_DIR)
    screen, images = init_display(png_paths)
    robot          = FaceTrackingRobot(screen, images)
    robot.start()