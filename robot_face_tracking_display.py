# robot_face_tracking.py
import cv2
import numpy as np
import pygame
import time
import sys
import os
import threading
from pdf2image import convert_from_path
from motor import tankMotor
from ultrasonic import Ultrasonic
from camera import Camera

# ===================================================
# 設定
# ===================================================
PDF_PATHS = [
    os.path.expanduser("~/kadai_kenkyu/happy.pdf"),       # 0: 通常時
    os.path.expanduser("~/kadai_kenkyu/chou_happy.pdf"),  # 1: 顔認識時
]
PNG_DIR  = os.path.expanduser("~/kadai_kenkyu/png_cache/")

# 顔インデックス定数
FACE_NORMAL = 0
FACE_HAPPY  = 1

# ロボット制御パラメータ
TARGET_DISTANCE    = 30   # 目標距離 (cm)
TOLERANCE_DISTANCE = 5    # 距離の許容誤差 (cm)
TOLERANCE_ANGLE    = 50   # 横方向の許容ずれ (px)
APPROACH_SPEED     = 800
TURN_SPEED         = 1000
FACE_LOST_TIMEOUT  = 2.0  # 顔消失後に表情を戻すまでの秒数

# ===================================================
# PDF → PNG 変換
# ===================================================
def convert_pdfs_to_png(pdf_paths, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    png_paths = []
    for pdf_path in pdf_paths:
        # PDFのファイル名（拡張子なし）をそのままPNG名に使う
        # 例: happy.pdf → png_cache/happy.png
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
        img = pygame.transform.rotate(img, 90)   # 必要に応じて 90 / -90 / 180
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

        # ハードウェア初期化
        self.motor      = tankMotor()
        self.ultrasonic = Ultrasonic()
        self.camera     = Camera()

        # 顔検出モデル
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # カメラ解像度・中心
        self.cam_w    = 640
        self.cam_h    = 480
        self.center_x = self.cam_w // 2

        # 状態
        self.running        = True
        self.current_frame  = None
        self.frame_lock     = threading.Lock()
        self.last_face_time = 0.0
        self.last_drawn     = -1   # 前回描画したインデックス（差分描画用）

    # --------------------------------------------------
    # カメラスレッド（常時フレーム取得）
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
    # 顔検出（最大の顔を返す）
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
    # モーター制御
    # --------------------------------------------------
    def _move(self, action):
        if   action == "forward":    self.motor.setMotorModel( APPROACH_SPEED,  APPROACH_SPEED)
        elif action == "backward":   self.motor.setMotorModel(-APPROACH_SPEED, -APPROACH_SPEED)
        elif action == "turn_left":  self.motor.setMotorModel(-TURN_SPEED,      TURN_SPEED)
        elif action == "turn_right": self.motor.setMotorModel( TURN_SPEED,     -TURN_SPEED)
        elif action == "search":     self.motor.setMotorModel(-TURN_SPEED // 2, TURN_SPEED // 2)
        else:                        self.motor.setMotorModel(0, 0)   # stop

    # --------------------------------------------------
    # メインループ
    # --------------------------------------------------
    def start(self):
        # カメラスレッド開始
        t = threading.Thread(target=self._capture_loop, daemon=True)
        t.start()
        time.sleep(2)  # カメラ起動待ち

        # 起動時は通常顔を表示
        show_face(self.screen, self.images, FACE_NORMAL)
        self.last_drawn = FACE_NORMAL

        print("顔追跡開始。ESC で終了。")

        try:
            while self.running:
                # ── pygameイベント処理 ──
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        raise KeyboardInterrupt

                # ── フレーム取得 ──
                with self.frame_lock:
                    frame = self.current_frame.copy() if self.current_frame is not None else None

                if frame is None:
                    time.sleep(0.05)
                    continue

                # ── 顔検出 ──
                face_x, face_y, face_w, face_h = self._detect_face(frame)
                now = time.time()

                if face_x is not None:
                    # ========== 顔あり ==========
                    self.last_face_time = now
                    target_face = FACE_HAPPY  # chou_happy を表示

                    # 横方向のずれでターン判定
                    deviation = face_x - self.center_x
                    if abs(deviation) > TOLERANCE_ANGLE:
                        action = "turn_right" if deviation > 0 else "turn_left"
                    else:
                        dist = self.ultrasonic.get_distance()
                        if dist == 0:
                            action = "stop"
                        elif dist > TARGET_DISTANCE + TOLERANCE_DISTANCE:
                            action = "forward"
                        elif dist < TARGET_DISTANCE - TOLERANCE_DISTANCE:
                            action = "backward"
                        else:
                            action = "stop"

                    print(f"顔検出: x={face_x}  動作={action}")

                else:
                    # ========== 顔なし ==========
                    elapsed = now - self.last_face_time
                    target_face = FACE_NORMAL if elapsed > FACE_LOST_TIMEOUT else FACE_HAPPY
                    action = "search"
                    print("顔なし: 探索中...")

                # ── モーター実行 ──
                self._move(action)

                # ── 表示更新（変化があるときだけ） ──
                if target_face != self.last_drawn:
                    show_face(self.screen, self.images, target_face)
                    self.last_drawn = target_face

                time.sleep(0.1)

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