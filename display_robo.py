import pygame
import time
import sys

# ===== 設定 =====
IMAGE_PATHS = [
    "/home/pi/faces/face1.png",  # 通常顔
    "/home/pi/faces/face2.png",  # 笑顔
    "/home/pi/faces/face3.png",  # 怒り顔
    "/home/pi/faces/face4.png",  # 眠り顔
]
INTERVAL = 3  # 切り替え間隔（秒）

# ===== 初期化 =====
pygame.init()
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.mouse.set_visible(False)  # マウスカーソルを非表示
screen_w, screen_h = screen.get_size()

def load_images(paths):
    """画像を読み込んでフルスクリーンサイズにリサイズ"""
    images = []
    for path in paths:
        img = pygame.image.load(path).convert()
        img = pygame.transform.scale(img, (screen_w, screen_h))  # 画面全体に引き伸ばし
        images.append(img)
    return images

def show_face(index):
    """指定インデックスの顔を表示"""
    screen.blit(images[index], (0, 0))
    pygame.display.flip()

# 画像を事前ロード
images = load_images(IMAGE_PATHS)
current_index = 0
last_switch = time.time()

# ===== メインループ =====
try:
    while True:
        # イベント処理（Ctrl+CやESCで終了）
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    raise KeyboardInterrupt

        # 3秒経過で次の画像へ
        now = time.time()
        if now - last_switch >= INTERVAL:
            show_face(current_index)
            current_index = (current_index + 1) % len(images)
            last_switch = now

        time.sleep(0.05)  # CPU負荷を下げる

except KeyboardInterrupt:
    pygame.quit()
    sys.exit()