# robot_face.py
import pygame
import time
import sys
import os
from pdf2image import convert_from_path

# ===== 設定 =====
PDF_PATHS = [
    os.path.expanduser("~/kadai_kenkyu/robo_display_img.pdf"),  # 通常顔
    os.path.expanduser("~/kadai_kenkyu/robo_display_img.pdf"),  # 笑顔
    os.path.expanduser("~/kadai_kenkyu/robo_display_img.pdf"),  # 怒り顔
    os.path.expanduser("~/kadai_kenkyu/robo_display_img.pdf"),  # 眠り顔
]
PNG_DIR = os.path.expanduser("~/kadai_kenkyu/png_cache/")  # 変換後PNGの保存先
INTERVAL = 3  # 切り替え間隔（秒）

# ===== PDF→PNG変換 =====
def convert_pdfs_to_png(pdf_paths, out_dir):
    """未変換のPDFだけPNGに変換して保存、パスのリストを返す"""
    os.makedirs(out_dir, exist_ok=True)  # 保存フォルダがなければ作成
    png_paths = []

    for i, pdf_path in enumerate(pdf_paths):
        png_path = os.path.join(out_dir, f"face_{i+1}.png")

        # すでに変換済みならスキップ
        if os.path.exists(png_path):
            print(f"スキップ（変換済み）: {png_path}")
        else:
            print(f"変換中: {pdf_path} → {png_path}")
            pages = convert_from_path(pdf_path, dpi=150)
            pages[0].save(png_path, "PNG")
            print(f"変換完了: {png_path}")

        png_paths.append(png_path)

    return png_paths

# ===== pygame初期化 =====
pygame.init()
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.mouse.set_visible(False)
screen_w, screen_h = screen.get_size()

# ===== 画像読み込み =====
def load_images(png_paths):
    images = []
    for path in png_paths:
        img = pygame.image.load(path).convert()
        img = pygame.transform.scale(img, (screen_w, screen_h))
        images.append(img)
    return images

def show_face(index):
    screen.blit(images[index], (0, 0))
    pygame.display.flip()

# ===== 起動時にPDF変換→画像読み込み =====
png_paths = convert_pdfs_to_png(PDF_PATHS, PNG_DIR)
images = load_images(png_paths)

current_index = 0
last_switch = time.time()

# ===== メインループ =====
try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    raise KeyboardInterrupt

        now = time.time()
        if now - last_switch >= INTERVAL:
            show_face(current_index)
            current_index = (current_index + 1) % len(images)
            last_switch = now

        time.sleep(0.05)

except KeyboardInterrupt:
    pygame.quit()
    sys.exit()