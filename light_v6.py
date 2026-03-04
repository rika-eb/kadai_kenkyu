import time
import board
import adafruit_tsl2591
from motor import tankMotor
from ultrasonic import Ultrasonic

# ===== センサ =====
i2c = board.I2C()
light = adafruit_tsl2591.TSL2591(i2c)
motor = tankMotor()
ultra = Ultrasonic()

# ===== DLI設定 =====
TARGET_DLI = 10
ACCUMULATION_HOURS = 10
LUX_TO_PPFD = 0.018

DLI_TARGET_UMOL = TARGET_DLI * 1_000_000
TOTAL_SECONDS = ACCUMULATION_HOURS * 3600

# ===== 制御パラメータ =====
BRIGHTNESS_TOLERANCE = 0.1
PPFD_DROP_RATIO = 0.8
DARK_RATIO = 0.9

TURN_POWER = 3000
TURN_STEP_TIME = 0.28
FORWARD_POWER = 3000
FORWARD_TIME = 0.4
MIN_MOVE_DISTANCE = 20

# ===== 状態 =====
accumulated_umol = 0
start_time = time.time()
last_integral_time = time.time()
best_global_avg = 0


# ===== 基本動作 =====
def stop():
    motor.setMotorModel(0, 0)


def rotate_left():
    motor.setMotorModel(-TURN_POWER, TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()


def rotate_right():
    motor.setMotorModel(TURN_POWER, -TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()


def rotate_180():
    for _ in range(6):
        rotate_right()
    stop()
    time.sleep(0.3)


def move_forward():
    if ultra.get_distance() > MIN_MOVE_DISTANCE:
        motor.setMotorModel(FORWARD_POWER, FORWARD_POWER)
        time.sleep(FORWARD_TIME)
        stop()
        time.sleep(0.3)


def get_lux():
    return light.lux or 0


def get_ppfd():
    return get_lux() * LUX_TO_PPFD


# ===== 常時積算 =====
def integrate_light():
    global accumulated_umol, last_integral_time

    now = time.time()
    dt = now - last_integral_time
    last_integral_time = now

    accumulated_umol += get_ppfd() * dt


# ===== 探索 =====
def explore(target_ppfd):

    global best_global_avg

    ANGLE_STEPS = 3
    reached_ppfd = 0
    best_global_avg = 0
    
    while True:

        #best_global_avg = 0

        lux_list = []
        best_ppfd = 0
        best_step = 0
        #best_offset = 0

        for _ in range(ANGLE_STEPS):
            rotate_left()
        
        for step in range(ANGLE_STEPS*2 +1):

          integrate_light()
          lux = get_lux()
          ppfd = lux * LUX_TO_PPFD
          distance = ultra.get_distance()
          
          lux_list.append(lux)

           # 目標に近い方向記
          if distance > MIN_MOVE_DISTANCE:
              if abs(ppfd - target_ppfd) < abs(best_ppfd - target_ppfd):
                best_ppfd = ppfd
                best_step = step
          rotate_right()

        # ③ 右端まで行っているので中央へ戻す
        for _ in range(ANGLE_STEPS):
            rotate_left()

          # 平均計算
        avg_lux = sum(lux_list) / len(lux_list)

        if avg_lux > best_global_avg:
            best_global_avg = avg_lux

        # 暗化判定 → Uターン
        if best_global_avg > 0 and avg_lux < best_global_avg * DARK_RATIO:
            print("Darker → U-turn")
            rotate_180()
            move_forward()
            return get_ppfd()
        offset_steps = best_step - ANGLE_STEPS
        # ===== ベスト方向へ回転 =====
        if offset_steps > 0:
            for _ in range(offset_steps):
                rotate_right()
        elif offset_steps < 0:
            for _ in range(-offset_steps):
                rotate_left()
        # ===== 前進 =====
        move_forward()

        integrate_light()

        reached_ppfd = get_ppfd()

        print(f"Reached PPFD: {reached_ppfd:.2f}")

        # ===== 目標到達判定 =====
        if reached_ppfd >= target_ppfd * (1 - BRIGHTNESS_TOLERANCE):
            print("Target brightness reached")
            break

        # これ以上明るくならない場合も終了
        if best_ppfd <= reached_ppfd:
            print("Max brightness reached in room")
            break
    return reached_ppfd

        


# ===== 滞在 =====
def stay(target_ppfd):

    stay_start_ppfd = get_ppfd()

    while True:

        integrate_light()

        elapsed = time.time() - start_time
        if elapsed >= TOTAL_SECONDS:
            return "finished"

        current_ppfd = get_ppfd()

        # 光減少判定（到達時基準）
        if current_ppfd < stay_start_ppfd * PPFD_DROP_RATIO:
            print("Light decreased → re-explore")
            return "reexplore"

        # 明るすぎ回避
        if current_ppfd > target_ppfd * (1 + BRIGHTNESS_TOLERANCE):
            rotate_left()
            move_forward()
            return "adjust"

        time.sleep(1)


# ===== メイン =====
try:

    while True:

        integrate_light()

        elapsed = time.time() - start_time
        remaining_time = TOTAL_SECONDS - elapsed

        if remaining_time <= 0:
            print("10h complete")
            break

        remaining_umol = DLI_TARGET_UMOL - accumulated_umol
        required_ppfd = remaining_umol / max(1, remaining_time)

        print(f"Required PPFD: {required_ppfd:.2f}")

        reached_ppfd = explore(required_ppfd)

        integrate_light()

        # ===== 不足補正（不足分を足す方式） =====
        if reached_ppfd < required_ppfd:
            shortage = required_ppfd - reached_ppfd
            required_ppfd = required_ppfd + shortage
            print(f"Increase next target to {required_ppfd:.2f}")

        result = stay(required_ppfd)

        if result == "finished":
            break

except KeyboardInterrupt:
    stop()
    print("Stopped manually")