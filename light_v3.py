import time
import board
import adafruit_tsl2591
from motor import tankMotor
from ultrasonic import Ultrasonic

i2c = board.I2C()
light = adafruit_tsl2591.TSL2591(i2c)

motor = tankMotor()
ultra = Ultrasonic()

TURN_STEP_TIME = 0.28      # 15°（キャリブ値）
FORWARD_POWER = 3000
TURN_POWER = 3000
STEP_ANGLE = 30
SCAN_STEPS = 3              # 90° ÷ 15°
MAX_FORWARD_TIME = 0.5
SAFE_DISTANCE = 25
STOP_DISTANCE = 15
STOP_MARGIN = 10
MAX_STEP_MOVE = 5

DARK_RATIO = 0.85

best_global_avg = 0


def stop():
    motor.setMotorModel(0,0)

def rotate_left_step():
    motor.setMotorModel(-TURN_POWER, TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()
    time.sleep(0.05)

def rotate_right_step():
    motor.setMotorModel(TURN_POWER, -TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()
    time.sleep(0.05)

def rotate_180():
    print("Turning 180")
    for _ in range(SCAN_STEPS * 2):
        rotate_right_step()

def get_lux():
    values = []
    for _ in range(3):
        values.append(light.lux or 0)
        time.sleep(0.05)
    return sum(values)/len(values)

def get_lux_fast():
    return light.lux or 0
try:
    #prev_lux = get_lux()

    while True:

        lux_list = []

        # ① 左へ90°移動
        for _ in range(SCAN_STEPS):
            rotate_left_step()
        max_lux = 0
        best_step =None
        best_distance = 0

        print("\n=== ±90° Scan Start ===")
        #time.sleep(0.2)

        # ② 左90°から右へ180°スキャン
        for i in range(SCAN_STEPS * 2 + 1):  # 180°分
            lux = get_lux()
            distance = ultra.get_distance()
            print(f"Step {i - SCAN_STEPS:2d}| Lux:{lux:.2f} | Dist:{distance:.2f}")
            lux_list.append(lux)

            if distance > SAFE_DISTANCE: #安全な方向のみ評価
                if lux > max_lux: #最も明るい方向を更新
                    max_lux = lux
                    best_step = i
                    best_distance = distance

            rotate_right_step()

        # ③ 右端まで行っているので中央へ戻す
        for _ in range(SCAN_STEPS):
            rotate_left_step()

        avg_lux = sum(lux_list)/len(lux_list)
        print(f"AVERAGE Lux:{avg_lux:.2f}")

        if avg_lux > best_global_avg:
            best_global_avg = avg_lux

        if best_global_avg > 0 and avg_lux < best_global_avg * DARK_RATIO:
            print("Enviornment significantly darker => GO back")
            #180回転して戻る
            rotate_180()

            current_distance = ultra.get_distance()
            move_distance = min(current_distance - STOP_MARGIN, MAX_STEP_MOVE)

        # if avg_lux < best_global_avg -5:
        #     print("Too dark compared to previous best area -> stop")
        #     stop()
        #     break
            if move_distance > 0:
                final_stop = current_distance - move_distance
                while ultra.get_distance() > final_stop + 1:
                    motor.setMotorModel(FORWARD_POWER, FORWARD_POWER)
                    time.sleep(0.02)
                
                stop()
            continue



        if best_step is None:
            print("No safe bright direction found. Skipping Move")
            time.sleep(0.5)
            continue
        
        # ④ ベスト方向へ回転\
        offset_steps = best_step - SCAN_STEPS #中央からのずれ計算

        print(f"Best direction offset: {offset_steps * STEP_ANGLE}°")

        if offset_steps > 0:
            for _ in range(offset_steps):
                rotate_right_step()
        elif offset_steps < 0:
            for _ in range(-offset_steps):
                rotate_left_step()

        current_distance = ultra.get_distance()
        safe_stop_distance = STOP_MARGIN

        max_possible_move = current_distance - safe_stop_distance

        if max_possible_move <= 0:
            print("Too close to obstacle, no forward movement")
            continue

        allowed_move  = min(max_possible_move, MAX_STEP_MOVE)

        final_stop_distance = current_distance - allowed_move
        print("Moving up to {allowed_move:.2f}cm")

        while ultra.get_distance() > final_stop_distance + 1:
            motor.setMotorModel(FORWARD_POWER, FORWARD_POWER)
            time.sleep(0.02)
        
        #print(f"Moving toward brightest point")
        #target_distance = best_distance - STOP_MARGIN
        #print(f"Target stop distance : {target_distance:.1f}")

        #while ultra.get_distance() > target_distance:
        #    motor.setMotorModel(FORWARD_POWER, FORWARD_POWER)
        #    time.sleep(0.2)
        
        stop()
        time.sleep(0.3)


except KeyboardInterrupt:
    stop()
    print("\nStopped manually")