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
TURN_STEP_TIME = 0.28  # 45度回転に必要な時間
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
    time.sleep(0.2)


def rotate_left():
    motor.setMotorModel(-TURN_POWER, TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()
    time.sleep(0.2)



def rotate_right():
    motor.setMotorModel(TURN_POWER, -TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()
    time.sleep(0.2)



def rotate_180():
    for _ in range(4):  # 45度×4 = 180度
        rotate_right()
    stop()
    time.sleep(0.2)


def move_forward():
    if ultra.get_distance() > MIN_MOVE_DISTANCE:
        motor.setMotorModel(FORWARD_POWER, FORWARD_POWER)
        time.sleep(FORWARD_TIME)
        stop()
        time.sleep(0.3)
        return True
    return False


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

    # -90, -45, 0, 45, 90度の5方向をスキャン
    ANGLE_STEPS = [-2, -1, 0, 1, 2]  # 45度単位での回転数
    
    previous_max_ppfd = 0  # 前回の移動での最大PPFD
    no_improvement_count = 0  # 改善がない回数
    MAX_NO_IMPROVEMENT = 3  # これ以上改善がなければ終了
    
    # 最も明るかった地点を記録
    brightest_ppfd = 0
    move_history = []  # 最も明るい地点からの移動履歴 [{'type': 'rotate_left'/'rotate_right'/'forward', 'count': n}, ...]
    
    while True:
        integrate_light()
        
        measurements = []
        
        # ① 左端（-90度）へ移動
        for _ in range(2):
            rotate_left()
        
        # ② -90度から90度まで測定（5方向）
        for i, angle_step in enumerate(ANGLE_STEPS):
            integrate_light()
            lux = get_lux()
            ppfd = lux * LUX_TO_PPFD
            distance = ultra.get_distance()
            
            measurements.append({
                'index': i,
                'angle_step': angle_step,
                'lux': lux,
                'ppfd': ppfd,
                'distance': distance,
                'can_move': distance > MIN_MOVE_DISTANCE
            })
            
            print(f"Angle {angle_step*45}°: Lux={lux:.2f}, PPFD={ppfd:.2f}, Dist={distance:.1f}cm")
            
            # 最後(i=4)以外は右回転して次の角度へ
            if i < len(ANGLE_STEPS) - 1:
                rotate_right()
        
        # ③ 中央（0度）へ戻す
        for _ in range(2):
            rotate_left()
        
        # ④ 平均明るさ計算と暗化判定
        avg_lux = sum(m['lux'] for m in measurements) / len(measurements)
        
        if best_global_avg == 0:
            best_global_avg = avg_lux
        
        if avg_lux < best_global_avg * DARK_RATIO:
            print(f"Darker detected (current:{avg_lux:.2f} < best:{best_global_avg:.2f}*{DARK_RATIO}) → U-turn")
            rotate_180()
            if move_forward():
                best_global_avg = 0  # リセット
                brightest_ppfd = 0
                move_history = []
            return get_ppfd()
        
        if avg_lux > best_global_avg:
            best_global_avg = avg_lux
        
        # ⑤ 最適方向を選択
        movable = [m for m in measurements if m['can_move']]
        
        if not movable:
            print("No movable direction → stay here")
            return get_ppfd()
        
        # 目標PPFDに最も近い方向を選択
        best = min(movable, key=lambda m: abs(m['ppfd'] - target_ppfd))
        
        # 現在の最大PPFDを記録
        current_max_ppfd = max(m['ppfd'] for m in measurements)
        current_ppfd = get_ppfd()
        
        print(f"Best direction: {best['angle_step']*45}° (PPFD={best['ppfd']:.2f})")
        
        # ⑥ ベスト方向へ回転
        rotation_count = 0
        if best['angle_step'] > 0:
            for _ in range(best['angle_step']):
                rotate_right()
            rotation_count = best['angle_step']
            rotation_type = 'rotate_right'
        elif best['angle_step'] < 0:
            for _ in range(-best['angle_step']):
                rotate_left()
            rotation_count = -best['angle_step']
            rotation_type = 'rotate_left'
        else:
            rotation_type = None
        
        # ⑦ 前進
        moved = move_forward()
        
        if not moved:
            print("Cannot move forward → stay here")
            return get_ppfd()
        
        integrate_light()
        reached_ppfd = get_ppfd()
        
        print(f"Reached PPFD: {reached_ppfd:.2f}, Target: {target_ppfd:.2f}")
        
        # 最も明るい地点を更新
        if reached_ppfd > brightest_ppfd:
            brightest_ppfd = reached_ppfd
            move_history = []  # 新しい最良地点なので履歴をクリア
            print(f"New brightest point: {brightest_ppfd:.2f}")
        else:
            # 最良地点からの移動を記録
            if rotation_type:
                move_history.append({'type': rotation_type, 'count': rotation_count})
            move_history.append({'type': 'forward', 'count': 1})
        
        # ⑧ 終了判定
        # 目標達成
        if reached_ppfd >= target_ppfd * (1 - BRIGHTNESS_TOLERANCE):
            print("Target brightness reached")
            break
        
        # 改善判定
        if current_max_ppfd <= previous_max_ppfd * 1.05:  # 5%以上の改善がない
            no_improvement_count += 1
            print(f"No significant improvement ({no_improvement_count}/{MAX_NO_IMPROVEMENT})")
            
            if no_improvement_count >= MAX_NO_IMPROVEMENT:
                print("Max brightness reached in room (no improvement)")
                
                # 最も明るかった地点に戻る
                if len(move_history) > 0:
                    print(f"Returning to brightest point (history: {len(move_history)} moves)")
                    
                    # 180度回転
                    rotate_180()
                    integrate_light()
                    
                    # 移動履歴を逆順に実行
                    for move in reversed(move_history):
                        if move['type'] == 'forward':
                            # 前進を逆にする = 前進（180度回転済みなので前進で戻る）
                            for _ in range(move['count']):
                                move_forward()
                                integrate_light()
                        elif move['type'] == 'rotate_right':
                            # 右回転を逆にする = 左回転
                            for _ in range(move['count']):
                                rotate_left()
                        elif move['type'] == 'rotate_left':
                            # 左回転を逆にする = 右回転
                            for _ in range(move['count']):
                                rotate_right()
                    
                    # 再び180度回転して元の向きに戻す
                    rotate_180()
                    integrate_light()
                    
                    final_ppfd = get_ppfd()
                    print(f"Returned to brightest point: PPFD={final_ppfd:.2f}")
                    return final_ppfd
                else:
                    print("Already at brightest point")
                    break
        else:
            no_improvement_count = 0  # 改善があればリセット
        
        previous_max_ppfd = current_max_ppfd
    
    return get_ppfd()


# ===== 滞在 =====
def stay(target_ppfd):
    stay_start_ppfd = get_ppfd()
    
    print(f"Staying at PPFD: {stay_start_ppfd:.2f}, Target: {target_ppfd:.2f}")

    while True:
        integrate_light()

        elapsed = time.time() - start_time
        if elapsed >= TOTAL_SECONDS:
            return "finished"

        current_ppfd = get_ppfd()

        # 光減少判定（滞在開始時基準）
        if current_ppfd < stay_start_ppfd * PPFD_DROP_RATIO:
            print(f"Light decreased ({current_ppfd:.2f} < {stay_start_ppfd * PPFD_DROP_RATIO:.2f}) → re-explore")
            return "reexplore"

        # 明るすぎ回避
        if current_ppfd > target_ppfd * (1 + BRIGHTNESS_TOLERANCE):
            print(f"Too bright ({current_ppfd:.2f} > {target_ppfd * (1 + BRIGHTNESS_TOLERANCE):.2f}) → adjust")
            rotate_left()
            if move_forward():
                return "adjust"

        time.sleep(1)


# ===== メイン =====
try:
    print("=== Light accumulation control started ===")
    print(f"Target DLI: {TARGET_DLI} mol/m²/d")
    print(f"Target time: {ACCUMULATION_HOURS}h")
    
    adjustment_ppfd = 0  # 不足分の補正値

    while True:
        integrate_light()

        elapsed = time.time() - start_time
        remaining_time = TOTAL_SECONDS - elapsed

        if remaining_time <= 0:
            print("=== 10h complete ===")
            print(f"Accumulated: {accumulated_umol / 1_000_000:.2f} mol/m²")
            break

        remaining_umol = DLI_TARGET_UMOL - accumulated_umol
        required_ppfd = remaining_umol / max(1, remaining_time)
        
        # 不足分の補正を加える
        required_ppfd += adjustment_ppfd

        print(f"\n--- New cycle ---")
        print(f"Elapsed: {elapsed/3600:.2f}h, Remaining: {remaining_time/3600:.2f}h")
        print(f"Accumulated: {accumulated_umol / 1_000_000:.2f} mol/m²")
        print(f"Required PPFD: {required_ppfd:.2f} µmol/m²/s")

        reached_ppfd = explore(required_ppfd)

        integrate_light()

        # ===== 不足補正 =====
        if reached_ppfd < required_ppfd * (1 - BRIGHTNESS_TOLERANCE):
            shortage = required_ppfd - reached_ppfd
            adjustment_ppfd += shortage
            print(f"Shortage detected: {shortage:.2f} µmol/m²/s")
            print(f"Adjustment for next cycle: +{adjustment_ppfd:.2f} µmol/m²/s")
        else:
            # 目標達成できた場合は補正をリセット
            adjustment_ppfd = 0

        result = stay(required_ppfd)

        if result == "finished":
            print("=== Target reached ===")
            break

except KeyboardInterrupt:
    stop()
    print("\n=== Stopped manually ===")
    print(f"Accumulated: {accumulated_umol / 1_000_000:.2f} mol/m²")

finally:
    stop()