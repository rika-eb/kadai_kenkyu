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
LUX_TO_PPFD = 0.0281

DLI_TARGET_UMOL = TARGET_DLI * 1_000_000
TOTAL_SECONDS = ACCUMULATION_HOURS * 3600

# ===== 制御パラメータ =====
BRIGHTNESS_TOLERANCE = 0.1
PPFD_DROP_RATIO = 0.8
DARK_RATIO = 0.9
ANGLE_CORRECTION_THRESHOLD = 0.05  # 15%以上のPPFD乖離で角度補正

TURN_POWER = 3000
TURN_STEP_TIME = 0.28
FORWARD_POWER = 3000
FORWARD_TIME = 0.4
MIN_MOVE_DISTANCE = 20

# ===== 30秒ごとの表示用 =====
PRINT_INTERVAL = 30

# ===== 状態 =====
accumulated_umol = 0
start_time = time.time()
last_integral_time = time.time()
last_print_time = time.time()
best_global_avg = 0


# ===== 基本動作 =====
def pause(msg="確認してください"):
    input(f"[PAUSE] {msg} → Enterで続行")


def stop():
    motor.setMotorModel(0, 0)
    time.sleep(0.3)


def rotate_left():
    motor.setMotorModel(-TURN_POWER, TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()
    time.sleep(0.3)


def rotate_right():
    motor.setMotorModel(TURN_POWER, -TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()
    time.sleep(0.3)


def rotate_180():
    for _ in range(4):
        rotate_right()
    stop()
    time.sleep(0.3)


def move_forward():
    if ultra.get_distance() > MIN_MOVE_DISTANCE:
        motor.setMotorModel(FORWARD_POWER, FORWARD_POWER)
        time.sleep(FORWARD_TIME)
        stop()
        time.sleep(1.0)
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

    ppfd = get_ppfd()
    increment = ppfd * dt
    accumulated_umol += increment


# ===== 30秒ごとの表示 =====
def print_status_if_needed():
    global last_print_time, accumulated_umol
    
    now = time.time()
    if now - last_print_time >= PRINT_INTERVAL:
        current_lux = get_lux()
        current_ppfd = get_ppfd()
        print(f"[Status] Lux={current_lux:.2f}, PPFD={current_ppfd:.2f} µmol/m²/s, Accumulated={accumulated_umol:.2f} µmol/m²")
        last_print_time = now


# ===== 角度補正 =====
def estimate_angle_from_ppfd(current_ppfd, measurements):
    """
    現在のPPFD値から、5点測定値を元に実際の角度を推定
    
    Args:
        current_ppfd: 現在測定されたPPFD値
        measurements: 5方向の測定結果リスト
    
    Returns:
        推定角度ステップ（-2～2の範囲）、None if 推定不可
    """
    # 測定値を角度ステップとPPFDのペアに
    angle_ppfd_pairs = [(m['angle_step'], m['ppfd']) for m in measurements]
    angle_ppfd_pairs.sort(key=lambda x: x[0])  # 角度順にソート
    
    # 現在値が測定範囲内かチェック
    min_ppfd = min(p[1] for p in angle_ppfd_pairs)
    max_ppfd = max(p[1] for p in angle_ppfd_pairs)
    
    if current_ppfd < min_ppfd * 0.8 or current_ppfd > max_ppfd * 1.2:
        print(f"  Current PPFD {current_ppfd:.2f} is out of measurement range [{min_ppfd:.2f}, {max_ppfd:.2f}]")
        return None
    
    # 線形補間で角度を推定
    for i in range(len(angle_ppfd_pairs) - 1):
        angle1, ppfd1 = angle_ppfd_pairs[i]
        angle2, ppfd2 = angle_ppfd_pairs[i + 1]
        
        # current_ppfdがこの区間内にあるか
        if min(ppfd1, ppfd2) <= current_ppfd <= max(ppfd1, ppfd2):
            # 線形補間
            if abs(ppfd2 - ppfd1) < 0.1:  # ほぼ同じ明るさ
                estimated_angle = (angle1 + angle2) / 2.0
            else:
                ratio = (current_ppfd - ppfd1) / (ppfd2 - ppfd1)
                estimated_angle = angle1 + ratio * (angle2 - angle1)
            
            print(f"  Estimated angle: {estimated_angle * 45:.1f}° (step {estimated_angle:.2f})")
            return estimated_angle
    
    # 最も近い値を選択
    closest = min(angle_ppfd_pairs, key=lambda x: abs(x[1] - current_ppfd))
    print(f"  Using closest match: angle step {closest[0]} (PPFD {closest[1]:.2f})")
    return closest[0]


def correct_angle_if_needed(expected_ppfd, expected_angle_step, measurements):
    """
    前進前に角度補正が必要かチェックし、必要なら補正する
    
    Args:
        expected_ppfd: 期待されるPPFD値
        expected_angle_step: 期待される角度ステップ
        measurements: 5方向の測定結果
    
    Returns:
        (補正後の角度ステップ, 実際に補正したか)
    """
    current_ppfd = get_ppfd()
    current_distance = ultra.get_distance()
    
    print(f"\n--- Angle correction check ---")
    print(f"  Expected: PPFD={expected_ppfd:.2f}, Angle={expected_angle_step*45}°")
    print(f"  Current:  PPFD={current_ppfd:.2f}, Distance={current_distance:.1f}cm")
    
    # 乖離度チェック
    deviation = abs(current_ppfd - expected_ppfd) / max(expected_ppfd, 0.1)
    print(f"  Deviation: {deviation*100:.1f}%")
    
    if deviation < ANGLE_CORRECTION_THRESHOLD:
        print(f"  → No correction needed (deviation < {ANGLE_CORRECTION_THRESHOLD*100:.0f}%)")
        return expected_angle_step, False
    
    # 角度推定
    print(f"  → Correction needed! Estimating actual angle...")
    estimated_angle = estimate_angle_from_ppfd(current_ppfd, measurements)
    
    if estimated_angle is None:
        print(f"  → Cannot estimate angle, using expected angle")
        return expected_angle_step, False
    
    # 補正角度を計算（整数ステップに丸める）
    correction_step = round(estimated_angle - expected_angle_step)
    
    if correction_step == 0:
        print(f"  → Estimated angle matches expected, no correction needed")
        return expected_angle_step, False
    
    print(f"  → Correction: {correction_step} steps ({correction_step*45}°)")
    
    # 補正回転を実行
    if correction_step > 0:
        for _ in range(abs(correction_step)):
            rotate_right()
        print(f"  → Rotated right {abs(correction_step)*45}°")
    else:
        for _ in range(abs(correction_step)):
            rotate_left()
        print(f"  → Rotated left {abs(correction_step)*45}°")
    
    # 補正後の確認
    corrected_ppfd = get_ppfd()
    corrected_distance = ultra.get_distance()
    print(f"  After correction: PPFD={corrected_ppfd:.2f}, Distance={corrected_distance:.1f}cm")
    
    return expected_angle_step + correction_step, True


# ===== 再スキャン（障害物回避用） =====
def rescan_for_brightest_movable():
    print("\n=== Re-scanning for brightest movable direction ===")
    
    ANGLE_STEPS = [-2, -1, 0, 1, 2]
    measurements = []
    
    for _ in range(2):
        rotate_left()
    
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
        
        print(f"  Re-scan Angle {angle_step*45}°: Lux={lux:.2f}, PPFD={ppfd:.2f}, Dist={distance:.1f}cm, Can move={distance > MIN_MOVE_DISTANCE}")
        
        if i < len(ANGLE_STEPS) - 1:
            rotate_right()
    
    for _ in range(2):
        rotate_left()
    
    movable = [m for m in measurements if m['can_move']]
    
    if not movable:
        print("  No movable direction found after re-scan → stay here")
        return None, measurements
    
    brightest = max(movable, key=lambda m: m['ppfd'])
    
    print(f"  Brightest movable direction: {brightest['angle_step']*45}° (PPFD={brightest['ppfd']:.2f}, Dist={brightest['distance']:.1f}cm)")
    
    return brightest, measurements


# ===== 探索 =====
def explore(target_ppfd):
    global best_global_avg

    ANGLE_STEPS = [-2, -1, 0, 1, 2]
    
    previous_reached_ppfd = 0
    no_improvement_count = 0
    MAX_NO_IMPROVEMENT = 3
    
    brightest_ppfd = 0
    move_history = []
    
    while True:
        integrate_light()
        print_status_if_needed()
        
        measurements = []
        
        for _ in range(2):
            rotate_left()
        
        for i, angle_step in enumerate(ANGLE_STEPS):
            integrate_light()
            print_status_if_needed()
            
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
            
            if i < len(ANGLE_STEPS) - 1:
                rotate_right()
        
        for _ in range(2):
            rotate_left()
        
        avg_lux = sum(m['lux'] for m in measurements) / len(measurements)
        
        if best_global_avg == 0:
            best_global_avg = avg_lux
        
        if avg_lux < best_global_avg * DARK_RATIO:
            print(f"Darker detected (current:{avg_lux:.2f} < best:{best_global_avg:.2f}*{DARK_RATIO}) → U-turn and continue exploring")
            rotate_180()
            if move_forward():
                best_global_avg = 0
                brightest_ppfd = 0
                move_history = []
                previous_reached_ppfd = 0
                no_improvement_count = 0
            continue
        
        if avg_lux > best_global_avg:
            best_global_avg = avg_lux
        
        movable = [m for m in measurements if m['can_move']]
        
        if not movable:
            print("No movable direction in initial scan → stay here")
            return get_ppfd()
        
        best = min(movable, key=lambda m: abs(m['ppfd'] - target_ppfd))
        
        print(f"Best direction: {best['angle_step']*45}° (PPFD={best['ppfd']:.2f})")
        
        rotation_angle = best['angle_step']
        
        if rotation_angle > 0:
            for _ in range(rotation_angle):
                rotate_right()
        elif rotation_angle < 0:
            for _ in range(-rotation_angle):
                rotate_left()
        
        # ★★★ 角度補正処理を追加 ★★★
                # ★★★ 角度補正処理を追加 ★★★
        corrected_angle, was_corrected = correct_angle_if_needed(
            best['ppfd'], 
            rotation_angle, 
            measurements
        )
        
        # 補正があった場合は履歴にも反映
        if was_corrected:
            rotation_angle = corrected_angle
        
        moved = move_forward()
        
        # ★★★ Cannot move forward時の処理 ★★★
        if not moved:
            current_distance = ultra.get_distance()
            print(f"Cannot move forward (distance={current_distance:.1f}cm < {MIN_MOVE_DISTANCE}cm)")
            
            # 元の向きに戻す
            if rotation_angle > 0:
                for _ in range(rotation_angle):
                    rotate_left()
            elif rotation_angle < 0:
                for _ in range(-rotation_angle):
                    rotate_right()
            
            # 再スキャンして最も明るい移動可能な方向を探す
            brightest_movable, rescan_measurements = rescan_for_brightest_movable()
            
            if brightest_movable is None:
                print("No alternative direction available → stay here")
                return get_ppfd()
            
            # 最も明るい方向へ回転
            rotation_angle = brightest_movable['angle_step']
            
            if rotation_angle > 0:
                for _ in range(rotation_angle):
                    rotate_right()
            elif rotation_angle < 0:
                for _ in range(-rotation_angle):
                    rotate_left()
            
            # 再スキャン後も角度補正を実行
            corrected_angle, was_corrected = correct_angle_if_needed(
                brightest_movable['ppfd'],
                rotation_angle,
                rescan_measurements
            )
            
            if was_corrected:
                rotation_angle = corrected_angle
            
            # 前進を試みる
            moved = move_forward()
            
            if not moved:
                print("Still cannot move forward after re-scan → stay here")
                # 元の向きに戻す
                if rotation_angle > 0:
                    for _ in range(rotation_angle):
                        rotate_left()
                elif rotation_angle < 0:
                    for _ in range(-rotation_angle):
                        rotate_right()
                return get_ppfd()
            
            print(f"Successfully moved to alternative direction")
        
        integrate_light()
        print_status_if_needed()
        
        reached_ppfd = get_ppfd()
        
        print(f"Reached PPFD: {reached_ppfd:.2f}, Target: {target_ppfd:.2f}")
        
        if reached_ppfd > brightest_ppfd:
            brightest_ppfd = reached_ppfd
            move_history = []
            print(f"New brightest point: {brightest_ppfd:.2f}")
        else:
            move_history.append({
                'rotation_angle': rotation_angle,
                'forward': True
            })
        
        if reached_ppfd >= target_ppfd * (1 - BRIGHTNESS_TOLERANCE):
            print("Target brightness reached")
            break
        
        if previous_reached_ppfd == 0:
            previous_reached_ppfd = reached_ppfd
        elif reached_ppfd <= previous_reached_ppfd * 1.05:
            no_improvement_count += 1
            print(f"No significant improvement in reached PPFD ({reached_ppfd:.2f} <= {previous_reached_ppfd:.2f}*1.05) - count: {no_improvement_count}/{MAX_NO_IMPROVEMENT}")
            
            if no_improvement_count >= MAX_NO_IMPROVEMENT:
                print(f"Max brightness reached in room (no improvement for {MAX_NO_IMPROVEMENT} moves)")
                
                if len(move_history) > 0:
                    print(f"\n=== Returning to brightest point (PPFD={brightest_ppfd:.2f}) ===")
                    print(f"Move history length: {len(move_history)} steps")
                    print("Return path:")
                    
                    print("  Step 1: Rotate 180°")
                    rotate_180()
                    integrate_light()
                    print_status_if_needed()
                    
                    step_num = 2
                    
                    for i, move in enumerate(reversed(move_history)):
                        print(f"  Step {step_num}: Move forward")
                        move_forward()
                        integrate_light()
                        print_status_if_needed()
                        step_num += 1
                        
                        rotation_angle = move['rotation_angle']
                        reversed_angle = -rotation_angle
                        
                        if reversed_angle != 0:
                            angle_degrees = reversed_angle * 45
                            if reversed_angle > 0:
                                print(f"  Step {step_num}: Rotate right {angle_degrees}° (reversing original {rotation_angle*45}° after 180° turn)")
                                for _ in range(reversed_angle):
                                    rotate_right()
                            else:
                                print(f"  Step {step_num}: Rotate left {-angle_degrees}° (reversing original {rotation_angle*45}° after 180° turn)")
                                for _ in range(-reversed_angle):
                                    rotate_left()
                            step_num += 1
                    
                    print(f"  Step {step_num}: Rotate 180° (restore original orientation)")
                    rotate_180()
                    integrate_light()
                    print_status_if_needed()
                    
                    final_ppfd = get_ppfd()
                    print(f"=== Returned to brightest point: PPFD={final_ppfd:.2f} ===\n")
                    return final_ppfd
                else:
                    print("Already at brightest point")
                    break
        else:
            no_improvement_count = 0
            previous_reached_ppfd = reached_ppfd
            print(f"Improvement detected: {reached_ppfd:.2f} > {previous_reached_ppfd * 1.05:.2f}")
    
    return get_ppfd()


# ===== 滞在 =====
def stay(target_ppfd):
    global accumulated_umol
    
    stay_start_ppfd = get_ppfd()
    
    print(f"=== Entering STAY phase ===")
    print(f"Staying at PPFD: {stay_start_ppfd:.2f} µmol/m²/s, Target: {target_ppfd:.2f} µmol/m²/s")
    print(f"Current accumulated: {accumulated_umol:.2f} µmol/m²")

    while True:
        integrate_light()
        print_status_if_needed()

        elapsed = time.time() - start_time
        if elapsed >= TOTAL_SECONDS:
            return "finished"

        current_ppfd = get_ppfd()
        if current_ppfd < stay_start_ppfd * PPFD_DROP_RATIO:
            print(f"Light decreased ({current_ppfd:.2f} < {stay_start_ppfd * PPFD_DROP_RATIO:.2f}) → re-explore")
            return "reexplore"

        if current_ppfd > target_ppfd * (1 + BRIGHTNESS_TOLERANCE):
            print(f"Too bright ({current_ppfd:.2f} > {target_ppfd * (1 + BRIGHTNESS_TOLERANCE):.2f}) → adjust")
            rotate_left()
            if move_forward():
                return "adjust"

        time.sleep(1)


# ===== メイン =====
try:
    print("=== Light accumulation control started ===")
    print(f"Target DLI: {TARGET_DLI} mol/m²/d ({DLI_TARGET_UMOL} µmol/m²)")
    print(f"Target time: {ACCUMULATION_HOURS}h ({TOTAL_SECONDS} seconds)")
    print(f"Lux to PPFD conversion: {LUX_TO_PPFD}")
    print(f"Angle correction threshold: {ANGLE_CORRECTION_THRESHOLD*100:.0f}%")
    
    adjustment_ppfd = 0

    while True:
        integrate_light()

        elapsed = time.time() - start_time
        remaining_time = TOTAL_SECONDS - elapsed

        if remaining_time <= 0:
            print("=== 10h complete ===")
            print(f"Accumulated: {accumulated_umol:.2f} µmol/m² ({accumulated_umol / 1_000_000:.3f} mol/m²)")
            break

        remaining_umol = DLI_TARGET_UMOL - accumulated_umol
        required_ppfd = remaining_umol / max(1, remaining_time)
        
        required_ppfd += adjustment_ppfd

        print(f"\n--- New cycle ---")
        print(f"Elapsed: {elapsed/3600:.2f}h, Remaining: {remaining_time/3600:.2f}h")
        print(f"Accumulated: {accumulated_umol:.2f} µmol/m² ({accumulated_umol / 1_000_000:.3f} mol/m²)")
        print(f"Remaining: {remaining_umol:.2f} µmol/m²")
        print(f"Required PPFD: {required_ppfd:.2f} µmol/m²/s")

        reached_ppfd = explore(required_ppfd)

        integrate_light()

        if reached_ppfd < required_ppfd * (1 - BRIGHTNESS_TOLERANCE):
            shortage = required_ppfd - reached_ppfd
            adjustment_ppfd += shortage
            print(f"Shortage detected: {shortage:.2f} µmol/m²/s")
            print(f"Adjustment for next cycle: +{adjustment_ppfd:.2f} µmol/m²/s")
        else:
            adjustment_ppfd = 0

        result = stay(required_ppfd)

        if result == "finished":
            print("=== Target reached ===")
            print(f"Final accumulated: {accumulated_umol:.2f} µmol/m² ({accumulated_umol / 1_000_000:.3f} mol/m²)")
            break

except KeyboardInterrupt:
    stop()
    print("\n=== Stopped manually ===")
    print(f"Accumulated: {accumulated_umol:.2f} µmol/m² ({accumulated_umol / 1_000_000:.3f} mol/m²)")

finally:
    stop()