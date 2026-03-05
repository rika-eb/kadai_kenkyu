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
DARK_RATIO = 0.7
ANGLE_CORRECTION_THRESHOLD = 0.15

TURN_POWER = 3000
TURN_STEP_TIME = 0.28
FORWARD_POWER = 3000
FORWARD_TIME = 0.4
MIN_MOVE_DISTANCE = 20
SAFE_DISTANCE_MARGIN = 30  # ★新規：安全距離（より遠くを確保）

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


def check_distance_safe(min_distance=MIN_MOVE_DISTANCE):
    """
    現在の距離が安全かチェック
    複数回測定して平均を取ることで精度向上
    """
    distances = []
    for _ in range(3):  # 3回測定
        distances.append(ultra.get_distance())
        time.sleep(0.1)
    
    avg_distance = sum(distances) / len(distances)
    is_safe = avg_distance > min_distance
    
    print(f"  Distance check: {avg_distance:.1f}cm (min: {min_distance}cm) → {'SAFE' if is_safe else 'BLOCKED'}")
    return is_safe, avg_distance


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


# ===== 角度補正（Luxベース + 距離監視強化） =====
def estimate_angle_from_lux(current_lux, measurements):
    """
    現在のLux値から、5点測定値を元に実際の角度を推定
    ★追加：距離も考慮して推定
    """
    # 測定値を角度ステップとLuxのペアに
    angle_lux_pairs = [(m['angle_step'], m['lux'], m['distance']) for m in measurements]
    angle_lux_pairs.sort(key=lambda x: x[0])
    
    # 現在値が測定範囲内かチェック
    min_lux = min(p[1] for p in angle_lux_pairs)
    max_lux = max(p[1] for p in angle_lux_pairs)
    
    if current_lux < min_lux * 0.8 or current_lux > max_lux * 1.2:
        print(f"  Current Lux {current_lux:.2f} is out of measurement range [{min_lux:.2f}, {max_lux:.2f}]")
        return None
    
    # 線形補間で角度を推定
    for i in range(len(angle_lux_pairs) - 1):
        angle1, lux1, dist1 = angle_lux_pairs[i]
        angle2, lux2, dist2 = angle_lux_pairs[i + 1]
        
        if min(lux1, lux2) <= current_lux <= max(lux1, lux2):
            if abs(lux2 - lux1) < 1.0:
                estimated_angle = (angle1 + angle2) / 2.0
            else:
                ratio = (current_lux - lux1) / (lux2 - lux1)
                estimated_angle = angle1 + ratio * (angle2 - angle1)
            
            # ★新規：推定角度の距離を補間
            estimated_distance = dist1 + ratio * (dist2 - dist1)
            print(f"  Estimated angle: {estimated_angle * 45:.1f}° (step {estimated_angle:.2f}), Est. distance: {estimated_distance:.1f}cm")
            
            # ★安全チェック：推定地点が近すぎる場合は却下
            if estimated_distance < SAFE_DISTANCE_MARGIN:
                print(f"  ⚠ Estimated position too close to obstacle, rejecting correction")
                return None
            
            return estimated_angle
    
    closest = min(angle_lux_pairs, key=lambda x: abs(x[1] - current_lux))
    print(f"  Using closest match: angle step {closest[0]} (Lux {closest[1]:.2f}, Dist {closest[2]:.1f}cm)")
    
    # ★安全チェック
    if closest[2] < SAFE_DISTANCE_MARGIN:
        print(f"  ⚠ Closest match too close to obstacle")
        return None
    
    return closest[0]


def correct_angle_if_needed(expected_lux, expected_angle_step, measurements):
    """
    前進前に角度補正が必要かチェックし、必要なら補正する
    ★追加：補正後に距離を再確認
    """
    current_lux = get_lux()
    current_ppfd = get_ppfd()
    
    # ★強化：距離の安全確認
    is_safe, current_distance = check_distance_safe(MIN_MOVE_DISTANCE)
    
    print(f"\n--- Angle correction check (Lux-based + Distance) ---")
    print(f"  Expected: Lux={expected_lux:.2f}, Angle={expected_angle_step*45}°")
    print(f"  Current:  Lux={current_lux:.2f}, PPFD={current_ppfd:.2f}, Distance={current_distance:.1f}cm")
    
    # ★距離が危険な場合は補正しない
    if not is_safe:
        print(f"  ⚠ Current distance unsafe, skipping angle correction")
        return expected_angle_step, False
    
    # Luxでの乖離度チェック
    deviation = abs(current_lux - expected_lux) / max(expected_lux, 1.0)
    print(f"  Lux Deviation: {deviation*100:.1f}%")
    
    if deviation < ANGLE_CORRECTION_THRESHOLD:
        print(f"  → No correction needed (deviation < {ANGLE_CORRECTION_THRESHOLD*100:.0f}%)")
        return expected_angle_step, False
    
    # 角度推定（Luxベース）
    print(f"  → Correction needed! Estimating actual angle from Lux...")
    estimated_angle = estimate_angle_from_lux(current_lux, measurements)
    
    if estimated_angle is None:
        print(f"  → Cannot estimate angle safely, using expected angle")
        return expected_angle_step, False
    
    # 補正角度を計算
    correction_step = round(estimated_angle - expected_angle_step)
    
    if correction_step == 0:
        print(f"  → Estimated angle matches expected, no correction needed")
        return expected_angle_step, False
    
    print(f"  → Correction: {correction_step} steps ({correction_step*45}°)")
    
    # 補正回転を実行
    if correction_step > 0:
        for i in range(abs(correction_step)):
            rotate_right()
            # ★各回転後に距離チェック
            temp_safe, temp_dist = check_distance_safe(SAFE_DISTANCE_MARGIN)
            if not temp_safe:
                print(f"  ⚠ Obstacle detected during correction, stopping rotation")
                # 戻す
                for j in range(i + 1):
                    rotate_left()
                return expected_angle_step, False
        print(f"  → Rotated right {abs(correction_step)*45}°")
    else:
        for i in range(abs(correction_step)):
            rotate_left()
            # ★各回転後に距離チェック
            temp_safe, temp_dist = check_distance_safe(SAFE_DISTANCE_MARGIN)
            if not temp_safe:
                print(f"  ⚠ Obstacle detected during correction, stopping rotation")
                # 戻す
                for j in range(i + 1):
                    rotate_right()
                return expected_angle_step, False
        print(f"  → Rotated left {abs(correction_step)*45}°")
    
    # 補正後の確認
    corrected_lux = get_lux()
    corrected_ppfd = get_ppfd()
    corrected_safe, corrected_distance = check_distance_safe(MIN_MOVE_DISTANCE)
    
    print(f"  After correction: Lux={corrected_lux:.2f}, PPFD={corrected_ppfd:.2f}, Distance={corrected_distance:.1f}cm, Safe={corrected_safe}")
    
    # ★補正後に距離が危険になった場合は元に戻す
    if not corrected_safe:
        print(f"  ⚠ Correction led to unsafe position, reverting")
        if correction_step > 0:
            for _ in range(abs(correction_step)):
                rotate_left()
        else:
            for _ in range(abs(correction_step)):
                rotate_right()
        return expected_angle_step, False
    
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
            'can_move': distance > SAFE_DISTANCE_MARGIN  # ★安全マージンを使用
        })
        
        print(f"  Re-scan Angle {angle_step*45}°: Lux={lux:.2f}, PPFD={ppfd:.2f}, Dist={distance:.1f}cm, Can move={distance > SAFE_DISTANCE_MARGIN}")
        
        if i < len(ANGLE_STEPS) - 1:
            rotate_right()
    
    for _ in range(2):
        rotate_left()
    
    movable = [m for m in measurements if m['can_move']]
    
    if not movable:
        print("  No movable direction found after re-scan → stay here")
        return None, measurements
    
    # ★距離も考慮して最良方向を選択
    # PPFDが高く、かつ距離が遠い方向を優先
    brightest = max(movable, key=lambda m: (m['ppfd'] * 0.7 + m['distance'] * 0.3))
    
    print(f"  Brightest movable direction: {brightest['angle_step']*45}° (Lux={brightest['lux']:.2f}, PPFD={brightest['ppfd']:.2f}, Dist={brightest['distance']:.1f}cm)")
    
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
                'can_move': distance > SAFE_DISTANCE_MARGIN  # ★安全マージン使用
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
        
        print(f"Best direction: {best['angle_step']*45}° (Lux={best['lux']:.2f}, PPFD={best['ppfd']:.2f}, Dist={best['distance']:.1f}cm)")
        
        rotation_angle = best['angle_step']
        
        if rotation_angle > 0:
            for _ in range(rotation_angle):
                rotate_right()
        elif rotation_angle < 0:
            for _ in range(-rotation_angle):
                rotate_left()
        
        # ★★★ 角度補正処理（距離監視強化版）★★★
        corrected_angle, was_corrected = correct_angle_if_needed(
            best['lux'],
            rotation_angle, 
            measurements
        )
        
        if was_corrected:
            rotation_angle = corrected_angle
        
        # ★★★ 前進直前の最終安全確認 ★★★
        print("\n--- Final safety check before moving ---")
        final_safe, final_distance = check_distance_safe(MIN_MOVE_DISTANCE)
        
        if not final_safe:
            print(f"⚠ Final safety check FAILED (distance={final_distance:.1f}cm)")
            print("→ Attempting alternative direction search")
            
            # 元の向きに戻す
            if rotation_angle > 0:
                for _ in range(rotation_angle):
                    rotate_left()
            elif rotation_angle < 0:
                for _ in range(-rotation_angle):
                    rotate_right()
            
            # 再スキャン
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
                brightest_movable['lux'],
                rotation_angle,
                rescan_measurements
            )
            
            if was_corrected:
                rotation_angle = corrected_angle
            
            # 再度最終安全確認
            final_safe, final_distance = check_distance_safe(MIN_MOVE_DISTANCE)
            
            if not final_safe:
                print("⚠ Still unsafe after re-scan → staying at current position")
                # 元の向きに戻す
                if rotation_angle > 0:
                    for _ in range(rotation_angle):
                        rotate_left()
                elif rotation_angle < 0:
                    for _ in range(-rotation_angle):
                        rotate_right()
                return get_ppfd()
        
        # ★前進実行
        print(f"✓ Safe to move forward (distance={final_distance:.1f}cm)")
        moved = move_forward()
        
        if not moved:
            print("⚠ Move forward failed despite safety check")
            # 元の向きに戻す
            if rotation_angle > 0:
                for _ in range(rotation_angle):
                    rotate_left()
            elif rotation_angle < 0:
                for _ in range(-rotation_angle):
                    rotate_right()
            return get_ppfd()
        
        print(f"✓ Successfully moved forward")
        
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
    print(f"Angle correction threshold (Lux-based): {ANGLE_CORRECTION_THRESHOLD*100:.0f}%")
    print(f"Safe distance margin: {SAFE_DISTANCE_MARGIN}cm")
    
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