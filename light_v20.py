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
TARGET_DLI = 0.0684
ACCUMULATION_HOURS = 10
LUX_TO_PPFD = 0.0281

DLI_TARGET_UMOL = TARGET_DLI * 1_000_000
TOTAL_SECONDS = ACCUMULATION_HOURS * 3600

# ===== 制御パラメータ =====
BRIGHTNESS_TOLERANCE = 0.1
PPFD_DROP_RATIO = 0.8
DARK_RATIO = 0.7  # 既存の暗化判定
SHADOW_ESCAPE_RATIO = 0.3  # ★新規：影エリア脱出判定（グローバル最大の30%以下）
ANGLE_CORRECTION_THRESHOLD = 0.15

TURN_POWER = 3000
TURN_STEP_TIME = 0.28
FORWARD_POWER = 3000
FORWARD_TIME = 0.3
MIN_MOVE_DISTANCE = 20
SAFE_DISTANCE_MARGIN = 25

# ===== 30秒ごとの表示用 =====
PRINT_INTERVAL = 30

# ===== 状態 =====
accumulated_umol = 0
start_time = time.time()
last_integral_time = time.time()
last_print_time = time.time()
best_global_avg = 0

# ★★★ 新規：グローバル明るさ追跡 ★★★
global_max_lux = 0  # これまでに観測した最大明るさ
global_max_history = []  # 明るさ履歴（時系列）


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
    distances = []
    for _ in range(3):
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


# ★★★ 新規：グローバル明るさ更新 ★★★
def update_global_brightness(current_lux):
    """測定した明るさをグローバル履歴に記録"""
    global global_max_lux, global_max_history
    
    if current_lux > global_max_lux:
        global_max_lux = current_lux
        print(f"  ★ New global maximum: {global_max_lux:.2f} lux")
    
    # 履歴に追加（最新100件を保持）
    global_max_history.append(current_lux)
    if len(global_max_history) > 100:
        global_max_history.pop(0)


# ★★★ 新規：影エリア判定 ★★★
def is_in_shadow_area(measurements):
    """
    現在のスキャン結果から影エリアにいるか判定
    
    判定基準：
    1. 全方向の平均がグローバル最大の30%以下
    2. かつ、グローバル最大が十分明るい（50 lux以上）
    """
    global global_max_lux
    
    if global_max_lux < 50:  # まだ十分なデータがない
        return False
    
    avg_lux = sum(m['lux'] for m in measurements) / len(measurements)
    max_current = max(m['lux'] for m in measurements)
    
    shadow_threshold = global_max_lux * SHADOW_ESCAPE_RATIO
    
    is_shadow = avg_lux < shadow_threshold
    
    if is_shadow:
        print(f"\n⚠️ SHADOW AREA DETECTED")
        print(f"  Current avg: {avg_lux:.2f} lux")
        print(f"  Current max: {max_current:.2f} lux")
        print(f"  Global max: {global_max_lux:.2f} lux")
        print(f"  Threshold: {shadow_threshold:.2f} lux ({SHADOW_ESCAPE_RATIO*100:.0f}% of global)")
    
    return is_shadow


# ★★★ 新規：影脱出行動 ★★★
def escape_shadow_area():
    """
    影エリアから脱出するための行動
    
    戦略：
    1. ランダム方向に180度回転
    2. 3回前進を試みる
    3. 各移動後に明るさを確認
    """
    print("\n=== SHADOW ESCAPE PROCEDURE ===")
    print("  Attempting to escape from dark area...")
    
    # 180度回転
    print("  Step 1: Rotate 180°")
    rotate_180()
    integrate_light()
    
    # 3回前進を試みる
    escape_success = False
    for attempt in range(3):
        print(f"  Step {attempt + 2}: Forward attempt {attempt + 1}/3")
        
        if move_forward():
            integrate_light()
            current_lux = get_lux()
            update_global_brightness(current_lux)
            
            print(f"    → Moved, current lux: {current_lux:.2f}")
            
            # 明るさが改善したか確認
            if global_max_lux > 0 and current_lux > global_max_lux * SHADOW_ESCAPE_RATIO:
                print(f"    ✓ Brightness improved! Escape successful")
                escape_success = True
                break
        else:
            print(f"    ✗ Cannot move forward")
            # 45度回転して別方向を試す
            rotate_right()
            integrate_light()
    
    if escape_success:
        print("=== Shadow escape SUCCESSFUL ===\n")
        return True
    else:
        print("=== Shadow escape FAILED (will continue normal exploration) ===\n")
        return False


# ===== 角度補正（Luxベース + 距離監視強化） =====
def estimate_angle_from_lux(current_lux, measurements):
    angle_lux_pairs = [(m['angle_step'], m['lux'], m['distance']) for m in measurements]
    angle_lux_pairs.sort(key=lambda x: x[0])
    
    min_lux = min(p[1] for p in angle_lux_pairs)
    max_lux = max(p[1] for p in angle_lux_pairs)
    
    if current_lux < min_lux * 0.8 or current_lux > max_lux * 1.2:
        print(f"  Current Lux {current_lux:.2f} is out of measurement range [{min_lux:.2f}, {max_lux:.2f}]")
        return None
    
    for i in range(len(angle_lux_pairs) - 1):
        angle1, lux1, dist1 = angle_lux_pairs[i]
        angle2, lux2, dist2 = angle_lux_pairs[i + 1]
        
        if min(lux1, lux2) <= current_lux <= max(lux1, lux2):
            if abs(lux2 - lux1) < 1.0:
                estimated_angle = (angle1 + angle2) / 2.0
                ratio = 0.5
            else:
                ratio = (current_lux - lux1) / (lux2 - lux1)
                estimated_angle = angle1 + ratio * (angle2 - angle1)
            
            estimated_distance = dist1 + ratio * (dist2 - dist1)
            print(f"  Estimated angle: {estimated_angle * 45:.1f}° (step {estimated_angle:.2f}), Est. distance: {estimated_distance:.1f}cm")
            
            if estimated_distance < SAFE_DISTANCE_MARGIN:
                print(f"  ⚠ Estimated position too close to obstacle, rejecting correction")
                return None
            
            return estimated_angle
    
    closest = min(angle_lux_pairs, key=lambda x: abs(x[1] - current_lux))
    print(f"  Using closest match: angle step {closest[0]} (Lux {closest[1]:.2f}, Dist {closest[2]:.1f}cm)")
    
    if closest[2] < SAFE_DISTANCE_MARGIN:
        print(f"  ⚠ Closest match too close to obstacle")
        return None
    
    return closest[0]

def correct_angle_if_needed(expected_lux, expected_angle_step, measurements):
    current_lux = get_lux()
    current_ppfd = get_ppfd()
    
    is_safe, current_distance = check_distance_safe(MIN_MOVE_DISTANCE)
    
    print(f"\n--- Angle correction check (Lux-based + Distance) ---")
    print(f"  Expected: Lux={expected_lux:.2f}, Angle={expected_angle_step*45}°")
    print(f"  Current:  Lux={current_lux:.2f}, PPFD={current_ppfd:.2f}, Distance={current_distance:.1f}cm")
    
    if not is_safe:
        print(f"  ⚠ Current distance unsafe, skipping angle correction")
        return expected_angle_step, False
    
    deviation = abs(current_lux - expected_lux) / max(expected_lux, 1.0)
    print(f"  Lux Deviation: {deviation*100:.1f}%")
    
    if deviation < ANGLE_CORRECTION_THRESHOLD:
        print(f"  → No correction needed (deviation < {ANGLE_CORRECTION_THRESHOLD*100:.0f}%)")
        return expected_angle_step, False
    
    print(f"  → Correction needed! Estimating actual angle from Lux...")
    estimated_angle = estimate_angle_from_lux(current_lux, measurements)
    
    if estimated_angle is None:
        print(f"  → Cannot estimate angle safely, using expected angle")
        return expected_angle_step, False
    
    correction_step = round(estimated_angle - expected_angle_step)
    
    if correction_step == 0:
        print(f"  → Estimated angle matches expected, no correction needed")
        return expected_angle_step, False
    
    print(f"  → Correction: {correction_step} steps ({correction_step*45}°)")
    
    # ★★★ 修正箇所 ★★★
    if correction_step > 0:
        for i in range(abs(correction_step)):
            rotate_right()
            temp_safe, temp_dist = check_distance_safe(SAFE_DISTANCE_MARGIN)
            if not temp_safe:
                print(f"  ⚠ Obstacle detected during correction, stopping rotation")
                for j in range(i + 1):
                    rotate_left()
                return expected_angle_step, False
        print(f"  → Rotated right {abs(correction_step)*45}°")  # ★修正：forループの外
    else:
        for i in range(abs(correction_step)):
            rotate_left()
            temp_safe, temp_dist = check_distance_safe(SAFE_DISTANCE_MARGIN)
            if not temp_safe:
                print(f"  ⚠ Obstacle detected during correction, stopping rotation")
                for j in range(i + 1):
                    rotate_right()
                return expected_angle_step, False
        print(f"  → Rotated left {abs(correction_step)*45}°")  # ★修正：forループの外
    
    corrected_lux = get_lux()
    corrected_ppfd = get_ppfd()
    corrected_safe, corrected_distance = check_distance_safe(MIN_MOVE_DISTANCE)
    
    print(f"  After correction: Lux={corrected_lux:.2f}, PPFD={corrected_ppfd:.2f}, Distance={corrected_distance:.1f}cm, Safe={corrected_safe}")
    
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
        
        # ★グローバル明るさ更新
        update_global_brightness(lux)
        
        measurements.append({
            'index': i,
            'angle_step': angle_step,
            'lux': lux,
            'ppfd': ppfd,
            'distance': distance,
            'can_move': distance > SAFE_DISTANCE_MARGIN
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
    
    uturn_retry_count = 0
    MAX_UTURN_RETRIES = 2
    
    shadow_escape_count = 0  # ★新規：影脱出試行回数
    MAX_SHADOW_ESCAPES = 2  # 最大2回まで
        
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
            
            # ★グローバル明るさ更新
            update_global_brightness(lux)
            
            measurements.append({
                'index': i,
                'angle_step': angle_step,
                'lux': lux,
                'ppfd': ppfd,
                'distance': distance,
                'can_move': distance > SAFE_DISTANCE_MARGIN
            })
            
            print(f"Angle {angle_step*45}°: Lux={lux:.2f}, PPFD={ppfd:.2f}, Dist={distance:.1f}cm")
            
            if i < len(ANGLE_STEPS) - 1:
                rotate_right()
        
        for _ in range(2):
            rotate_left()
        
        # ★★★ 新規：影エリア判定 ★★★
        if is_in_shadow_area(measurements) and shadow_escape_count < MAX_SHADOW_ESCAPES:
            shadow_escape_count += 1
            print(f"→ Shadow escape attempt {shadow_escape_count}/{MAX_SHADOW_ESCAPES}")
            
            if escape_shadow_area():
                # 脱出成功：状態リセットして探索継続
                best_global_avg = 0
                brightest_ppfd = 0
                move_history = []
                previous_reached_ppfd = 0
                no_improvement_count = 0
                uturn_retry_count = 0
                shadow_escape_count = 0  # リセット
                continue
            else:
                # 脱出失敗：通常探索を継続
                print("  Continuing with normal exploration despite shadow...")
        
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
            print("No movable direction in initial scan")
            
            if uturn_retry_count < MAX_UTURN_RETRIES:
                uturn_retry_count += 1
                print(f"→ Attempting U-turn escape (retry {uturn_retry_count}/{MAX_UTURN_RETRIES})")
                print("  Rotating 180° to explore opposite direction...")
                
                rotate_180()
                integrate_light()
                
                if move_forward():
                    print("  ✓ Successfully escaped, continuing exploration")
                    best_global_avg = 0
                    brightest_ppfd = 0
                    move_history = []
                    previous_reached_ppfd = 0
                    no_improvement_count = 0
                    uturn_retry_count = 0
                    continue
                else:
                    print("  ✗ Cannot move forward even after U-turn")
                    continue
            else:
                print(f"→ Already tried U-turn {MAX_UTURN_RETRIES} times")
                print("→ Completely blocked, staying at current position")
                return get_ppfd()
        
        uturn_retry_count = 0
        
        best = min(movable, key=lambda m: abs(m['ppfd'] - target_ppfd))
        
        print(f"Best direction: {best['angle_step']*45}° (Lux={best['lux']:.2f}, PPFD={best['ppfd']:.2f}, Dist={best['distance']:.1f}cm)")
        
        rotation_angle = best['angle_step']
        
        if rotation_angle > 0:
            for _ in range(rotation_angle):
                rotate_right()
        elif rotation_angle < 0:
            for _ in range(-rotation_angle):
                rotate_left()
        
        corrected_angle, was_corrected = correct_angle_if_needed(
            best['lux'],
            rotation_angle, 
            measurements
        )
        
        if was_corrected:
            rotation_angle = corrected_angle
        
        print("\n--- Final safety check before moving ---")
        final_safe, final_distance = check_distance_safe(MIN_MOVE_DISTANCE)
        
        if not final_safe:
            print(f"⚠ Final safety check FAILED (distance={final_distance:.1f}cm)")
            print("→ Attempting alternative direction search")
            
            if rotation_angle > 0:
                for _ in range(rotation_angle):
                    rotate_left()
            elif rotation_angle < 0:
                for _ in range(-rotation_angle):
                    rotate_right()
            
            brightest_movable, rescan_measurements = rescan_for_brightest_movable()
            
            if brightest_movable is None:
                print("No alternative direction available")
                
                if uturn_retry_count < MAX_UTURN_RETRIES:
                    uturn_retry_count += 1
                    print(f"→ Attempting U-turn escape (retry {uturn_retry_count}/{MAX_UTURN_RETRIES})")
                    print("  Rotating 180° to explore opposite direction...")
                    
                    rotate_180()
                    integrate_light()
                    
                    if move_forward():
                        print("  ✓ Successfully escaped, continuing exploration")
                        best_global_avg = 0
                        brightest_ppfd = 0
                        move_history = []
                        previous_reached_ppfd = 0
                        no_improvement_count = 0
                        uturn_retry_count = 0
                        continue
                    else:
                        print("  ✗ Cannot move forward even after U-turn")
                        continue
                else:
                    print(f"→ Already tried U-turn {MAX_UTURN_RETRIES} times, staying here")
                    return get_ppfd()
            
            rotation_angle = brightest_movable['angle_step']
            
            if rotation_angle > 0:
                for _ in range(rotation_angle):
                    rotate_right()
            elif rotation_angle < 0:
                for _ in range(-rotation_angle):
                    rotate_left()
            
            corrected_angle, was_corrected = correct_angle_if_needed(
                brightest_movable['lux'],
                rotation_angle,
                rescan_measurements
            )
            
            if was_corrected:
                rotation_angle = corrected_angle
            
            final_safe, final_distance = check_distance_safe(MIN_MOVE_DISTANCE)
            
            if not final_safe:
                print("⚠ Still unsafe after re-scan → trying U-turn as last resort")
                
                if rotation_angle > 0:
                    for _ in range(rotation_angle):
                        rotate_left()
                elif rotation_angle < 0:
                    for _ in range(-rotation_angle):
                        rotate_right()
                
                if uturn_retry_count < MAX_UTURN_RETRIES:
                    uturn_retry_count += 1
                    print(f"→ Last resort U-turn (retry {uturn_retry_count}/{MAX_UTURN_RETRIES})")
                    rotate_180()
                    integrate_light()
                    
                    if move_forward():
                        print("  ✓ Escaped via last resort U-turn")
                        best_global_avg = 0
                        brightest_ppfd = 0
                        move_history = []
                        previous_reached_ppfd = 0
                        no_improvement_count = 0
                        uturn_retry_count = 0
                        continue
                    else:
                        print("  ✗ Last resort U-turn also failed")
                        continue
                else:
                    return get_ppfd()
        
        print(f"✓ Safe to move forward (distance={final_distance:.1f}cm)")
        moved = move_forward()
        
        if not moved:
            print("⚠ Move forward failed despite safety check")
            
            if rotation_angle > 0:
                for _ in range(rotation_angle):
                    rotate_left()
            elif rotation_angle < 0:
                for _ in range(-rotation_angle):
                    rotate_right()
            
            if uturn_retry_count < MAX_UTURN_RETRIES:
                uturn_retry_count += 1
                print(f"→ Attempting U-turn after move failure (retry {uturn_retry_count}/{MAX_UTURN_RETRIES})")
                rotate_180()
                integrate_light()
                
                if move_forward():
                    print("  ✓ Escaped via U-turn after move failure")
                    best_global_avg = 0
                    brightest_ppfd = 0
                    move_history = []
                    previous_reached_ppfd = 0
                    no_improvement_count = 0
                    uturn_retry_count = 0
                    continue
                else:
                    continue
            else:
                return get_ppfd()
        
        print(f"✓ Successfully moved forward")
        
        integrate_light()
        print_status_if_needed()
        
        # ★移動後も明るさ更新
        current_lux = get_lux()
        update_global_brightness(current_lux)
        
        reached_ppfd = get_ppfd()
        
        print(f"Reached PPFD: {reached_ppfd:.2f}, Target: {target_ppfd:.2f}")
        
        if reached_ppfd > brightest_ppfd:
            brightest_ppfd = reached_ppfd
            move_history = []
            print(f"New brightest point: {brightest_ppfd:.2f}")
            shadow_escape_count = 0  # 成功したらリセット
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
    target_ppfd_constant = DLI_TARGET_UMOL / TOTAL_SECONDS
    
    print("=== Light accumulation control started ===")
    print(f"Target DLI: {TARGET_DLI} mol/m²/d ({DLI_TARGET_UMOL} µmol/m²)")
    print(f"Target time: {ACCUMULATION_HOURS}h ({TOTAL_SECONDS} seconds)")
    print(f"★ Target PPFD (constant): {target_ppfd_constant:.2f} µmol/m²/s")
    print(f"  (If maintained constantly for {ACCUMULATION_HOURS}h, achieves {TARGET_DLI} mol/m²/d)")
    print(f"Lux to PPFD conversion: {LUX_TO_PPFD}")
    print(f"Angle correction threshold (Lux-based): {ANGLE_CORRECTION_THRESHOLD*100:.0f}%")
    print(f"Safe distance margin: {SAFE_DISTANCE_MARGIN}cm")
    print(f"★ Shadow escape threshold: {SHADOW_ESCAPE_RATIO*100:.0f}% of global max")
    print()
    
    adjustment_ppfd = 0

    while True:
        integrate_light()

        elapsed = time.time() - start_time
        remaining_time = TOTAL_SECONDS - elapsed

        if remaining_time <= 0:
            print("=== 10h complete ===")
            print(f"Accumulated: {accumulated_umol:.2f} µmol/m² ({accumulated_umol / 1_000_000:.3f} mol/m²)")
            print(f"Target was: {DLI_TARGET_UMOL} µmol/m² ({TARGET_DLI} mol/m²/d)")
            print(f"Global max lux observed: {global_max_lux:.2f}")
            achievement_rate = (accumulated_umol / DLI_TARGET_UMOL) * 100
            print(f"Achievement rate: {achievement_rate:.1f}%")
            break

        remaining_umol = DLI_TARGET_UMOL - accumulated_umol
        required_ppfd = remaining_umol / max(1, remaining_time)
        
        required_ppfd += adjustment_ppfd

        print(f"\n--- New cycle ---")
        print(f"Elapsed: {elapsed/3600:.2f}h, Remaining: {remaining_time/3600:.2f}h")
        print(f"Accumulated: {accumulated_umol:.2f} µmol/m² ({accumulated_umol / 1_000_000:.3f} mol/m²)")
        print(f"Remaining: {remaining_umol:.2f} µmol/m²")
        print(f"Required PPFD (dynamic): {required_ppfd:.2f} µmol/m²/s")
        print(f"Target PPFD (constant): {target_ppfd_constant:.2f} µmol/m²/s")
        print(f"Global max lux: {global_max_lux:.2f}")

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
            print(f"Target was: {DLI_TARGET_UMOL} µmol/m² ({TARGET_DLI} mol/m²/d)")
            print(f"Global max lux observed: {global_max_lux:.2f}")
            achievement_rate = (accumulated_umol / DLI_TARGET_UMOL) * 100
            print(f"Achievement rate: {achievement_rate:.1f}%")
            break

except KeyboardInterrupt:
    stop()
    print("\n=== Stopped manually ===")
    print(f"Accumulated: {accumulated_umol:.2f} µmol/m² ({accumulated_umol / 1_000_000:.3f} mol/m²)")
    print(f"Target was: {DLI_TARGET_UMOL} µmol/m² ({TARGET_DLI} mol/m²/d)")
    print(f"Global max lux observed: {global_max_lux:.2f}")
    if DLI_TARGET_UMOL > 0:
        achievement_rate = (accumulated_umol / DLI_TARGET_UMOL) * 100
        print(f"Achievement rate: {achievement_rate:.1f}%")

finally:
    stop()