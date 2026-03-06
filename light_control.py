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
DARK_RATIO = 0.7
SHADOW_ESCAPE_RATIO = 0.3
ANGLE_CORRECTION_THRESHOLD = 0.15

TURN_POWER = 3000
TURN_STEP_TIME = 0.28           # 45度相当の回転時間
FORWARD_POWER = 3000
FORWARD_TIME = 0.3
MIN_MOVE_DISTANCE = 20
SAFE_DISTANCE_MARGIN = 25

# ===== 影エリア判定パラメータ =====
STARTUP_DARK_THRESHOLD = 8 #50     # global_max_luxが不十分と判断する閾値
RELATIVE_SHADOW_RATIO = 0.4     # スキャン内最大値の40%以下なら影と判断
MAX_UNIFORM_DARK = 3            # 一様暗スキャン連続N回で強制脱出

# ===== 30秒ごとの表示用 =====
PRINT_INTERVAL = 30

# ===== 状態 =====
accumulated_umol = 0
start_time = time.time()
last_integral_time = time.time()
last_print_time = time.time()
best_global_avg = 0

global_max_lux = 0
global_max_history = []
uniform_dark_count = 0          # 一様暗スキャン連続回数
scan_count = 0                  # 総スキャン回数


# ===== 基本動作 =====
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


def rotate_partial(fraction):
    """
    fraction: -1.0〜+1.0 の割合で TURN_STEP_TIME 分だけ回転
    正なら右、負なら左。|fraction| < 0.1 は無視。
    """
    if abs(fraction) < 0.1:
        print(f"  Partial rotation skipped (fraction={fraction:.3f} too small)")
        return

    duration = TURN_STEP_TIME * abs(fraction)
    direction = "right" if fraction > 0 else "left"
    print(f"  Partial rotation: {direction}, {fraction:.2f} steps "
          f"({duration:.3f}s = {fraction * 45:.1f}°)")

    if fraction > 0:
        motor.setMotorModel(TURN_POWER, -TURN_POWER)
    else:
        motor.setMotorModel(-TURN_POWER, TURN_POWER)

    time.sleep(duration)
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

    print(f"  Distance check: {avg_distance:.1f}cm (min: {min_distance}cm) "
          f"→ {'SAFE' if is_safe else 'BLOCKED'}")
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
    global last_print_time

    now = time.time()
    if now - last_print_time >= PRINT_INTERVAL:
        current_lux = get_lux()
        current_ppfd = get_ppfd()
        print(f"[Status] Lux={current_lux:.2f}, PPFD={current_ppfd:.2f} µmol/m²/s, "
              f"Accumulated={accumulated_umol:.2f} µmol/m²")
        last_print_time = now


# ===== グローバル明るさ更新 =====
def update_global_brightness(current_lux):
    global global_max_lux, global_max_history

    if current_lux > global_max_lux:
        global_max_lux = current_lux
        print(f"  ★ New global maximum: {global_max_lux:.2f} lux")

    global_max_history.append(current_lux)
    if len(global_max_history) > 100:
        global_max_history.pop(0)


# ===== 影エリア判定（改訂版） =====
def is_in_shadow_area(measurements):
    """
    影エリア判定。3つのモードで対応：

    モードA：global_max_lux >= STARTUP_DARK_THRESHOLD
        → グローバル最大値との比較（従来通り）

    モードB-Critical：全方向がほぼ0 lux
        → センサ異常か完全遮光 → 強制脱出

    モードB：起動直後でグローバルデータ不足
        → スキャン内の相対比較
        → 方向ムラが大きい場合は影と判断
        → 一様に暗い場合はN回連続で強制脱出
    """
    global uniform_dark_count, scan_count

    scan_count += 1
    avg_lux = sum(m['lux'] for m in measurements) / len(measurements)
    max_current = max(m['lux'] for m in measurements)

    # --- モードA：十分なグローバルデータあり ---
    if global_max_lux >= STARTUP_DARK_THRESHOLD:
        shadow_threshold = global_max_lux * SHADOW_ESCAPE_RATIO
        is_shadow = avg_lux < shadow_threshold

        if is_shadow:
            print(f"\n⚠️ [ModeA] SHADOW AREA DETECTED")
            print(f"  Current avg: {avg_lux:.2f} lux")
            print(f"  Global max:  {global_max_lux:.2f} lux")
            print(f"  Threshold:   {shadow_threshold:.2f} lux "
                  f"({SHADOW_ESCAPE_RATIO * 100:.0f}% of global)")
        return is_shadow

    # --- モードB-Critical：全方向ほぼ0 ---
    if max_current < 1.0:
        print(f"\n⚠️ [ModeB-Critical] All directions near zero lux "
              f"(max={max_current:.2f}) → forcing escape")
        return True

    # --- モードB：相対比較 ---
    relative_ratio = avg_lux / max_current

    # 方向ムラが大きい → 明るい方向が存在 → 影と判断して脱出
    if relative_ratio < RELATIVE_SHADOW_RATIO:
        uniform_dark_count = 0
        print(f"\n⚠️ [ModeB] STARTUP SHADOW DETECTED (directional unevenness)")
        print(f"  Scan avg:  {avg_lux:.2f} lux")
        print(f"  Scan max:  {max_current:.2f} lux")
        print(f"  Ratio:     {relative_ratio:.2f} (threshold: {RELATIVE_SHADOW_RATIO})")
        print(f"  Global max so far: {global_max_lux:.2f} (< {STARTUP_DARK_THRESHOLD})")
        return True

    # 一様に暗い → カウントを積み上げてN回で強制脱出
    if avg_lux < STARTUP_DARK_THRESHOLD:
        uniform_dark_count += 1
        print(f"  [ModeB] Uniformly dark environment: "
              f"count={uniform_dark_count}/{MAX_UNIFORM_DARK} "
              f"(avg={avg_lux:.2f}, ratio={relative_ratio:.2f})")

        if uniform_dark_count >= MAX_UNIFORM_DARK:
            uniform_dark_count = 0
            print(f"  [ModeB] Uniformly dark for {MAX_UNIFORM_DARK} scans → forcing escape")
            return True

        return False

    # 十分明るい → 影でない
    uniform_dark_count = 0
    return False


# ===== 影脱出行動 =====
def escape_shadow_area():
    print("\n=== SHADOW ESCAPE PROCEDURE ===")
    print("  Attempting to escape from dark area...")

    print("  Step 1: Rotate 180°")
    rotate_180()
    integrate_light()

    escape_success = False
    for attempt in range(3):
        print(f"  Step {attempt + 2}: Forward attempt {attempt + 1}/3")

        if move_forward():
            integrate_light()
            current_lux = get_lux()
            update_global_brightness(current_lux)

            print(f"    → Moved, current lux: {current_lux:.2f}")

            if global_max_lux > 0 and current_lux > global_max_lux * SHADOW_ESCAPE_RATIO:
                print(f"    ✓ Brightness improved! Escape successful")
                escape_success = True
                break
        else:
            print(f"    ✗ Cannot move forward")
            rotate_right()
            integrate_light()

    if escape_success:
        print("=== Shadow escape SUCCESSFUL ===\n")
        return True
    else:
        print("=== Shadow escape FAILED (will continue normal exploration) ===\n")
        return False


# ===== 角度推定（改訂版） =====
def estimate_angle_from_lux(current_lux, measurements):
    """
    現在のlux値からスキャン結果を線形補間して推定角度を返す。

    複数ペアにマッチした場合はペアの平均luxが最大の候補を選択。
    障害物距離は補間せず、両端点の最小値で保守的に安全確認のみ行う。
    """
    angle_lux_pairs = [
        (m['angle_step'], m['lux'], m['distance']) for m in measurements
    ]
    angle_lux_pairs.sort(key=lambda x: x[0])

    min_lux = min(p[1] for p in angle_lux_pairs)
    max_lux = max(p[1] for p in angle_lux_pairs)

    if current_lux < min_lux * 0.8 or current_lux > max_lux * 1.2:
        print(f"  Current Lux {current_lux:.2f} out of range "
              f"[{min_lux:.2f}, {max_lux:.2f}]")
        return None

    # 全マッチ候補を収集
    candidates = []

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

            # 距離は補間せず両端点の最小値で保守的に安全確認
            min_dist = min(dist1, dist2)
            avg_lux_of_pair = (lux1 + lux2) / 2.0

            candidates.append({
                'angle': estimated_angle,
                'min_dist': min_dist,
                'avg_lux': avg_lux_of_pair,
                'pair': (angle1, angle2),
            })
            print(f"  Candidate: angle={estimated_angle:.2f} "
                  f"({estimated_angle * 45:.1f}°), "
                  f"min_dist={min_dist:.1f}cm "
                  f"(raw: {dist1:.1f}/{dist2:.1f}cm), "
                  f"pair_avg_lux={avg_lux_of_pair:.2f}")

    # マッチなし → 最近傍にフォールバック
    if not candidates:
        closest = min(angle_lux_pairs, key=lambda x: abs(x[1] - current_lux))
        print(f"  No interpolation match, using closest: "
              f"angle={closest[0]}, lux={closest[1]:.2f}, dist={closest[2]:.1f}cm")
        if closest[2] < SAFE_DISTANCE_MARGIN:
            print(f"  ⚠ Closest match too close to obstacle")
            return None
        return closest[0]

    # 障害物チェック：補間なしの保守的距離で判定
    safe_candidates = [c for c in candidates if c['min_dist'] >= SAFE_DISTANCE_MARGIN]

    if not safe_candidates:
        print(f"  All candidates too close to obstacle")
        return None

    if len(safe_candidates) == 1:
        chosen = safe_candidates[0]
        print(f"  Single candidate: angle={chosen['angle']:.2f} "
              f"({chosen['angle'] * 45:.1f}°)")
        return chosen['angle']

    # 複数候補 → ペアの平均luxが最大の候補を選択
    chosen = max(safe_candidates, key=lambda c: c['avg_lux'])
    print(f"  Multiple candidates ({len(safe_candidates)}), "
          f"choosing brightest pair: "
          f"angle={chosen['angle']:.2f} ({chosen['angle'] * 45:.1f}°), "
          f"pair_avg_lux={chosen['avg_lux']:.2f}, "
          f"min_dist={chosen['min_dist']:.1f}cm")

    return chosen['angle']


# ===== 角度補正（改訂版） =====
def correct_angle_if_needed(expected_lux, expected_angle_step, measurements):
    """
    回転後のlux偏差を確認し、ずれていれば整数ステップ＋小数部分回転で補正する。
    """
    current_lux = get_lux()
    current_ppfd = get_ppfd()

    is_safe, current_distance = check_distance_safe(MIN_MOVE_DISTANCE)

    print(f"\n--- Angle correction check (Lux-based + Partial rotation) ---")
    print(f"  Expected: Lux={expected_lux:.2f}, Angle={expected_angle_step * 45}°")
    print(f"  Current:  Lux={current_lux:.2f}, PPFD={current_ppfd:.2f}, "
          f"Distance={current_distance:.1f}cm")

    if not is_safe:
        print(f"  ⚠ Unsafe distance, skipping correction")
        return expected_angle_step, False

    deviation = abs(current_lux - expected_lux) / max(expected_lux, 1.0)
    print(f"  Lux Deviation: {deviation * 100:.1f}%")

    if deviation < ANGLE_CORRECTION_THRESHOLD:
        print(f"  → No correction needed (deviation < "
              f"{ANGLE_CORRECTION_THRESHOLD * 100:.0f}%)")
        return expected_angle_step, False

    print(f"  → Correction needed. Estimating actual angle from Lux...")
    estimated_angle = estimate_angle_from_lux(current_lux, measurements)

    if estimated_angle is None:
        print(f"  → Cannot estimate angle safely, using expected angle")
        return expected_angle_step, False

    # 小数のまま差分を計算
    raw_correction = estimated_angle - expected_angle_step
    int_correction = int(raw_correction)            # 整数部：45度刻み
    frac_correction = raw_correction - int_correction  # 小数部：部分回転

    print(f"  Raw correction: {raw_correction:.3f} steps ({raw_correction * 45:.1f}°)")
    print(f"  → Integer part:  {int_correction} steps ({int_correction * 45}°)")
    print(f"  → Fraction part: {frac_correction:.3f} steps "
          f"({frac_correction * 45:.1f}°)")

    if int_correction == 0 and abs(frac_correction) < 0.1:
        print(f"  → Correction too small, skipping")
        return expected_angle_step, False

    # --- 整数部の回転 ---
    if int_correction > 0:
        for i in range(int_correction):
            rotate_right()
            temp_safe, _ = check_distance_safe(SAFE_DISTANCE_MARGIN)
            if not temp_safe:
                print(f"  ⚠ Obstacle during integer correction at step {i+1}, reverting")
                for _ in range(i + 1):
                    rotate_left()
                return expected_angle_step, False
        print(f"  → Rotated right {int_correction * 45}°")

    elif int_correction < 0:
        for i in range(-int_correction):
            rotate_left()
            temp_safe, _ = check_distance_safe(SAFE_DISTANCE_MARGIN)
            if not temp_safe:
                print(f"  ⚠ Obstacle during integer correction at step {i+1}, reverting")
                for _ in range(i + 1):
                    rotate_right()
                return expected_angle_step, False
        print(f"  → Rotated left {-int_correction * 45}°")

    # --- 小数部の部分回転 ---
    if abs(frac_correction) >= 0.1:
        temp_safe, _ = check_distance_safe(SAFE_DISTANCE_MARGIN)
        if temp_safe:
            rotate_partial(frac_correction)
        else:
            print(f"  ⚠ Obstacle detected before partial rotation, skipping")

    # --- 補正後の安全確認 ---
    corrected_lux = get_lux()
    corrected_safe, corrected_distance = check_distance_safe(MIN_MOVE_DISTANCE)

    print(f"  After correction: Lux={corrected_lux:.2f}, "
          f"Distance={corrected_distance:.1f}cm, Safe={corrected_safe}")

    if not corrected_safe:
        print(f"  ⚠ Unsafe after correction, reverting all")
        # 整数部を戻す
        if int_correction > 0:
            for _ in range(int_correction):
                rotate_left()
        elif int_correction < 0:
            for _ in range(-int_correction):
                rotate_right()
        # 小数部を戻す（符号反転）
        if abs(frac_correction) >= 0.1:
            rotate_partial(-frac_correction)
        return expected_angle_step, False

    # 小数込みの補正量を返す
    total_correction = int_correction + frac_correction
    return expected_angle_step + total_correction, True


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

        update_global_brightness(lux)

        measurements.append({
            'index': i,
            'angle_step': angle_step,
            'lux': lux,
            'ppfd': ppfd,
            'distance': distance,
            'can_move': distance > SAFE_DISTANCE_MARGIN
        })

        print(f"  Re-scan Angle {angle_step * 45}°: Lux={lux:.2f}, "
              f"PPFD={ppfd:.2f}, Dist={distance:.1f}cm, "
              f"Can move={distance > SAFE_DISTANCE_MARGIN}")

        if i < len(ANGLE_STEPS) - 1:
            rotate_right()

    for _ in range(2):
        rotate_left()

    movable = [m for m in measurements if m['can_move']]

    if not movable:
        print("  No movable direction found after re-scan → stay here")
        return None, measurements

    brightest = max(movable, key=lambda m: m['ppfd'])

    print(f"  Brightest movable direction: {brightest['angle_step'] * 45}° "
          f"(Lux={brightest['lux']:.2f}, PPFD={brightest['ppfd']:.2f}, "
          f"Dist={brightest['distance']:.1f}cm)")

    return brightest, measurements


# ===== Uターン脱出ヘルパー =====
def attempt_uturn_escape(uturn_retry_count, max_retries,
                         best_global_avg_ref, brightest_ppfd_ref,
                         move_history_ref):
    """
    Uターンして前進を試みる共通処理。
    成功時は状態をリセットして True を返す。
    """
    if uturn_retry_count >= max_retries:
        print(f"→ Already tried U-turn {max_retries} times, giving up")
        return False, uturn_retry_count

    uturn_retry_count += 1
    print(f"→ U-turn escape attempt {uturn_retry_count}/{max_retries}")
    rotate_180()
    integrate_light()

    if move_forward():
        print("  ✓ U-turn escape successful, resetting state")
        best_global_avg_ref[0] = 0
        brightest_ppfd_ref[0] = 0
        move_history_ref[0] = []
        return True, 0   # カウントもリセット

    print("  ✗ Cannot move forward after U-turn")
    return False, uturn_retry_count


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

    shadow_escape_count = 0
    MAX_SHADOW_ESCAPES = 2

    # ミュータブル参照としてラップ（ヘルパー関数への受け渡し用）
    _best_global_avg = [best_global_avg]
    _brightest_ppfd = [brightest_ppfd]
    _move_history = [move_history]

    def reset_exploration_state():
        _best_global_avg[0] = 0
        _brightest_ppfd[0] = 0
        _move_history[0] = []
        nonlocal previous_reached_ppfd, no_improvement_count, uturn_retry_count
        previous_reached_ppfd = 0
        no_improvement_count = 0
        uturn_retry_count = 0

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

            update_global_brightness(lux)

            measurements.append({
                'index': i,
                'angle_step': angle_step,
                'lux': lux,
                'ppfd': ppfd,
                'distance': distance,
                'can_move': distance > SAFE_DISTANCE_MARGIN
            })

            print(f"Angle {angle_step * 45}°: Lux={lux:.2f}, "
                  f"PPFD={ppfd:.2f}, Dist={distance:.1f}cm")

            if i < len(ANGLE_STEPS) - 1:
                rotate_right()

        for _ in range(2):
            rotate_left()

        # ===== 影エリア判定 =====
        if is_in_shadow_area(measurements) and shadow_escape_count < MAX_SHADOW_ESCAPES:
            shadow_escape_count += 1
            print(f"→ Shadow escape attempt {shadow_escape_count}/{MAX_SHADOW_ESCAPES}")

            if escape_shadow_area():
                reset_exploration_state()
                shadow_escape_count = 0
                continue
            else:
                print("  Continuing with normal exploration despite shadow...")

        avg_lux = sum(m['lux'] for m in measurements) / len(measurements)

        if _best_global_avg[0] == 0:
            _best_global_avg[0] = avg_lux

        # ===== 暗化判定 =====
        if avg_lux < _best_global_avg[0] * DARK_RATIO:
            print(f"Darker detected "
                  f"(current:{avg_lux:.2f} < best:{_best_global_avg[0]:.2f}"
                  f"*{DARK_RATIO}) → U-turn")
            rotate_180()
            if move_forward():
                reset_exploration_state()
            continue

        if avg_lux > _best_global_avg[0]:
            _best_global_avg[0] = avg_lux

        # ===== 移動可能方向の選択 =====
        movable = [m for m in measurements if m['can_move']]

        if not movable:
            print("No movable direction in initial scan")
            escaped, uturn_retry_count = attempt_uturn_escape(
                uturn_retry_count, MAX_UTURN_RETRIES,
                _best_global_avg, _brightest_ppfd, _move_history
            )
            if escaped:
                previous_reached_ppfd = 0
                no_improvement_count = 0
                continue
            else:
                if uturn_retry_count >= MAX_UTURN_RETRIES:
                    return get_ppfd()
                continue

        uturn_retry_count = 0

        best = min(movable, key=lambda m: abs(m['ppfd'] - target_ppfd))

        print(f"Best direction: {best['angle_step'] * 45}° "
              f"(Lux={best['lux']:.2f}, PPFD={best['ppfd']:.2f}, "
              f"Dist={best['distance']:.1f}cm)")

        rotation_angle = best['angle_step']

        if rotation_angle > 0:
            for _ in range(rotation_angle):
                rotate_right()
        elif rotation_angle < 0:
            for _ in range(-rotation_angle):
                rotate_left()

        # ===== 角度補正 =====
        corrected_angle, was_corrected = correct_angle_if_needed(
            best['lux'], rotation_angle, measurements
        )
        if was_corrected:
            rotation_angle = corrected_angle

        # ===== 最終安全確認 =====
        print("\n--- Final safety check before moving ---")
        final_safe, final_distance = check_distance_safe(MIN_MOVE_DISTANCE)

        if not final_safe:
            print(f"⚠ Final safety check FAILED (distance={final_distance:.1f}cm)")
            print("→ Reverting rotation and re-scanning")

            # 回転を元に戻す
            if rotation_angle > 0:
                for _ in range(int(abs(rotation_angle))):
                    rotate_left()
            elif rotation_angle < 0:
                for _ in range(int(abs(rotation_angle))):
                    rotate_right()

            brightest_movable, rescan_measurements = rescan_for_brightest_movable()

            if brightest_movable is None:
                escaped, uturn_retry_count = attempt_uturn_escape(
                    uturn_retry_count, MAX_UTURN_RETRIES,
                    _best_global_avg, _brightest_ppfd, _move_history
                )
                if escaped:
                    previous_reached_ppfd = 0
                    no_improvement_count = 0
                    continue
                else:
                    if uturn_retry_count >= MAX_UTURN_RETRIES:
                        return get_ppfd()
                    continue

            rotation_angle = brightest_movable['angle_step']

            if rotation_angle > 0:
                for _ in range(rotation_angle):
                    rotate_right()
            elif rotation_angle < 0:
                for _ in range(-rotation_angle):
                    rotate_left()

            corrected_angle, was_corrected = correct_angle_if_needed(
                brightest_movable['lux'], rotation_angle, rescan_measurements
            )
            if was_corrected:
                rotation_angle = corrected_angle

            final_safe, final_distance = check_distance_safe(MIN_MOVE_DISTANCE)

            if not final_safe:
                print("⚠ Still unsafe after re-scan → last resort U-turn")

                if rotation_angle > 0:
                    for _ in range(int(abs(rotation_angle))):
                        rotate_left()
                elif rotation_angle < 0:
                    for _ in range(int(abs(rotation_angle))):
                        rotate_right()

                escaped, uturn_retry_count = attempt_uturn_escape(
                    uturn_retry_count, MAX_UTURN_RETRIES,
                    _best_global_avg, _brightest_ppfd, _move_history
                )
                if escaped:
                    previous_reached_ppfd = 0
                    no_improvement_count = 0
                    continue
                else:
                    if uturn_retry_count >= MAX_UTURN_RETRIES:
                        return get_ppfd()
                    continue

        print(f"✓ Safe to move forward (distance={final_distance:.1f}cm)")
        moved = move_forward()

        if not moved:
            print("⚠ Move forward failed despite safety check")

            if rotation_angle > 0:
                for _ in range(int(abs(rotation_angle))):
                    rotate_left()
            elif rotation_angle < 0:
                for _ in range(int(abs(rotation_angle))):
                    rotate_right()

            escaped, uturn_retry_count = attempt_uturn_escape(
                uturn_retry_count, MAX_UTURN_RETRIES,
                _best_global_avg, _brightest_ppfd, _move_history
            )
            if escaped:
                previous_reached_ppfd = 0
                no_improvement_count = 0
                continue
            else:
                if uturn_retry_count >= MAX_UTURN_RETRIES:
                    return get_ppfd()
                continue

        print(f"✓ Successfully moved forward")

        integrate_light()
        print_status_if_needed()

        current_lux = get_lux()
        update_global_brightness(current_lux)

        reached_ppfd = get_ppfd()
        print(f"Reached PPFD: {reached_ppfd:.2f}, Target: {target_ppfd:.2f}")

        if reached_ppfd > _brightest_ppfd[0]:
            _brightest_ppfd[0] = reached_ppfd
            _move_history[0] = []
            print(f"New brightest point: {_brightest_ppfd[0]:.2f}")
            shadow_escape_count = 0
        else:
            _move_history[0].append({
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
            print(f"No significant improvement: "
                  f"{reached_ppfd:.2f} <= {previous_reached_ppfd:.2f}*1.05 "
                  f"— count: {no_improvement_count}/{MAX_NO_IMPROVEMENT}")

            if no_improvement_count >= MAX_NO_IMPROVEMENT:
                print(f"Max brightness reached (no improvement for "
                      f"{MAX_NO_IMPROVEMENT} moves)")

                if len(_move_history[0]) > 0:
                    print(f"\n=== Returning to brightest point "
                          f"(PPFD={_brightest_ppfd[0]:.2f}) ===")
                    print(f"Move history: {len(_move_history[0])} steps")

                    rotate_180()
                    integrate_light()
                    print_status_if_needed()

                    for move in reversed(_move_history[0]):
                        move_forward()
                        integrate_light()
                        print_status_if_needed()

                        rev_angle = -move['rotation_angle']
                        if rev_angle > 0:
                            for _ in range(int(abs(rev_angle))):
                                rotate_right()
                        elif rev_angle < 0:
                            for _ in range(int(abs(rev_angle))):
                                rotate_left()

                    rotate_180()
                    integrate_light()
                    print_status_if_needed()

                    final_ppfd = get_ppfd()
                    print(f"=== Returned to brightest point: "
                          f"PPFD={final_ppfd:.2f} ===\n")
                    return final_ppfd
                else:
                    print("Already at brightest point")
                    break
        else:
            no_improvement_count = 0
            previous_reached_ppfd = reached_ppfd
            print(f"Improvement detected: "
                  f"{reached_ppfd:.2f} > {previous_reached_ppfd * 1.05:.2f}")

    best_global_avg = _best_global_avg[0]
    return get_ppfd()


# ===== 滞在 =====
def stay(target_ppfd):
    stay_start_ppfd = get_ppfd()

    print(f"=== Entering STAY phase ===")
    print(f"Staying at PPFD: {stay_start_ppfd:.2f} µmol/m²/s, "
          f"Target: {target_ppfd:.2f} µmol/m²/s")
    print(f"Current accumulated: {accumulated_umol:.2f} µmol/m²")

    while True:
        integrate_light()
        print_status_if_needed()

        elapsed = time.time() - start_time
        if elapsed >= TOTAL_SECONDS:
            return "finished"

        current_ppfd = get_ppfd()
        if current_ppfd < stay_start_ppfd * PPFD_DROP_RATIO:
            print(f"Light decreased "
                  f"({current_ppfd:.2f} < {stay_start_ppfd * PPFD_DROP_RATIO:.2f})"
                  f" → re-explore")
            return "reexplore"

        if current_ppfd > target_ppfd * (1 + BRIGHTNESS_TOLERANCE):
            print(f"Too bright "
                  f"({current_ppfd:.2f} > {target_ppfd * (1 + BRIGHTNESS_TOLERANCE):.2f})"
                  f" → adjust")
            rotate_left()
            if move_forward():
                return "adjust"

        time.sleep(1)


# ===== メイン =====
try:
    target_ppfd_constant = DLI_TARGET_UMOL / TOTAL_SECONDS

    print("=== Light accumulation control started ===")
    print(f"Target DLI:             {TARGET_DLI} mol/m²/d "
          f"({DLI_TARGET_UMOL} µmol/m²)")
    print(f"Target time:            {ACCUMULATION_HOURS}h ({TOTAL_SECONDS} seconds)")
    print(f"Target PPFD (constant): {target_ppfd_constant:.2f} µmol/m²/s")
    print(f"Lux to PPFD conversion: {LUX_TO_PPFD}")
    print(f"Angle correction threshold: {ANGLE_CORRECTION_THRESHOLD * 100:.0f}%")
    print(f"Safe distance margin:   {SAFE_DISTANCE_MARGIN}cm")
    print(f"Shadow escape threshold (ModeA): "
          f"{SHADOW_ESCAPE_RATIO * 100:.0f}% of global max")
    print(f"Shadow escape threshold (ModeB): "
          f"relative ratio < {RELATIVE_SHADOW_RATIO} or "
          f"uniform dark for {MAX_UNIFORM_DARK} scans")
    print()

    adjustment_ppfd = 0

    while True:
        integrate_light()

        elapsed = time.time() - start_time
        remaining_time = TOTAL_SECONDS - elapsed

        if remaining_time <= 0:
            print("=== 10h complete ===")
            print(f"Accumulated:    {accumulated_umol:.2f} µmol/m² "
                  f"({accumulated_umol / 1_000_000:.3f} mol/m²)")
            print(f"Target was:     {DLI_TARGET_UMOL} µmol/m² "
                  f"({TARGET_DLI} mol/m²/d)")
            print(f"Global max lux: {global_max_lux:.2f}")
            achievement_rate = (accumulated_umol / DLI_TARGET_UMOL) * 100
            print(f"Achievement rate: {achievement_rate:.1f}%")
            break

        remaining_umol = DLI_TARGET_UMOL - accumulated_umol
        required_ppfd = remaining_umol / max(1, remaining_time)
        required_ppfd += adjustment_ppfd

        print(f"\n--- New cycle ---")
        print(f"Elapsed:    {elapsed / 3600:.2f}h, "
              f"Remaining:  {remaining_time / 3600:.2f}h")
        print(f"Accumulated: {accumulated_umol:.2f} µmol/m² "
              f"({accumulated_umol / 1_000_000:.3f} mol/m²)")
        print(f"Remaining:   {remaining_umol:.2f} µmol/m²")
        print(f"Required PPFD (dynamic):  {required_ppfd:.2f} µmol/m²/s")
        print(f"Target PPFD (constant):   {target_ppfd_constant:.2f} µmol/m²/s")
        print(f"Global max lux:           {global_max_lux:.2f}")

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
            print(f"Final accumulated: {accumulated_umol:.2f} µmol/m² "
                  f"({accumulated_umol / 1_000_000:.3f} mol/m²)")
            print(f"Target was:        {DLI_TARGET_UMOL} µmol/m² "
                  f"({TARGET_DLI} mol/m²/d)")
            print(f"Global max lux:    {global_max_lux:.2f}")
            achievement_rate = (accumulated_umol / DLI_TARGET_UMOL) * 100
            print(f"Achievement rate:  {achievement_rate:.1f}%")
            break

except KeyboardInterrupt:
    stop()
    print("\n=== Stopped manually ===")
    print(f"Accumulated:  {accumulated_umol:.2f} µmol/m² "
          f"({accumulated_umol / 1_000_000:.3f} mol/m²)")
    print(f"Target was:   {DLI_TARGET_UMOL} µmol/m² ({TARGET_DLI} mol/m²/d)")
    print(f"Global max lux: {global_max_lux:.2f}")
    if DLI_TARGET_UMOL > 0:
        achievement_rate = (accumulated_umol / DLI_TARGET_UMOL) * 100
        print(f"Achievement rate: {achievement_rate:.1f}%")

finally:
    stop()
