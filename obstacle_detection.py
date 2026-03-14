"""
obstacle_detection_test.py

障害物判定あり/なし の比較実験用プログラム。

起動時に「障害物判定を有効にするか」だけを選択し、
あとは explore → stay を自動で走らせる。

障害物判定が使われている箇所（全6箇所）をフラグでゲート：
  1. scan_direction()      : can_move の判定
  2. move_forward()        : 前進直前チェック
  3. escape_shadow_area()  : 影脱出前進前チェック
  4. correct_angle_if_needed() : 角度補正中の各ステップチェック
  5. explore() 最終安全確認   : 前進前の check_distance_safe()
  6. estimate_angle_from_lux() : 補間候補の障害物フィルタ

判定なしモードでは距離の測定・表示は行うが、
判定結果を無視して常にSAFE・can_move=Trueとして扱う。
"""

import time
import board
import adafruit_tsl2591
from motor import tankMotor
from ultrasonic import Ultrasonic

import sys
import os
from datetime import datetime

# ===== ログ設定 =====
LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)

# ===== センサ =====
i2c = board.I2C()
light = adafruit_tsl2591.TSL2591(i2c)
motor = tankMotor()
ultra = Ultrasonic()

# ===== DLI設定 =====
TARGET_DLI = 0.1224
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
TURN_STEP_TIME = 0.28
FORWARD_POWER = 3000
FORWARD_TIME = 0.3
MIN_MOVE_DISTANCE = 20
SAFE_DISTANCE_MARGIN = 25

# ===== 影エリア判定パラメータ =====
STARTUP_DARK_THRESHOLD = 8
RELATIVE_SHADOW_RATIO = 0.4
MAX_UNIFORM_DARK = 3
ESCAPE_FORWARD_ATTEMPTS = 2
SHADOW_HISTORY_WINDOW = 10
SHADOW_MEDIAN_MIN_SAMPLES = 5

# ===== 探索角度設定 =====
ANGLE_STEPS = [-3, -2, -1, 0, 1, 2]
SCAN_START_STEPS = 2
SCAN_END_STEPS = 2

# ===== 表示用 =====
PRINT_INTERVAL = 30

# ===== 状態変数 =====
accumulated_umol = 0
start_time = time.time()
last_integral_time = time.time()
last_print_time = time.time()
best_global_avg = 0

global_max_lux = 0
global_max_history = []
uniform_dark_count = 0
scan_count = 0

# ===== 障害物判定モード（起動時に設定） =====
OBSTACLE_DETECTION_ENABLED = True  # 起動時に上書きされる


# ===== 起動時モード選択 =====
def select_obstacle_mode():
    global OBSTACLE_DETECTION_ENABLED

    print("\n" + "=" * 60)
    print("  障害物判定あり/なし 比較実験")
    print("=" * 60)
    print("\n  障害物判定を有効にしますか？")
    print("    y → あり（距離センサで障害物を検出したら回避・停止）")
    print("    n → なし（距離を計測・表示はするが判定を無視して常に前進）")

    while True:
        raw = input("\n  選択 [y/n]: ").strip().lower()
        if raw in ("y", "yes", ""):
            OBSTACLE_DETECTION_ENABLED = True
            print("  → 障害物判定: あり")
            return
        elif raw in ("n", "no"):
            OBSTACLE_DETECTION_ENABLED = False
            print("  → 障害物判定: なし")
            print("  ⚠ 注意: 障害物に衝突する可能性があります。")
            print("         実験中は目を離さず、危険な場合はCtrl+Cで停止してください。")
            confirm = input("  続行しますか？ [y/n]: ").strip().lower()
            if confirm in ("y", "yes"):
                return
            else:
                print("  キャンセルしました。再選択してください。")
        else:
            print("  ⚠ y または n を入力してください。")


# ===== ログ初期化 =====
def init_log():
    mode_str = "obstacle_ON" if OBSTACLE_DETECTION_ENABLED else "obstacle_OFF"
    log_filename = os.path.join(
        LOG_DIR,
        f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{mode_str}.txt"
    )

    class Tee:
        def __init__(self, *files):
            self.files = files
        def write(self, obj):
            for f in self.files:
                f.write(obj)
                f.flush()
        def flush(self):
            for f in self.files:
                f.flush()

    log_file = open(log_filename, 'w', encoding='utf-8')
    sys.stdout = Tee(sys.__stdout__, log_file)
    print(f"=== Log started: {log_filename} ===")
    print(f"=== Obstacle detection: "
          f"{'ENABLED' if OBSTACLE_DETECTION_ENABLED else 'DISABLED'} ===")
    return log_file


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
    """
    障害物判定あり : 前進前に距離チェック。近ければ前進しない。
    障害物判定なし : 距離を表示するだけで必ず前進する。
    """
    dist = ultra.get_distance()

    if OBSTACLE_DETECTION_ENABLED:
        if dist > MIN_MOVE_DISTANCE:
            motor.setMotorModel(FORWARD_POWER, FORWARD_POWER)
            time.sleep(FORWARD_TIME)
            stop()
            time.sleep(1.0)
            return True
        print(f"  [Obstacle] Blocked (dist={dist:.1f}cm "
              f"< {MIN_MOVE_DISTANCE}cm) → skip forward")
        return False
    else:
        # 距離は表示するが判定を無視して前進
        print(f"  [Obstacle DISABLED] dist={dist:.1f}cm "
              f"(ignored) → moving forward")
        motor.setMotorModel(FORWARD_POWER, FORWARD_POWER)
        time.sleep(FORWARD_TIME)
        stop()
        time.sleep(1.0)
        return True


def get_lux():
    return light.lux or 0


def get_ppfd():
    return get_lux() * LUX_TO_PPFD


def check_distance_safe(min_distance=MIN_MOVE_DISTANCE):
    """
    障害物判定あり : 距離を測定して SAFE/BLOCKED を返す。
    障害物判定なし : 距離を測定・表示するが常に (True, dist) を返す。
    """
    distances = []
    for _ in range(3):
        distances.append(ultra.get_distance())
        time.sleep(0.1)
    avg_distance = sum(distances) / len(distances)

    if OBSTACLE_DETECTION_ENABLED:
        is_safe = avg_distance > min_distance
        print(f"  Distance check: {avg_distance:.1f}cm (min: {min_distance}cm) "
              f"→ {'SAFE' if is_safe else 'BLOCKED'}")
        return is_safe, avg_distance
    else:
        # 距離は表示するが常にSAFEとして返す
        print(f"  Distance check: {avg_distance:.1f}cm "
              f"[Obstacle DISABLED → always SAFE]")
        return True, avg_distance


# ===== 常時積算 =====
def integrate_light():
    global accumulated_umol, last_integral_time
    now = time.time()
    dt = now - last_integral_time
    last_integral_time = now
    accumulated_umol += get_ppfd() * dt


# ===== 30秒ごとの表示 =====
def print_status_if_needed():
    global last_print_time
    now = time.time()
    if now - last_print_time >= PRINT_INTERVAL:
        print(f"[Status] Lux={get_lux():.2f}, PPFD={get_ppfd():.2f} µmol/m²/s, "
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


# ===== 影判定基準lux（中央値ベース） =====
def get_reference_lux():
    if len(global_max_history) >= SHADOW_MEDIAN_MIN_SAMPLES:
        recent = global_max_history[-SHADOW_HISTORY_WINDOW:]
        sorted_vals = sorted(recent)
        n = len(sorted_vals)
        median = sorted_vals[n // 2] if n % 2 == 1 \
            else (sorted_vals[n // 2 - 1] + sorted_vals[n // 2]) / 2.0
        print(f"  Reference lux: median={median:.2f} "
              f"(from {n} samples, global_max={global_max_lux:.2f})")
        return median
    else:
        print(f"  Reference lux: global_max={global_max_lux:.2f} "
              f"(insufficient history: "
              f"{len(global_max_history)}/{SHADOW_MEDIAN_MIN_SAMPLES})")
        return global_max_lux


# ===== 影エリア判定 =====
def is_in_shadow_area(measurements):
    global uniform_dark_count, scan_count
    scan_count += 1
    avg_lux = sum(m['lux'] for m in measurements) / len(measurements)
    max_current = max(m['lux'] for m in measurements)

    if global_max_lux >= STARTUP_DARK_THRESHOLD:
        reference_lux = get_reference_lux()
        shadow_threshold = reference_lux * SHADOW_ESCAPE_RATIO
        is_shadow = avg_lux < shadow_threshold
        if is_shadow:
            print(f"\n⚠️ [ModeA] SHADOW AREA DETECTED")
            print(f"  avg={avg_lux:.2f}, ref={reference_lux:.2f}, "
                  f"threshold={shadow_threshold:.2f}")
        return is_shadow

    if max_current < 1.0:
        print(f"\n⚠️ [ModeB-Critical] All near zero → forcing escape")
        return True

    relative_ratio = avg_lux / max_current
    if relative_ratio < RELATIVE_SHADOW_RATIO:
        uniform_dark_count = 0
        print(f"\n⚠️ [ModeB] STARTUP SHADOW (ratio={relative_ratio:.2f})")
        return True

    if avg_lux < STARTUP_DARK_THRESHOLD:
        uniform_dark_count += 1
        print(f"  [ModeB] Uniformly dark: {uniform_dark_count}/{MAX_UNIFORM_DARK}")
        if uniform_dark_count >= MAX_UNIFORM_DARK:
            uniform_dark_count = 0
            return True
        return False

    uniform_dark_count = 0
    return False


# ===== 影脱出行動 =====
def escape_shadow_area():
    print("\n=== SHADOW ESCAPE PROCEDURE ===")
    rotate_180()
    integrate_light()

    for attempt in range(ESCAPE_FORWARD_ATTEMPTS):
        print(f"  Forward attempt {attempt + 1}/{ESCAPE_FORWARD_ATTEMPTS}")

        # ここも check_distance_safe を経由するのでフラグが効く
        is_safe, dist = check_distance_safe(MIN_MOVE_DISTANCE)
        if not is_safe:
            print(f"    ✗ Blocked ({dist:.1f}cm), rotating right")
            rotate_right()
            integrate_light()
            is_safe, dist = check_distance_safe(MIN_MOVE_DISTANCE)
            if not is_safe:
                print(f"    ✗ Alternative also blocked, skipping")
                continue

        if move_forward():
            integrate_light()
            current_lux = get_lux()
            update_global_brightness(current_lux)
            reference_lux = get_reference_lux()
            if reference_lux > 0 and \
                    current_lux > reference_lux * SHADOW_ESCAPE_RATIO:
                print(f"    ✓ Escape successful")
                print("=== Shadow escape SUCCESSFUL ===\n")
                return True
        else:
            print(f"    ✗ Cannot move forward")

    print("=== Shadow escape FAILED ===\n")
    return False


# ===== 角度スキャン =====
def scan_direction():
    """
    障害物判定あり : can_move = distance > SAFE_DISTANCE_MARGIN
    障害物判定なし : can_move = True（距離は表示するが常に移動可）
    """
    measurements = []

    for _ in range(SCAN_START_STEPS):
        rotate_left()

    for i, angle_step in enumerate(ANGLE_STEPS):
        integrate_light()
        print_status_if_needed()

        lux = get_lux()
        ppfd = lux * LUX_TO_PPFD
        distance = ultra.get_distance()

        update_global_brightness(lux)

        if OBSTACLE_DETECTION_ENABLED:
            can_move = distance > SAFE_DISTANCE_MARGIN
        else:
            # 距離は記録するが移動可否は常にTrue
            can_move = True

        measurements.append({
            'index': i,
            'angle_step': angle_step,
            'lux': lux,
            'ppfd': ppfd,
            'distance': distance,
            'can_move': can_move
        })

        move_str = 'OK' if can_move else 'NG'
        disabled_note = '' if OBSTACLE_DETECTION_ENABLED \
            else ' [obstacle ignored]'
        print(f"  Scan {angle_step * 45:+4d}°: Lux={lux:.2f}, "
              f"Dist={distance:.1f}cm, Move={move_str}{disabled_note}")

        if i < len(ANGLE_STEPS) - 1:
            rotate_right()

    for _ in range(SCAN_END_STEPS):
        rotate_left()

    return measurements


# ===== 角度推定 =====
def estimate_angle_from_lux(current_lux, measurements):
    """
    障害物判定あり : SAFE_DISTANCE_MARGIN 未満の候補を除外
    障害物判定なし : 距離チェックをスキップして全候補を対象にする
    """
    angle_lux_pairs = sorted(
        [(m['angle_step'], m['lux'], m['distance']) for m in measurements],
        key=lambda x: x[0]
    )
    min_lux = min(p[1] for p in angle_lux_pairs)
    max_lux = max(p[1] for p in angle_lux_pairs)

    if current_lux < min_lux * 0.8 or current_lux > max_lux * 1.2:
        print(f"  Lux {current_lux:.2f} out of range [{min_lux:.2f}, {max_lux:.2f}]")
        return None

    candidates = []
    for i in range(len(angle_lux_pairs) - 1):
        angle1, lux1, dist1 = angle_lux_pairs[i]
        angle2, lux2, dist2 = angle_lux_pairs[i + 1]
        if min(lux1, lux2) <= current_lux <= max(lux1, lux2):
            if abs(lux2 - lux1) < 1.0:
                estimated_angle = (angle1 + angle2) / 2.0
            else:
                ratio = (current_lux - lux1) / (lux2 - lux1)
                estimated_angle = angle1 + ratio * (angle2 - angle1)
            candidates.append({
                'angle': estimated_angle,
                'min_dist': min(dist1, dist2),
                'avg_lux': (lux1 + lux2) / 2.0,
            })

    if not candidates:
        closest = min(angle_lux_pairs, key=lambda x: abs(x[1] - current_lux))
        if OBSTACLE_DETECTION_ENABLED and closest[2] < SAFE_DISTANCE_MARGIN:
            return None
        return closest[0]

    if OBSTACLE_DETECTION_ENABLED:
        safe = [c for c in candidates if c['min_dist'] >= SAFE_DISTANCE_MARGIN]
        if not safe:
            return None
        return max(safe, key=lambda c: c['avg_lux'])['angle']
    else:
        # 障害物チェックなし：全候補から最明を選択
        return max(candidates, key=lambda c: c['avg_lux'])['angle']


# ===== 角度補正 =====
def correct_angle_if_needed(expected_lux, expected_angle_step, measurements):
    """
    check_distance_safe() を経由するので、
    OBSTACLE_DETECTION_ENABLED=False のとき常にSAFEが返り補正が続行される。
    """
    current_lux = get_lux()
    is_safe, current_distance = check_distance_safe(MIN_MOVE_DISTANCE)

    print(f"\n--- Angle correction check ---")
    print(f"  Expected: Lux={expected_lux:.2f}, Angle={expected_angle_step * 45}°")
    print(f"  Current:  Lux={current_lux:.2f}, Distance={current_distance:.1f}cm")

    if not is_safe:
        print(f"  ⚠ Unsafe, skipping correction")
        return expected_angle_step, False

    deviation = abs(current_lux - expected_lux) / max(expected_lux, 1.0)
    print(f"  Deviation: {deviation * 100:.1f}%")

    if deviation < ANGLE_CORRECTION_THRESHOLD:
        print(f"  → No correction needed")
        return expected_angle_step, False

    estimated_angle = estimate_angle_from_lux(current_lux, measurements)
    if estimated_angle is None:
        print(f"  → Cannot estimate, skipping")
        return expected_angle_step, False

    raw_correction = estimated_angle - expected_angle_step
    int_correction = int(raw_correction)
    frac_correction = raw_correction - int_correction

    print(f"  Correction: {raw_correction:.3f} steps ({raw_correction * 45:.1f}°)")

    if int_correction == 0 and abs(frac_correction) < 0.1:
        print(f"  → Too small, skipping")
        return expected_angle_step, False

    if int_correction > 0:
        for i in range(int_correction):
            rotate_right()
            temp_safe, _ = check_distance_safe(SAFE_DISTANCE_MARGIN)
            if not temp_safe:
                for _ in range(i + 1):
                    rotate_left()
                return expected_angle_step, False
    elif int_correction < 0:
        for i in range(-int_correction):
            rotate_left()
            temp_safe, _ = check_distance_safe(SAFE_DISTANCE_MARGIN)
            if not temp_safe:
                for _ in range(i + 1):
                    rotate_right()
                return expected_angle_step, False

    if abs(frac_correction) >= 0.1:
        temp_safe, _ = check_distance_safe(SAFE_DISTANCE_MARGIN)
        if temp_safe:
            rotate_partial(frac_correction)

    corrected_safe, corrected_distance = check_distance_safe(MIN_MOVE_DISTANCE)
    if not corrected_safe:
        if int_correction > 0:
            for _ in range(int_correction):
                rotate_left()
        elif int_correction < 0:
            for _ in range(-int_correction):
                rotate_right()
        if abs(frac_correction) >= 0.1:
            rotate_partial(-frac_correction)
        return expected_angle_step, False

    return expected_angle_step + int_correction + frac_correction, True


# ===== 再スキャン =====
def rescan_for_brightest_movable():
    print("\n=== Re-scanning for brightest movable direction ===")
    measurements = scan_direction()
    # can_move はすでにフラグを考慮した値が入っている
    movable = [m for m in measurements if m['can_move']]
    if not movable:
        return None, measurements
    brightest = max(movable, key=lambda m: m['ppfd'])
    print(f"  Brightest: {brightest['angle_step'] * 45}° "
          f"(Lux={brightest['lux']:.2f})")
    return brightest, measurements


# ===== Uターン脱出 =====
def attempt_uturn_escape(uturn_retry_count, max_retries,
                         best_global_avg_ref, brightest_ppfd_ref,
                         move_history_ref):
    if uturn_retry_count >= max_retries:
        print(f"→ Already tried U-turn {max_retries} times, giving up")
        return False, uturn_retry_count

    uturn_retry_count += 1
    print(f"→ U-turn escape attempt {uturn_retry_count}/{max_retries}")
    rotate_180()
    integrate_light()

    if move_forward():
        best_global_avg_ref[0] = 0
        brightest_ppfd_ref[0] = 0
        move_history_ref[0] = []
        return True, 0

    return False, uturn_retry_count


# ===== 探索 =====
def explore(target_ppfd):
    global best_global_avg

    previous_reached_ppfd = 0
    no_improvement_count = 0
    MAX_NO_IMPROVEMENT = 3
    brightest_ppfd = 0
    move_history = []
    uturn_retry_count = 0
    MAX_UTURN_RETRIES = 2
    shadow_escape_count = 0
    MAX_SHADOW_ESCAPES = 2

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

        print("\n--- scan ---")
        measurements = scan_direction()

        # 影エリア判定（障害物フラグとは独立して常に有効）
        if is_in_shadow_area(measurements) and \
                shadow_escape_count < MAX_SHADOW_ESCAPES:
            shadow_escape_count += 1
            print(f"→ Shadow escape attempt "
                  f"{shadow_escape_count}/{MAX_SHADOW_ESCAPES}")
            if escape_shadow_area():
                reset_exploration_state()
                shadow_escape_count = 0
                continue
            else:
                print("  Continuing despite shadow...")

        avg_lux = sum(m['lux'] for m in measurements) / len(measurements)

        if _best_global_avg[0] == 0:
            _best_global_avg[0] = avg_lux

        # 暗化判定
        if avg_lux < _best_global_avg[0] * DARK_RATIO:
            print(f"Darker detected → U-turn")
            rotate_180()
            if move_forward():
                reset_exploration_state()
            continue

        if avg_lux > _best_global_avg[0]:
            _best_global_avg[0] = avg_lux

        # 移動可能方向の選択（can_moveはフラグ考慮済み）
        movable = [m for m in measurements if m['can_move']]

        if not movable:
            # 障害物判定なしのとき全方向can_move=Trueなのでここには来ない
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
        rotation_angle = best['angle_step']

        print(f"Best direction: {rotation_angle * 45}° "
              f"(Lux={best['lux']:.2f}, PPFD={best['ppfd']:.2f}, "
              f"Dist={best['distance']:.1f}cm)")

        if rotation_angle > 0:
            for _ in range(rotation_angle):
                rotate_right()
        elif rotation_angle < 0:
            for _ in range(-rotation_angle):
                rotate_left()

        corrected_angle, was_corrected = correct_angle_if_needed(
            best['lux'], rotation_angle, measurements
        )
        if was_corrected:
            rotation_angle = corrected_angle

        # ===== 最終安全確認（check_distance_safe経由でフラグが効く） =====
        print("\n--- Final safety check ---")
        final_safe, final_distance = check_distance_safe(MIN_MOVE_DISTANCE)

        if not final_safe:
            # 障害物判定ありのときのみここに入る
            print(f"⚠ Blocked (dist={final_distance:.1f}cm) → reverting & re-scan")

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

            post_rotate_lux = get_lux()
            post_rotate_ppfd = post_rotate_lux * LUX_TO_PPFD
            if post_rotate_ppfd >= target_ppfd * (1 - BRIGHTNESS_TOLERANCE):
                if post_rotate_ppfd > _brightest_ppfd[0]:
                    _brightest_ppfd[0] = post_rotate_ppfd
                    _move_history[0] = []
                update_global_brightness(post_rotate_lux)
                break

            corrected_angle, was_corrected = correct_angle_if_needed(
                brightest_movable['lux'], rotation_angle, rescan_measurements
            )
            if was_corrected:
                rotation_angle = corrected_angle

            final_safe, final_distance = check_distance_safe(MIN_MOVE_DISTANCE)
            if not final_safe:
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

        moved = move_forward()

        if not moved:
            # 障害物判定なしのとき move_forward() は常にTrueなのでここには来ない
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

        integrate_light()
        print_status_if_needed()

        current_lux = get_lux()
        update_global_brightness(current_lux)
        reached_ppfd = get_ppfd()
        print(f"Reached PPFD: {reached_ppfd:.2f}, Target: {target_ppfd:.2f}")

        if reached_ppfd > _brightest_ppfd[0]:
            _brightest_ppfd[0] = reached_ppfd
            _move_history[0] = []
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
            print(f"No improvement: {no_improvement_count}/{MAX_NO_IMPROVEMENT}")
            if no_improvement_count >= MAX_NO_IMPROVEMENT:
                if len(_move_history[0]) > 0:
                    print(f"\n=== Returning to brightest point "
                          f"(PPFD={_brightest_ppfd[0]:.2f}) ===")
                    rotate_180()
                    integrate_light()
                    for move in reversed(_move_history[0]):
                        move_forward()
                        integrate_light()
                        rev_angle = -move['rotation_angle']
                        if rev_angle > 0:
                            for _ in range(int(abs(rev_angle))):
                                rotate_right()
                        elif rev_angle < 0:
                            for _ in range(int(abs(rev_angle))):
                                rotate_left()
                    rotate_180()
                    integrate_light()
                    return get_ppfd()
                else:
                    break
        else:
            no_improvement_count = 0
            previous_reached_ppfd = reached_ppfd

    best_global_avg = _best_global_avg[0]
    return get_ppfd()


# ===== 滞在 =====
def stay(target_ppfd):
    stay_start_ppfd = get_ppfd()
    print(f"=== Entering STAY phase ===")
    print(f"PPFD: {stay_start_ppfd:.2f}, Target: {target_ppfd:.2f}")
    print(f"Obstacle detection: "
          f"{'ENABLED' if OBSTACLE_DETECTION_ENABLED else 'DISABLED'}")

    while True:
        integrate_light()
        print_status_if_needed()

        if time.time() - start_time >= TOTAL_SECONDS:
            return "finished"

        current_ppfd = get_ppfd()
        if current_ppfd < stay_start_ppfd * PPFD_DROP_RATIO:
            print(f"Light decreased → re-explore")
            return "reexplore"

        if current_ppfd > target_ppfd * (1 + BRIGHTNESS_TOLERANCE):
            print(f"Too bright → adjust")
            rotate_left()
            if move_forward():
                return "adjust"

        time.sleep(1)


# ===== メイン =====
select_obstacle_mode()
log_file = init_log()

try:
    target_ppfd_constant = DLI_TARGET_UMOL / TOTAL_SECONDS

    print("=== Light accumulation control started ===")
    print(f"Obstacle detection:      "
          f"{'ENABLED' if OBSTACLE_DETECTION_ENABLED else 'DISABLED'}")
    print(f"Target DLI:              {TARGET_DLI} mol/m²/d")
    print(f"Target PPFD (constant):  {target_ppfd_constant:.2f} µmol/m²/s")
    print(f"Accumulation time:       {ACCUMULATION_HOURS}h")
    print()

    adjustment_ppfd = 0

    while True:
        integrate_light()
        elapsed = time.time() - start_time
        remaining_time = TOTAL_SECONDS - elapsed

        if remaining_time <= 0:
            print("=== 10h complete ===")
            print(f"Accumulated:      {accumulated_umol:.2f} µmol/m²")
            achievement_rate = (accumulated_umol / DLI_TARGET_UMOL) * 100
            print(f"Achievement rate: {achievement_rate:.1f}%")
            break

        remaining_umol = DLI_TARGET_UMOL - accumulated_umol
        required_ppfd = remaining_umol / max(1, remaining_time) + adjustment_ppfd

        print(f"\n--- New cycle ---")
        print(f"Elapsed: {elapsed/3600:.2f}h, Remaining: {remaining_time/3600:.2f}h")
        print(f"Accumulated: {accumulated_umol:.2f} µmol/m²")
        print(f"Required PPFD: {required_ppfd:.2f} µmol/m²/s")

        reached_ppfd = explore(required_ppfd)
        integrate_light()

        if reached_ppfd < required_ppfd * (1 - BRIGHTNESS_TOLERANCE):
            shortage = required_ppfd - reached_ppfd
            adjustment_ppfd += shortage
            print(f"Shortage: +{adjustment_ppfd:.2f} µmol/m²/s carry-over")
        else:
            adjustment_ppfd = 0

        result = stay(required_ppfd)
        if result == "finished":
            print("=== Target reached ===")
            print(f"Final accumulated: {accumulated_umol:.2f} µmol/m²")
            print(f"Achievement rate:  "
                  f"{accumulated_umol / DLI_TARGET_UMOL * 100:.1f}%")
            break

except KeyboardInterrupt:
    stop()
    print("\n=== Stopped manually ===")
    print(f"Accumulated: {accumulated_umol:.2f} µmol/m²")
    if DLI_TARGET_UMOL > 0:
        print(f"Achievement rate: "
              f"{accumulated_umol / DLI_TARGET_UMOL * 100:.1f}%")

finally:
    stop()
    log_file.close()
    sys.stdout = sys.__stdout__