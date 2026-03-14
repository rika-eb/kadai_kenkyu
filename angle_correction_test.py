"""
angle_correction_test.py

角度補正あり/なしの比較実験用プログラム。

フロー：
  1. スキャン（各方向でEnter待ち → シール貼れる）
  2. 補正あり/なしを選択
  3. ターゲット方向へ回転（Enter待ち）
  4. 前進（Enter待ち）
  5. 結果表示
"""

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

# ===== パラメータ =====
LUX_TO_PPFD = 0.0281
TURN_POWER = 3000
TURN_STEP_TIME = 0.28       # 45°相当の回転時間
FORWARD_POWER = 3000
FORWARD_TIME = 0.3
MIN_MOVE_DISTANCE = 20
SAFE_DISTANCE_MARGIN = 25
ANGLE_CORRECTION_THRESHOLD = 0.15

# スキャン範囲：-90°〜+90°（5方向）
ANGLE_STEPS = [-2, -1, 0, 1, 2]
SCAN_START_STEPS = 2    # 正面から左に2ステップ(-90°)へ
SCAN_END_STEPS = 2      # +90°から正面(0°)へ戻す


# ===== ユーティリティ =====
def wait_enter(msg=""):
    """Enterキーを押すまで待機"""
    if msg:
        print(f"\n  >>> {msg}")
    input("      [Enter を押して次へ進む]")


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
    fraction: -1.0〜+1.0。正なら右、負なら左。
    |fraction| < 0.1 は無視。
    """
    if abs(fraction) < 0.1:
        print(f"  部分回転スキップ (fraction={fraction:.3f} が小さすぎ)")
        return
    duration = TURN_STEP_TIME * abs(fraction)
    direction = "右" if fraction > 0 else "左"
    print(f"  部分回転: {direction}方向, {fraction:.2f}ステップ "
          f"({duration:.3f}s = {fraction * 45:.1f}°)")
    if fraction > 0:
        motor.setMotorModel(TURN_POWER, -TURN_POWER)
    else:
        motor.setMotorModel(-TURN_POWER, TURN_POWER)
    time.sleep(duration)
    stop()
    time.sleep(0.3)


def move_forward():
    dist = ultra.get_distance()
    if dist > MIN_MOVE_DISTANCE:
        motor.setMotorModel(FORWARD_POWER, FORWARD_POWER)
        time.sleep(FORWARD_TIME)
        stop()
        time.sleep(1.0)
        return True
    print(f"  前進不可（距離={dist:.1f}cm < {MIN_MOVE_DISTANCE}cm）")
    return False


def get_lux():
    return light.lux or 0


def get_dist():
    vals = [ultra.get_distance() for _ in range(3)]
    return sum(vals) / len(vals)


def estimate_angle_from_lux(current_lux, measurements):
    """
    現在のlux値からスキャン結果を線形補間して推定角度を返す。
    """
    pairs = sorted(
        [(m['angle_step'], m['lux']) for m in measurements],
        key=lambda x: x[0]
    )
    min_lux = min(p[1] for p in pairs)
    max_lux = max(p[1] for p in pairs)

    if current_lux < min_lux * 0.8 or current_lux > max_lux * 1.2:
        print(f"  現在lux {current_lux:.2f} がスキャン範囲外 "
              f"[{min_lux:.2f}, {max_lux:.2f}]")
        return None

    candidates = []
    for i in range(len(pairs) - 1):
        a1, l1 = pairs[i]
        a2, l2 = pairs[i + 1]
        if min(l1, l2) <= current_lux <= max(l1, l2):
            if abs(l2 - l1) < 1.0:
                est = (a1 + a2) / 2.0
            else:
                ratio = (current_lux - l1) / (l2 - l1)
                est = a1 + ratio * (a2 - a1)
            avg_lux = (l1 + l2) / 2.0
            candidates.append((est, avg_lux))
            print(f"  補間候補: {est:.2f}ステップ ({est*45:.1f}°), "
                  f"ペア平均lux={avg_lux:.2f}")

    if not candidates:
        print("  補間マッチなし")
        return None

    chosen = max(candidates, key=lambda c: c[1])
    return chosen[0]


# ===== STEP 1: スキャン =====
def do_scan():
    """
    各方向に回転しながらlux・距離を記録。
    各方向でEnterを押すことでシールを貼るタイミングを確保。
    """
    print("\n" + "=" * 60)
    print("STEP 1: スキャン")
    print("  各方向に回転します。止まったら床にシールを貼ってください。")
    print("=" * 60)

    measurements = []

    print(f"\n  スキャン開始位置（-90°）へ移動します...")
    wait_enter("スキャン開始位置（-90°）へ回転します")
    for _ in range(SCAN_START_STEPS):
        rotate_left()

    for i, angle_step in enumerate(ANGLE_STEPS):
        angle_deg = angle_step * 45
        lux = get_lux()
        dist = get_dist()
        ppfd = lux * LUX_TO_PPFD

        print(f"\n  ----------------------------------------")
        print(f"  【方向 {angle_deg:+d}°】")
        print(f"    Lux    = {lux:.2f}")
        print(f"    PPFD   = {ppfd:.4f} µmol/m²/s")
        print(f"    距離   = {dist:.1f}cm")
        print(f"    移動可否: {'OK' if dist > SAFE_DISTANCE_MARGIN else 'NG'}")

        measurements.append({
            'angle_step': angle_step,
            'lux': lux,
            'ppfd': ppfd,
            'distance': dist,
            'can_move': dist > SAFE_DISTANCE_MARGIN
        })

        if i < len(ANGLE_STEPS) - 1:
            wait_enter("シールを貼ったら次の方向（右+45°）へ回転します")
            rotate_right()
        else:
            wait_enter("シールを貼ったらスキャン完了（正面へ戻します）")

    # +90° → 正面
    for _ in range(SCAN_END_STEPS):
        rotate_left()

    print("\n  ✓ 正面（0°）に戻りました")
    return measurements


# ===== STEP 2: ターゲット選択 =====
def select_target(measurements):
    print("\n" + "=" * 60)
    print("STEP 2: ターゲット方向の選択")
    print("=" * 60)
    print("\n  スキャン結果一覧:")
    print(f"  {'角度':>6}  {'Lux':>8}  {'PPFD':>8}  {'距離':>8}  移動")
    print("  " + "-" * 50)
    for m in measurements:
        print(f"  {m['angle_step']*45:>+5}°  "
              f"{m['lux']:>8.2f}  "
              f"{m['ppfd']:>8.4f}  "
              f"{m['distance']:>7.1f}cm  "
              f"{'OK' if m['can_move'] else 'NG'}")

    movable = [m for m in measurements if m['can_move']]
    if not movable:
        print("\n  ⚠ 移動可能な方向がありません。終了します。")
        return None

    best = max(movable, key=lambda m: m['lux'])
    print(f"\n  推奨ターゲット: {best['angle_step']*45:+d}° "
          f"(最も明るい方向, Lux={best['lux']:.2f})")

    while True:
        try:
            choices = [m['angle_step']*45 for m in movable]
            raw = input(f"\n  ターゲット角度を入力 {choices} "
                        f"[Enter で推奨 {best['angle_step']*45:+d}°]: ")
            if raw.strip() == "":
                return best
            val = int(raw.strip())
            matched = [m for m in movable if m['angle_step'] * 45 == val]
            if matched:
                return matched[0]
            print(f"  ⚠ {val}° は有効な選択肢にありません。")
        except ValueError:
            print("  ⚠ 整数で入力してください。")


# ===== STEP 3: 補正モード選択 =====
def select_correction_mode():
    print("\n" + "=" * 60)
    print("STEP 3: 角度補正モードの選択")
    print("=" * 60)
    while True:
        raw = input(
            "\n  角度補正を行いますか？\n"
            "    y → あり（Lux偏差15%以上で自動補正）\n"
            "    n → なし（指定ステップ数のみ回転）\n"
            "  選択 [y/n]: "
        ).strip().lower()
        if raw in ("y", "yes", ""):
            print("  → 補正あり")
            return True
        elif raw in ("n", "no"):
            print("  → 補正なし")
            return False
        print("  ⚠ y または n を入力してください。")


# ===== STEP 4: 回転 =====
def do_rotate(target, correction_enabled, measurements):
    print("\n" + "=" * 60)
    print("STEP 4: 回転")
    print("=" * 60)

    rotation_steps = target['angle_step']
    target_lux = target['lux']

    print(f"\n  ターゲット: {rotation_steps*45:+d}°")
    print(f"  期待lux:   {target_lux:.2f}")

    wait_enter(f"{rotation_steps*45:+d}° 方向へ回転します")

    if rotation_steps > 0:
        for _ in range(rotation_steps):
            rotate_right()
    elif rotation_steps < 0:
        for _ in range(-rotation_steps):
            rotate_left()

    # ★ 回転後、lux測定の前にEnter待ち
    # → ここで手動でロボットを数度ずらすと補正が発動しやすくなる
    print(f"\n  ✓ 回転完了（指定角度: {rotation_steps*45:+d}°）")
    print(f"  期待lux: {target_lux:.2f}")
    print(f"\n  ★ 必要であれば今ロボットを手で数度ずらしてください")
    wait_enter("ずらし終わったらlux測定を行います")

    # Enter後に測定（ずらした状態が反映される）
    lux_after_rotate = get_lux()
    dist_after_rotate = get_dist()
    deviation = abs(lux_after_rotate - target_lux) / max(target_lux, 1.0)

    print(f"\n  ----------------------------------------")
    print(f"  【回転後の測定値】")
    print(f"    期待lux:   {target_lux:.2f}")
    print(f"    実測lux:   {lux_after_rotate:.2f}")
    print(f"    lux偏差:   {deviation*100:.1f}%  "
          f"{'→ 補正が発動します' if deviation >= ANGLE_CORRECTION_THRESHOLD else '→ 補正不要'}")
    print(f"    距離:      {dist_after_rotate:.1f}cm")

    result = {
        'rotation_steps': rotation_steps,
        'target_lux': target_lux,
        'lux_before_correction': lux_after_rotate,
        'dist_before_correction': dist_after_rotate,
        'deviation_before': deviation,
        'correction_applied': False,
        'correction_steps': 0,
        'correction_fraction': 0.0,
        'lux_after_correction': lux_after_rotate,
        'dist_after_correction': dist_after_rotate,
        'deviation_after': deviation,
    }

    # ===== 補正なし =====
    if not correction_enabled:
        print("\n  補正なしモード: このまま進みます")
        wait_enter("現在位置を記録・シールを貼ったら前進へ進みます")
        return result, rotation_steps

    # ===== 補正あり =====
    print(f"\n  ----------------------------------------")
    print(f"  【角度補正チェック】(閾値={ANGLE_CORRECTION_THRESHOLD*100:.0f}%)")

    if deviation < ANGLE_CORRECTION_THRESHOLD:
        print(f"  → 偏差 {deviation*100:.1f}% < 閾値 "
              f"{ANGLE_CORRECTION_THRESHOLD*100:.0f}% → 補正不要")
        wait_enter("現在位置を記録・シールを貼ったら前進へ進みます")
        return result, rotation_steps

    print(f"  → 偏差 {deviation*100:.1f}% ≥ 閾値 "
          f"{ANGLE_CORRECTION_THRESHOLD*100:.0f}% → 補正実行")

    estimated = estimate_angle_from_lux(lux_after_rotate, measurements)
    if estimated is None:
        print("  → 推定不可。補正スキップ。")
        wait_enter("現在位置を記録・シールを貼ったら前進へ進みます")
        return result, rotation_steps

    raw_correction = estimated - rotation_steps
    int_correction = int(raw_correction)
    frac_correction = raw_correction - int_correction

    print(f"\n  推定現在角度: {estimated:.2f}ステップ ({estimated*45:.1f}°)")
    print(f"  目標角度:     {rotation_steps}ステップ ({rotation_steps*45}°)")
    print(f"  補正量合計:   {raw_correction:.3f}ステップ ({raw_correction*45:.1f}°)")
    print(f"    整数部:     {int_correction}ステップ ({int_correction*45}°)")
    print(f"    端数部:     {frac_correction:.3f}ステップ "
          f"({frac_correction*45:.1f}°)")

    if int_correction == 0 and abs(frac_correction) < 0.1:
        print("  → 補正量が小さすぎるためスキップ")
        wait_enter("現在位置を記録・シールを貼ったら前進へ進みます")
        return result, rotation_steps

    wait_enter(f"補正回転を行います（{raw_correction*45:.1f}°）")

    if int_correction > 0:
        for _ in range(int_correction):
            rotate_right()
    elif int_correction < 0:
        for _ in range(-int_correction):
            rotate_left()

    if abs(frac_correction) >= 0.1:
        rotate_partial(frac_correction)

    # 補正後の測定
    lux_after_corr = get_lux()
    dist_after_corr = get_dist()
    deviation_after = abs(lux_after_corr - target_lux) / max(target_lux, 1.0)

    print(f"\n  ----------------------------------------")
    print(f"  【補正後】")
    print(f"    期待lux:   {target_lux:.2f}")
    print(f"    実測lux:   {lux_after_corr:.2f}")
    print(f"    lux偏差:   {deviation_after*100:.1f}%")
    print(f"    距離:      {dist_after_corr:.1f}cm")

    result.update({
        'correction_applied': True,
        'correction_steps': int_correction,
        'correction_fraction': frac_correction,
        'lux_after_correction': lux_after_corr,
        'dist_after_correction': dist_after_corr,
        'deviation_after': deviation_after,
    })

    corrected_rotation = rotation_steps + raw_correction
    wait_enter("補正完了。現在位置を記録・シールを貼ったら前進へ進みます")
    return result, corrected_rotation


# ===== STEP 5: 前進 =====
def do_forward():
    print("\n" + "=" * 60)
    print("STEP 5: 前進")
    print("=" * 60)

    lux_before = get_lux()
    dist_before = get_dist()
    print(f"\n  前進前: lux={lux_before:.2f}, 距離={dist_before:.1f}cm")

    wait_enter("前進します")
    moved = move_forward()

    lux_after = get_lux()
    dist_after = get_dist()
    print(f"\n  前進後: lux={lux_after:.2f}, 距離={dist_after:.1f}cm")
    if not moved:
        print("  ⚠ 前進できませんでした")

    wait_enter("到達地点にシールを貼って記録してください")
    return {
        'moved': moved,
        'lux_before': lux_before,
        'lux_after': lux_after,
        'dist_before': dist_before,
        'dist_after': dist_after,
    }


# ===== 結果サマリ =====
def print_summary(target, correction_enabled, rotate_result, forward_result):
    print("\n" + "=" * 60)
    print("結果サマリ")
    print("=" * 60)

    print(f"\n  補正モード:  {'あり' if correction_enabled else 'なし'}")
    print(f"  ターゲット:  {target['angle_step']*45:+d}°  "
          f"(期待lux={target['lux']:.2f})")

    print(f"\n  ---- 回転精度 ----")
    print(f"  回転直後の偏差:  {rotate_result['deviation_before']*100:.1f}%"
          f"  (期待={rotate_result['target_lux']:.2f}, "
          f"実測={rotate_result['lux_before_correction']:.2f})")

    if correction_enabled:
        if rotate_result['correction_applied']:
            print(f"  補正後の偏差:    {rotate_result['deviation_after']*100:.1f}%"
                  f"  (実測={rotate_result['lux_after_correction']:.2f})")
            print(f"  補正量:          "
                  f"{rotate_result['correction_steps']}ステップ"
                  f" + {rotate_result['correction_fraction']:.2f}ステップ端数"
                  f" = {(rotate_result['correction_steps']+rotate_result['correction_fraction'])*45:.1f}°")
            improvement = (rotate_result['deviation_before']
                           - rotate_result['deviation_after'])
            print(f"  偏差改善:        {improvement*100:.1f}ポイント")
        else:
            print(f"  補正:            不要（偏差が閾値未満）")
    else:
        print(f"  補正:            スキップ（補正なしモード）")

    print(f"\n  ---- 前進結果 ----")
    print(f"  前進:      {'成功' if forward_result['moved'] else '失敗'}")
    print(f"  前進前lux: {forward_result['lux_before']:.2f}")
    print(f"  前進後lux: {forward_result['lux_after']:.2f}")

    print("\n" + "=" * 60)
    print("  ★ 実際の到達位置はシールの場所と定規で計測してください")
    print("=" * 60)


# ===== メイン =====
def main():
    print("\n" + "=" * 60)
    print("  角度補正あり/なし 比較実験")
    print("=" * 60)
    print("\n  ロボットを出発点に置き、シールを貼る準備をしてください。")
    wait_enter("準備ができたらスキャンを開始します")

    try:
        measurements = do_scan()
        target = select_target(measurements)
        if target is None:
            return

        correction_enabled = select_correction_mode()
        rotate_result, _ = do_rotate(target, correction_enabled, measurements)
        forward_result = do_forward()
        print_summary(target, correction_enabled, rotate_result, forward_result)

    except KeyboardInterrupt:
        print("\n\n  中断されました")
    finally:
        stop()
        print("  モーター停止。終了します。")


if __name__ == "__main__":
    main()