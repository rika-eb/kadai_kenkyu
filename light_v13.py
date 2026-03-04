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

DLI_TARGET_UMOL = TARGET_DLI * 1_000_000  # µmol/m²
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

# ===== 30秒ごとの表示用 =====
PRINT_INTERVAL = 30

# ===== 状態 =====
accumulated_umol = 0  # µmol/m²単位で積算
start_time = time.time()
last_integral_time = time.time()
last_print_time = time.time()
best_global_avg = 0


# ===== 基本動作 =====
def stop():
    motor.setMotorModel(0, 0)
    time.sleep(0.1)


def rotate_left():
    motor.setMotorModel(-TURN_POWER, TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()
    time.sleep(0.1)


def rotate_right():
    motor.setMotorModel(TURN_POWER, -TURN_POWER)
    time.sleep(TURN_STEP_TIME)
    stop()
    time.sleep(0.1)


def rotate_180():
    for _ in range(4):
        rotate_right()
    stop()
    time.sleep(0.1)


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

    ppfd = get_ppfd()  # µmol/m²/s
    increment = ppfd * dt  # µmol/m²
    accumulated_umol += increment


# ===== 30秒ごとの表示 =====
def print_status_if_needed():
    """30秒ごとに現在の照度と積算値を表示"""
    global last_print_time, accumulated_umol
    
    now = time.time()
    if now - last_print_time >= PRINT_INTERVAL:
        current_lux = get_lux()
        current_ppfd = get_ppfd()
        print(f"[Status] Lux={current_lux:.2f}, PPFD={current_ppfd:.2f} µmol/m²/s, Accumulated={accumulated_umol:.2f} µmol/m²")
        last_print_time = now


# ===== 探索 =====
def explore(target_ppfd):
    global best_global_avg

    ANGLE_STEPS = [-2, -1, 0, 1, 2]
    
    previous_reached_ppfd = 0  # ★ 変更: 移動後に到達したPPFDを記録
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
                previous_reached_ppfd = 0  # ★ リセット
                no_improvement_count = 0
            continue
        
        if avg_lux > best_global_avg:
            best_global_avg = avg_lux
        
        movable = [m for m in measurements if m['can_move']]
        
        if not movable:
            print("No movable direction → stay here")
            return get_ppfd()
        
        best = min(movable, key=lambda m: abs(m['ppfd'] - target_ppfd))
        
        print(f"Best direction: {best['angle_step']*45}° (PPFD={best['ppfd']:.2f})")
        
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
        
        moved = move_forward()
        
        if not moved:
            print("Cannot move forward → stay here")
            return get_ppfd()
        
        integrate_light()
        print_status_if_needed()
        
        reached_ppfd = get_ppfd()
        
        print(f"Reached PPFD: {reached_ppfd:.2f}, Target: {target_ppfd:.2f}")
        
        if reached_ppfd > brightest_ppfd:
            brightest_ppfd = reached_ppfd
            move_history = []
            print(f"New brightest point: {brightest_ppfd:.2f}")
        else:
            if rotation_type:
                move_history.append({'type': rotation_type, 'count': rotation_count})
            move_history.append({'type': 'forward', 'count': 1})
        
        if reached_ppfd >= target_ppfd * (1 - BRIGHTNESS_TOLERANCE):
            print("Target brightness reached")
            break
        
        # ★★★ 変更: reached_ppfdを基準に判定 ★★★
        if previous_reached_ppfd == 0:
            # 初回は比較基準を設定
            previous_reached_ppfd = reached_ppfd
        elif reached_ppfd <= previous_reached_ppfd * 1.05:
            # 5%以上の改善がない場合
            no_improvement_count += 1
            print(f"No significant improvement in reached PPFD ({reached_ppfd:.2f} <= {previous_reached_ppfd:.2f}*1.05) - count: {no_improvement_count}/{MAX_NO_IMPROVEMENT}")
            
            if no_improvement_count >= MAX_NO_IMPROVEMENT:
                print(f"Max brightness reached in room (no improvement for {MAX_NO_IMPROVEMENT} moves)")
                
                if len(move_history) > 0:
                    print(f"\n=== Returning to brightest point (PPFD={brightest_ppfd:.2f}) ===")
                    print(f"Move history length: {len(move_history)} steps")
                    print("Return path:")
                    
                    # 戻る経路を表示
                    print("  Step 1: Rotate 180°")
                    rotate_180()
                    integrate_light()
                    print_status_if_needed()
                    
                    step_num = 2
                    for i, move in enumerate(reversed(move_history)):
                        if move['type'] == 'forward':
                            for _ in range(move['count']):
                                print(f"  Step {step_num}: Move forward")
                                move_forward()
                                integrate_light()
                                print_status_if_needed()
                                step_num += 1
                        elif move['type'] == 'rotate_right':
                            angle = move['count'] * 45
                            print(f"  Step {step_num}: Rotate right {angle}° (after 180° turn)")
                            for _ in range(move['count']):
                                rotate_right()
                            step_num += 1
                        elif move['type'] == 'rotate_left':
                            angle = move['count'] * 45
                            print(f"  Step {step_num}: Rotate left {angle}° (after 180° turn)")
                            for _ in range(move['count']):
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
            # 改善があった場合
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