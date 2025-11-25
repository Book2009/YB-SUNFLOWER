import sys
sys.path.append("/home/Book_Pi/DFRobot_RaspberryPi_Expansion_Board")
from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_IIC as Board
from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_Servo as Servo

import serial
import time
from collections import deque

# Import RPi.GPIO FIRST before any other GPIO-related imports
import RPi.GPIO as GPIO

import board as board_pins
import busio
import adafruit_bno055
COMPASS_AVAILABLE = True

# ============================================================================
# GPIO Initialization
# ============================================================================
print("Initializing GPIO...")
try:
    GPIO.cleanup()
except:
    pass

time.sleep(0.2)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor pins
MOTOR_IN1 = 6
MOTOR_IN2 = 7
MOTOR_ENA = 5

# Setup motor pins
print("Setting up motor pins...")
try:
    GPIO.setup([MOTOR_IN1, MOTOR_IN2, MOTOR_ENA], GPIO.OUT, initial=GPIO.LOW)
    print("GPIO pins configured successfully")
except Exception as e:
    print(f"GPIO setup warning: {e}")

# ============================================================================
# Configuration Constants
# ============================================================================

SERVO_CHANNEL = 0
SERVO_CENTER = 92
MAX_TURN_ANGLE = 45

LIDAR_PORT = '/dev/ttyAMA0'
LIDAR_BAUD = 460800

LIGHT_SENSOR_A3 = 3
LINE_THRESHOLD = 2600

FRONT_MIN = 355
FRONT_MAX = 5
LEFT_MIN = 265
LEFT_MAX = 275
RIGHT_MIN = 85
RIGHT_MAX = 95

LIDAR_SIDE_CLEARANCE_THRESHOLD = 2000

HEADING_TOLERANCE = 10
HEADING_KP = 0.5

BASE_SPEED = 55
SLOW_SPEED = 50
TURN_SPEED = 50

TARGET_WALL_WIDE = 450
TARGET_WALL_NARROW = 250
MIN_SAFE_DISTANCE = 180
KP_WALL = 0.04
KD_WALL = 0.025

FRONT_TURN_THRESHOLD = 700
PRE_TURN_STEER_ANGLE = 30
REVERSE_SPEED = 50

board = Board(1, 0x10)

# ============================================================================
# Helper Functions
# ============================================================================

def normalize_angle(angle):
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle

def angle_difference(target, current):
    diff = target - current
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return diff

def correct_lidar_angle(lidar_angle, heading_offset):
    corrected = lidar_angle - heading_offset
    return normalize_angle(corrected)

# ============================================================================
# Simple Motor Class (No Encoder)
# ============================================================================

class SimpleMotor:
    def __init__(self, in1, in2, ena):
        self.in1, self.in2, self.ena = in1, in2, ena
        
        print("Configuring motor pins...")
        
        # Motor control pins
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.ena, GPIO.OUT)
        
        # PWM
        self.pwm = GPIO.PWM(self.ena, 1000)
        self.pwm.start(0)
        print("Motor configured successfully")
    
    def set_speed(self, speed):
        speed = max(-100, min(100, speed))
        try:
            if speed > 0:
                GPIO.output(self.in1, GPIO.HIGH)
                GPIO.output(self.in2, GPIO.LOW)
                self.pwm.ChangeDutyCycle(abs(speed))
            elif speed < 0:
                GPIO.output(self.in1, GPIO.LOW)
                GPIO.output(self.in2, GPIO.HIGH)
                self.pwm.ChangeDutyCycle(abs(speed))
            else:
                self.stop()
        except Exception as e:
            print(f"Motor speed error: {e}")
    
    def stop(self):
        try:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)
            self.pwm.ChangeDutyCycle(0)
        except:
            pass
    
    def cleanup(self):
        self.stop()
        try:
            self.pwm.stop()
        except:
            pass

class Vehicle:
    def __init__(self):
        print("\n" + "="*70)
        print(" WRO FUTURE ENGINEERS 2025 - LIDAR TURN DECISION")
        print("="*70)
        
        self._init_board()
        self._init_motor()
        self._init_servo()
        self._init_compass()
        self._init_lidar()
        
        # Compass navigation
        self.initial_heading = None
        self.target_heading = None
        self.heading_offset = 0
        
        # Mission state
        self.turn_count = 0
        self.turn_direction = None  # Will be set by first LiDAR decision
        
        # Navigation state
        self.last_left_error = 0
        self.last_right_error = 0
        self.current_speed = BASE_SPEED
        self.navigation_mode = "wall_follow"
        
        # Turn sequence state
        self.waiting_for_clearance = False
        self.line_detected_time = 0
        
        # Corridor width
        self.corridor_width_history = deque(maxlen=20)
        
        # Steering
        self.steering_history = deque(maxlen=3)
        
        # LiDAR
        self.lidar_buffer = bytearray()
        self.front_data = deque(maxlen=10)
        self.left_data = deque(maxlen=10)
        self.right_data = deque(maxlen=10)
        
        print("\n" + "="*70)
        print(" ALL SYSTEMS READY")
        print("="*70 + "\n")
    
    def _init_board(self):
        print("[1/5] Expansion Board...")
        self.board = Board(1, 0x10)
        while self.board.begin() != self.board.STA_OK:
            time.sleep(1)
        self.board.set_adc_enable()
        print("  OK")
    
    def _init_motor(self):
        print("[2/5] Simple Motor...")
        self.motor = SimpleMotor(MOTOR_IN1, MOTOR_IN2, MOTOR_ENA)
        print("  OK")
    
    def _init_servo(self):
        print("[3/5] Servo...")
        self.servo_ctrl = Servo(self.board)
        self.servo_ctrl.begin()
        self.servo_ctrl.move(SERVO_CHANNEL, SERVO_CENTER)
        print("  OK")
    
    def _init_compass(self):
        print("[4/5] BNO055 Compass...")
        try:
            i2c = busio.I2C(board_pins.SCL, board_pins.SDA)
            self.compass = adafruit_bno055.BNO055_I2C(i2c)
            time.sleep(1)
            print("  OK")
        except:
            print("  FAILED (continuing without compass)")
            self.compass = None
    
    def _init_lidar(self):
        print("[5/5] LiDAR...")
        self.lidar = serial.Serial(port=LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=1)
        
        print("  Sending scan command...")
        self.lidar.write(bytes([0xA5, 0x20]))
        time.sleep(0.5)
        response = self.lidar.read(7)
        print(f"  Scan response: {response.hex()}")
        print("  OK")
    
    # ========================================================================
    # COMPASS
    # ========================================================================
    
    def heading(self):
        if self.compass:
            try:
                euler = self.compass.euler
                if euler and euler[0] is not None:
                    return round(euler[0], 1)
            except:
                pass
        return None
    
    def set_initial_heading(self):
        heading = self.heading()
        if heading is not None:
            self.initial_heading = heading
            self.target_heading = heading
            print(f"Initial heading set: {self.initial_heading}Â°")
        else:
            print("WARNING: Could not set initial heading!")
    
    def update_heading_offset(self):
        current = self.heading()
        if current is not None and self.target_heading is not None:
            self.heading_offset = angle_difference(self.target_heading, current)
        else:
            self.heading_offset = 0
    
    def calculate_heading_correction(self):
        if self.heading_offset == 0:
            return 0
        correction = -self.heading_offset * HEADING_KP
        return max(-30, min(30, correction))
    
    # ========================================================================
    # LIDAR
    # ========================================================================
    
    def update_lidar(self):
        try:
            if self.lidar.in_waiting > 0:
                self.lidar_buffer.extend(self.lidar.read(self.lidar.in_waiting))
            
            self.update_heading_offset()
            
            while len(self.lidar_buffer) >= 5:
                point_data = self.lidar_buffer[:5]
                self.lidar_buffer = self.lidar_buffer[5:]
                
                point = self._parse_scan_point(point_data)
                if point and point['distance'] > 0:
                    raw_angle = point['angle']
                    corrected_angle = correct_lidar_angle(raw_angle, self.heading_offset)
                    dist = point['distance']
                    
                    if self._is_front_angle(corrected_angle):
                        self.front_data.append(dist)
                    elif self._is_left(corrected_angle):
                        self.left_data.append(dist)
                    elif self._is_right(corrected_angle):
                        self.right_data.append(dist)
        except Exception as e:
            print(f"LiDAR update error: {e}")
    
    def _parse_scan_point(self, data):
        if len(data) < 5:
            return None
        
        quality = (data[0] >> 2) & 0x3F
        angle_q6 = (data[1] >> 1) | (data[2] << 7)
        angle = (angle_q6 / 64.0) % 360.0
        distance_q2 = data[3] | (data[4] << 8)
        distance = distance_q2 / 4.0
        
        return {
            'angle': angle,
            'distance': distance,
            'quality': quality
        }
    
    def _is_front_angle(self, angle):
        return angle >= FRONT_MIN or angle <= FRONT_MAX
    
    def _is_left(self, angle):
        return abs(angle - LEFT_MIN) <= 5
    
    def _is_right(self, angle):
        return abs(angle - RIGHT_MIN) <= 5
    
    def distance_front(self):
        if self.front_data:
            return round(sum(self.front_data) / len(self.front_data))
        return None
    
    def distance_left(self):
        if self.left_data:
            return round(sum(self.left_data) / len(self.left_data))
        return None
    
    def distance_right(self):
        if self.right_data:
            return round(sum(self.right_data) / len(self.right_data))
        return None
    
    # ========================================================================
    # LIGHT SENSOR - ONLY A3
    # ========================================================================
    
    def read_light_sensor_a3(self):
        """Read only A3 light sensor"""
        try:
            value = self.board.get_adc_value(LIGHT_SENSOR_A3)
            return value
        except:
            return None
    
    def detect_line(self):
        """
        Detect line using ONLY A3 sensor
        Returns: True if line detected, False otherwise
        """
        a3_value = self.read_light_sensor_a3()
        
        if a3_value is None:
            return False
        
        # Line detected when sensor reads below threshold (dark line)
        if a3_value < LINE_THRESHOLD:
            print(f">>> LINE DETECTED! A3 sensor: {a3_value}")
            return True
        
        return False
    
    # ========================================================================
    # NEW TURNING STRATEGY
    # ========================================================================
    
    def decide_turn_direction_lidar(self):
        """
        NEW: Decide turn direction based on which side has > 1500mm clearance
        Returns: 'left' if left side is clear, 'right' if right side is clear
        """
        left_dist = self.distance_left()
        right_dist = self.distance_right()
        
        print(f"\n  LiDAR Turn Decision:")
        print(f"    Left side: {left_dist}mm")
        print(f"    Right side: {right_dist}mm")
        print(f"    Threshold: {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm")
        
        if left_dist is None:
            left_dist = 0
        if right_dist is None:
            right_dist = 0
        
        # Check which side has > 1500mm clearance
        left_clear = left_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD
        right_clear = right_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD
        
        if left_clear and not right_clear:
            print(f"  â†’ LEFT side is CLEAR ({left_dist}mm > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm)")
            return 'left'
        elif right_clear and not left_clear:
            print(f"  â†’ RIGHT side is CLEAR ({right_dist}mm > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm)")
            return 'right'
        elif left_clear and right_clear:
            # Both clear, pick the clearer one
            if left_dist > right_dist:
                print(f"  â†’ BOTH clear, LEFT is clearer ({left_dist}mm > {right_dist}mm)")
                return 'left'
            else:
                print(f"  â†’ BOTH clear, RIGHT is clearer ({right_dist}mm >= {left_dist}mm)")
                return 'right'
        else:
            # Neither clear, pick the larger one
            if left_dist > right_dist:
                print(f"  â†’ NEITHER > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm, choosing LEFT ({left_dist}mm > {right_dist}mm)")
                return 'left'
            else:
                print(f"  â†’ NEITHER > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm, choosing RIGHT ({right_dist}mm >= {left_dist}mm)")
                return 'right'
    
    # ========================================================================
    # MOTOR & STEERING
    # ========================================================================
    
    def forward(self, speed=BASE_SPEED):
        self.motor.set_speed(speed)
        self.current_speed = speed
    
    def stop(self):
        self.motor.stop()
        self.current_speed = 0
    
    def steer(self, angle):
        angle = max(-30, min(30, angle))
        servo_pos = SERVO_CENTER - angle
        self.servo_ctrl.move(SERVO_CHANNEL, int(servo_pos))
    
    def steer_smooth(self, target_angle):
        target_angle = max(-30, min(30, target_angle))
        self.steering_history.append(target_angle)
        
        if len(self.steering_history) > 0:
            smooth_angle = sum(self.steering_history) / len(self.steering_history)
            self.steer(smooth_angle)
    
    def steer_center(self):
        self.servo_ctrl.move(SERVO_CHANNEL, SERVO_CENTER)
        self.steering_history.clear()
    
    # ========================================================================
    # NAVIGATION
    # ========================================================================
    
    def estimate_corridor_width(self):
        left = self.distance_left()
        right = self.distance_right()
        
        if left and right:
            total_width = left + right
            self.corridor_width_history.append(total_width)
            
            if len(self.corridor_width_history) >= 10:
                avg_width = sum(self.corridor_width_history) / len(self.corridor_width_history)
                return "wide" if avg_width > 750 else "narrow"
        
        return "unknown"
    
    def calculate_steering(self):
        """Improved wall following + Compass heading correction"""
        left_dist = self.distance_left()
        right_dist = self.distance_right()
        front_dist = self.distance_front()
        
        corridor_type = self.estimate_corridor_width()
        
        # Adjust target based on corridor width
        if corridor_type == "narrow":
            target_distance = TARGET_WALL_NARROW
        else:
            target_distance = TARGET_WALL_WIDE
        
        steering = 0
        heading_correction = self.calculate_heading_correction()
        
        # PRIORITY 1: Front obstacle avoidance
        if front_dist and front_dist < FRONT_TURN_THRESHOLD:
            # Front obstacle detected - prepare to turn
            if left_dist and right_dist:
                # Steer away from closer wall
                if right_dist < left_dist:
                    steering = 25  # Steer left away from right wall
                    print(f"  Front obstacle! Steering LEFT (R:{right_dist} < L:{left_dist})")
                else:
                    steering = -25  # Steer right away from left wall
                    print(f"  Front obstacle! Steering RIGHT (L:{left_dist} < R:{right_dist})")
            self.forward(SLOW_SPEED)
            return max(-30, min(30, steering))
        
        # PRIORITY 2: Right wall following (preferred)
        if right_dist and right_dist < 1000:  # Right wall detected within 1000mm
            error = target_distance - right_dist
            derivative = error - self.last_right_error
            self.last_right_error = error
            
            # Calculate steering to maintain target distance
            steering = error * KP_WALL + derivative * KD_WALL
            
            # Speed adjustment based on proximity
            if right_dist < MIN_SAFE_DISTANCE:
                self.forward(SLOW_SPEED)
                steering *= 1.5  # More aggressive steering when too close
            elif right_dist < MIN_SAFE_DISTANCE * 1.5:
                self.forward(int(BASE_SPEED * 0.75))
            else:
                self.forward(BASE_SPEED)
        
        # PRIORITY 3: Left wall following (if no right wall)
        elif left_dist and left_dist < 1000:  # Left wall detected within 1000mm
            error = target_distance - left_dist
            derivative = error - self.last_left_error
            self.last_left_error = error
            
            # Calculate steering (negative because we want to steer right when too close to left)
            steering = -(error * KP_WALL + derivative * KD_WALL)
            
            # Speed adjustment based on proximity
            if left_dist < MIN_SAFE_DISTANCE:
                self.forward(SLOW_SPEED)
                steering *= 1.5  # More aggressive steering when too close
            elif left_dist < MIN_SAFE_DISTANCE * 1.5:
                self.forward(int(BASE_SPEED * 0.75))
            else:
                self.forward(BASE_SPEED)
        
        # PRIORITY 4: No walls detected - go straight
        else:
            steering = 0
            self.forward(BASE_SPEED)
            self.last_left_error = 0
            self.last_right_error = 0
        
        # Add compass correction (smaller weight so it doesn't override wall avoidance)
        steering += heading_correction * 0.5
        
        return max(-30, min(30, steering))
    
    # ========================================================================
    # 90-DEGREE TURN
    # ========================================================================
    
    def execute_90_degree_turn(self, direction):
        """Execute a compass-guided 90-degree turn"""
        print(f"\n  Executing 90° turn {direction.upper()}")
        self.navigation_mode = "turning"
        
        pre_turn_target = self.target_heading
        
        # Safety check: if no compass heading, use dead reckoning
        if pre_turn_target is None:
            print("  WARNING: No compass heading! Using dead reckoning...")
            self.steer_center()
            self.forward(SLOW_SPEED)
            time.sleep(0.3)  # Move forward
            
            # Execute turn by time
            self.forward(TURN_SPEED)
            if direction == 'right':
                self.steer(-PRE_TURN_STEER_ANGLE)
            else:
                self.steer(PRE_TURN_STEER_ANGLE)
            
            time.sleep(0.8)  # Turn for fixed time
            
            self.steer_center()
            self.forward(BASE_SPEED)
            time.sleep(0.3)
            self.navigation_mode = "wall_follow"
            return
        
        if direction == 'right':
            new_target = normalize_angle(pre_turn_target + 90)
        else:
            new_target = normalize_angle(pre_turn_target - 90)
        
        print(f"  Current target: {pre_turn_target:.0f}° → New target: {new_target:.0f}°")
        
        # Rest of the existing code...
        # Move forward to clear obstacle
        print("  Moving forward...")
        self.steer_center()
        self.forward(SLOW_SPEED)
        time.sleep(0.1)
        
        # Execute turn
        print(f"  Turning to {new_target:.0f}°...")
        self.forward(TURN_SPEED)
        
        if direction == 'right':
            self.steer(-PRE_TURN_STEER_ANGLE)
        else:
            self.steer(PRE_TURN_STEER_ANGLE)
        
        turn_start_time = time.time()
        max_turn_time = 0.8
        
        while time.time() - turn_start_time < max_turn_time:
            self.update_lidar()
            current = self.heading()
            
            if current is not None:
                error = abs(angle_difference(new_target, current))
                
                if error < HEADING_TOLERANCE:
                    print(f"  Turn complete! Heading: {current:.0f}° (error: {error:.1f}°)")
                    break
                
                if error < 15:
                    self.forward(SLOW_SPEED)
                    if direction == 'right':
                        self.steer(-20)
                    else:
                        self.steer(20)
            
            time.sleep(0.02)
        
        self.target_heading = new_target
        
        self.steer_center()
        self.forward(BASE_SPEED)
        time.sleep(0.5)
        
        self.navigation_mode = "wall_follow"
        print(f"  Now facing {self.target_heading:.0f}° - continuing")
    
    # ========================================================================
    # MAIN MISSION
    # ========================================================================
    
    def run_mission(self):
        print("\n" + "="*70)
        print(" WRO MISSION: NEW TURNING STRATEGY")
        print(" 1. Detect line with A3 sensor")
        print(" 2. Move forward until side > 1500mm")
        print(" 3. Turn to that side and remember direction")
        print(" 4. Repeat for all subsequent lines")
        print("="*70 + "\n")
        
        print("Calibrating compass...")
        for _ in range(20):
            self.update_lidar()
            time.sleep(0.05)
        self.set_initial_heading()
        
        try:
            print("Starting mission...")
            self.forward(BASE_SPEED)
            last_print = time.time()
            
            while True:
                self.update_lidar()
                
                # ========================================================
                # CHECK STOP CONDITIONS (after 12 turns completed)
                # ========================================================
                if self.turn_count >= 12 and not self.waiting_for_clearance:
                    current_heading = self.heading()
                    front_dist = self.distance_front()
                    
                    if current_heading is not None and front_dist is not None:
                        heading_error = abs(angle_difference(self.target_heading, current_heading))
                        
                        if heading_error <= HEADING_TOLERANCE and front_dist < 1800:
                            print(f"\n{'='*70}")
                            print(" âœ“ STOP CONDITIONS MET!")
                            print(f" Heading error: {heading_error:.1f}Â° (â‰¤10Â°) âœ“")
                            print(f" Front distance: {front_dist}mm (<1800mm) âœ“")
                            print(f"{'='*70}")
                            break
                        
                # ========================================================
                # PHASE 1: Detect line with A3 sensor
                # ========================================================
                # PHASE 1: Detect line with A3 sensor
                # ========================================================
                # PHASE 1: Detect line with A3 sensor
                # ========================================================
                if not self.waiting_for_clearance and self.turn_count < 12:
                    # Only check for line if enough time has passed since last detection
                    current_time = time.time()
                    if not hasattr(self, 'last_line_detect_time_ignore'):
                        self.last_line_detect_time_ignore = 0
                    
                    # Ignore lines for X seconds after detecting one
                    LINE_IGNORE_TIME = 3.0  # Adjust this value (in seconds)
                    
                    if current_time - self.last_line_detect_time_ignore > LINE_IGNORE_TIME:
                        if self.detect_line():
                            self.last_line_detect_time_ignore = current_time  # Record detection time
                            self.turn_count += 1
                            lap = (self.turn_count - 1) // 4 + 1
                            section = (self.turn_count - 1) % 4 + 1
                            
                            print(f"\n{'='*70}")
                            print(f" TURN #{self.turn_count} - LINE DETECTED")
                            print(f" LAP: {lap}/3 | SECTION: {section}/4")
                            print(f"{'='*70}")
                            
                            # FIRST TURN: Decide direction using LiDAR
                            if self.turn_direction is None:
                                print(" FIRST TURN - Need to decide direction")
                                print(" Moving forward to check side clearance...")
                                self.waiting_for_clearance = True
                                self.line_detected_time = time.time()
                            else:
                                # Subsequent turns: Use memorized direction
                                print(f" Using memorized direction: {self.turn_direction.upper()}")
                                self.waiting_for_clearance = True
                                self.line_detected_time = time.time()
                
                # ========================================================
                # PHASE 2: Wait for side clearance (> 1500mm)
                # ========================================================
                if self.waiting_for_clearance:
                    # Continue moving forward
                    steering = self.calculate_steering()
                    self.steer_smooth(steering)
                    
                    # For first turn: continuously check which side becomes clear
                    if self.turn_direction is None:
                        left_dist = self.distance_left()
                        right_dist = self.distance_right()
                        
                        left_clear = left_dist and left_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD
                        right_clear = right_dist and right_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD
                        
                        if left_clear or right_clear:
                            # One side is clear - make decision
                            self.turn_direction = self.decide_turn_direction_lidar()
                            print(f"\n>>> TURN DIRECTION SET: {self.turn_direction.upper()} <<<")
                            print(f">>> This direction will be used for all 12 turns <<<\n")
                            
                            # Execute turn
                            self.execute_90_degree_turn(self.turn_direction)
                            self.waiting_for_clearance = False
                        else:
                            # Keep checking
                            if time.time() - last_print >= 0.5:
                                print(f"Waiting... L:{left_dist}mm R:{right_dist}mm (need > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm)")
                                last_print = time.time()
                    else:
                        # Subsequent turns: wait for memorized side to be clear
                        if self.turn_direction == 'left':
                            check_dist = self.distance_left()
                        else:
                            check_dist = self.distance_right()
                        
                        if check_dist and check_dist > LIDAR_SIDE_CLEARANCE_THRESHOLD:
                            print(f"\n>>> {self.turn_direction.upper()} side CLEAR ({check_dist}mm) - TURNING <<<\n")
                            self.execute_90_degree_turn(self.turn_direction)
                            self.waiting_for_clearance = False
                        else:
                            if time.time() - last_print >= 0.5:
                                print(f"Waiting for {self.turn_direction} side... {check_dist}mm (need > {LIDAR_SIDE_CLEARANCE_THRESHOLD}mm)")
                                last_print = time.time()
                    
                    # Safety timeout
                    if time.time() - self.line_detected_time > 5.0:
                        print("TIMEOUT - forcing turn")
                        if self.turn_direction is None:
                            self.turn_direction = self.decide_turn_direction_lidar()
                        self.execute_90_degree_turn(self.turn_direction)
                        self.waiting_for_clearance = False
                
                # Normal navigation when not in turn sequence
                if self.navigation_mode == "wall_follow" and not self.waiting_for_clearance:
                    steering = self.calculate_steering()
                    self.steer_smooth(steering)
                
                # Status updates
                if time.time() - last_print >= 0.5 and not self.waiting_for_clearance:
                    last_print = time.time()
                    
                    left = self.distance_left() or 0
                    right = self.distance_right() or 0
                    front = self.distance_front() or 0
                    corridor = self.estimate_corridor_width()
                    current_heading = self.heading() or 0
                    
                    if self.turn_count >= 12:
                        # Show stop condition status after 12 turns
                        heading_error = abs(angle_difference(self.target_heading, current_heading))
                        print(f"[12/12 COMPLETE] Checking stop: H_err={heading_error:.1f}Â° | Front={front}mm | "
                              f"L:{left:4.0f} R:{right:4.0f}")
                    else:
                        # Handle None values in formatting
                        heading_offset_str = f"{self.heading_offset:+.1f}" if self.heading_offset is not None else "N/A"
                        target_heading_str = f"{self.target_heading:.0f}" if self.target_heading is not None else "N/A"

                        print(f"[{self.turn_count}/12] {corridor.upper()} | "
                              f"H:{current_heading:.0f}° → {target_heading_str}° (Δ{heading_offset_str}°) | "
                              f"L:{left:4.0f} R:{right:4.0f} F:{front:4.0f}mm | "
                              f"Dir:{self.turn_direction or 'TBD'}")
                
                time.sleep(0.02)
    
            # After breaking from loop (stop conditions met)
            self.stop()
            self.steer_center()
            
        except KeyboardInterrupt:
            print("\nInterrupted")
        finally:
            self.cleanup()
    
    def cleanup(self):
        print("\nShutting down...")
        try:
            self.stop()
            self.steer_center()
            time.sleep(0.1)
            self.lidar.write(bytes([0xA5, 0x25]))
            time.sleep(0.1)
            self.lidar.close()
        except:
            pass
        try:
            self.motor.cleanup()
        except:
            pass
        try:
            GPIO.cleanup()
        except:
            pass
        print("Done\n")

if __name__ == "__main__":
    car = Vehicle()
    
    print("\nSYSTEM TEST:")
    print("-" * 70)
    for i in range(100):
        car.update_lidar()
        time.sleep(0.02)
    
    car.set_initial_heading()
    print(f"Compass: {car.heading()}")
    
    # Run the main mission
    try:
        car.run_mission()
    except KeyboardInterrupt:
        print("Interrupted by user.")
        car.cleanup()
