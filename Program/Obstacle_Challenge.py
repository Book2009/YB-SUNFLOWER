import sys
sys.path.append("/home/Book_Pi/DFRobot_RaspberryPi_Expansion_Board")
from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_IIC as Board
from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_Servo as Servo

import serial
import time
from collections import deque
import RPi.GPIO as GPIO

try:
    import board as board_pins
    import busio
    import adafruit_bno055
    COMPASS_AVAILABLE = True
except ImportError as e:
    COMPASS_AVAILABLE = False
    board_pins = None
    busio = None
    adafruit_bno055 = None

import math
import atexit
import cv2 as cv
from picamera2 import Picamera2
import numpy as np
import statistics

MOTOR_IN1 = 6
MOTOR_IN2 = 7
MOTOR_ENA = 5

SERVO_CHANNEL = 0
SERVO_CENTER = 94
MAX_TURN_ANGLE = 60

LIDAR_PORT = '/dev/ttyAMA0'
LIDAR_BAUD = 460800

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CROP_TOP = 220
CROP_BOTTOM = 380

SHOW_CAMERA = False

RED_RANGES = [(np.array([173, 144, 61]), np.array([179, 255, 165]))]
GREEN_RANGES = [(np.array([53, 90, 79]), np.array([68, 198, 150]))]

ULTRA = 0
LIGHT_SENSOR_RED = 2
LIGHT_SENSOR_WHITE = 3
RED_THRESHOLD = 2000
WHITE_THRESHOLD = 3200

FRONT_MIN = 355
FRONT_MAX = 5
LEFT_ZONE_MIN = 250
LEFT_ZONE_MAX = 290
RIGHT_ZONE_MIN = 70
RIGHT_ZONE_MAX = 110

LIDAR_MIN_QUALITY = 5
LIDAR_MIN_DISTANCE = 30
LIDAR_MAX_DISTANCE = 4000
LIDAR_OUTLIER_THRESHOLD = 0.3

HEADING_TOLERANCE = 10
HEADING_KP = 0.5

BASE_SPEED = 50
SLOW_SPEED = 40
TURN_SPEED = 40
PARKING_SPEED = 50

TARGET_WALL_WIDE = 450
TARGET_WALL_NARROW = 180
MIN_SAFE_DISTANCE = 150
KP_WALL = 0.015
KD_WALL = 0.025

INNER_WALL_TARGET = 140
OUTER_WALL_TARGET = 140
KP_PARKING = 0.06
KD_PARKING = 0.06

OBSTACLE_MIN_AREA = 275
OBSTACLE_Y_MIN = 35
OBSTACLE_Y_MAX = 250

RED_TARGET = 100
GREEN_TARGET = 500

OBSTACLE_KP = 0.08
OBSTACLE_KD = 0.012

EMERGENCY_AREA_THRESHOLD = 18000
EMERGENCY_Y_THRESHOLD = 240

IGNORE_SIDE_OBSTACLES_TIME = 0.8
OBSTACLE_X_MIN = 50 
OBSTACLE_X_MAX = 590
LINE_IGNORE_TIME = 2.0
FIRST_LINE_SAMPLE_TIME = 0.1

OBSTACLE_PASSED_Y_THRESHOLD = 35
OBSTACLE_PASSED_TIME = 0.8

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))

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

class SimpleMotor:
    def __init__(self, in1, in2, ena):
        self.in1, self.in2, self.ena = in1, in2, ena
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.ena, GPIO.OUT)
        self.pwm = GPIO.PWM(self.ena, 1000)
        self.pwm.start(0)
    
    def set_speed(self, speed):
        speed = max(-100, min(100, speed))
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
    
    def stop(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
    
    def cleanup(self):
        self.stop()
        self.pwm.stop()

def create_multi_range_mask(hsv: np.ndarray, color_ranges):
    if not color_ranges:
        return np.zeros(hsv.shape[:2], dtype=np.uint8)
    
    combined_mask = cv.inRange(hsv, color_ranges[0][0], color_ranges[0][1])
    
    for lower, upper in color_ranges[1:]:
        mask = cv.inRange(hsv, lower, upper)
        combined_mask = cv.bitwise_or(combined_mask, mask)
    
    return combined_mask

class Picamera2Detector:
    def __init__(self, show_display=True):
        self.show_display = show_display
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "BGR888"}
            )
            self.picam2.start()
            time.sleep(1)
        except Exception as e:
            raise
    
    def detect_obstacles(self):
        try:
            frame = self.picam2.capture_array()
            
            if frame is None or frame.size == 0:
                return []
            
            if len(frame.shape) != 3 or frame.shape[0] < CROP_BOTTOM or frame.shape[1] < CAMERA_WIDTH:
                return []
            
            cropped_frame = frame[CROP_TOP:CROP_BOTTOM, :]
            
            if cropped_frame.size == 0:
                return []
            
            hsv_frame = cv.cvtColor(cropped_frame, cv.COLOR_RGB2HSV)
            hsv_frame = cv.GaussianBlur(hsv_frame, (5, 5), 0)
            
            detected_obstacles = []
            
            if self.show_display:
                display_frame = cropped_frame.copy()
                cv.line(display_frame, (RED_TARGET, 0), (RED_TARGET, 200), (0, 0, 255), 2)
                cv.line(display_frame, (GREEN_TARGET, 0), (GREEN_TARGET, 200), (0, 255, 0), 2)
                cv.rectangle(display_frame, (0, OBSTACLE_Y_MIN), (CAMERA_WIDTH, OBSTACLE_Y_MAX), 
                           (255, 255, 0), 1)
            
            color_configs = {
                'red': RED_RANGES,
                'green': GREEN_RANGES
            }

            for color_name, color_ranges in color_configs.items():
                mask = create_multi_range_mask(hsv_frame, color_ranges)
                
                kernel = np.ones((5, 5), np.uint8)
                mask = cv.erode(mask, kernel, iterations=1)
                mask = cv.dilate(mask, kernel, iterations=2)
                
                contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    area = cv.contourArea(contour)
                    
                    if area < OBSTACLE_MIN_AREA:
                        continue
                    
                    x, y, w, h = cv.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    if center_y < OBSTACLE_Y_MIN or center_y > OBSTACLE_Y_MAX:
                        continue
                    
                    aspect_ratio = float(w) / h if h > 0 else 0
                    if aspect_ratio > 3.5 or aspect_ratio < 0.25:
                        continue
                    
                    distance = np.sqrt((center_x - CAMERA_WIDTH/2)**2 + 
                                      (center_y - (CROP_BOTTOM-CROP_TOP)/2)**2)
                    
                    obstacle_info = {
                        'color': color_name,
                        'center_x': center_x,
                        'center_y': center_y,
                        'area': area,
                        'width': w,
                        'height': h,
                        'distance': distance
                    }
                    detected_obstacles.append(obstacle_info)
                    
                    if self.show_display:
                        color_bgr = (0, 0, 255) if color_name == 'red' else (0, 255, 0)
                        cv.rectangle(display_frame, (x, y), (x+w, y+h), color_bgr, 2)
                        cv.circle(display_frame, (center_x, center_y), 5, color_bgr, -1)
                        label = f"{color_name.upper()} A:{int(area)}"
                        cv.putText(display_frame, label, (x, y-10), 
                                  cv.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)
                        target_x = RED_TARGET if color_name == 'red' else GREEN_TARGET
                        cv.line(display_frame, (center_x, center_y), (target_x, center_y), 
                               color_bgr, 1)
            
            if self.show_display:
                info_text = f"Obstacles: {len(detected_obstacles)}"
                cv.putText(display_frame, info_text, (10, 30), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv.imshow('WRO Camera View', display_frame)
                cv.waitKey(1)
            
            return detected_obstacles
            
        except Exception as e:
            return []
    
    def cleanup(self):
        try:
            self.picam2.stop()
            cv.destroyAllWindows()
        except:
            pass

class Vehicle:
    def __init__(self):
        self._init_board()
        self._init_motor()
        self._init_servo()
        self._init_compass()
        self._init_lidar()
        self._init_camera()
        
        self.initial_heading = None
        self.target_heading = None
        self.heading_offset = 0
        
        self.turn_count = 0
        self.turn_direction = None
        
        self.last_left_error = 0
        self.last_right_error = 0
        self.current_speed = BASE_SPEED
        self.navigation_mode = "wall_follow"
        
        self.steering_history = deque(maxlen=3)
        
        self.lidar_buffer = bytearray()
        self.front_data = deque(maxlen=20)
        self.left_data = deque(maxlen=20)
        self.right_data = deque(maxlen=20)

        self.last_line_detection_time = 0
        
        self.prev_obstacle_error = 0
        self.just_finished_turn = False
        self.turn_finish_time = 0
        
        self.parking_phase = False
        self.inner_wall_side = None
        self.last_inner_error = 0
        self.parking_pillars_detected = 0
        self.last_outer_distance = 9999
        self.parking_exit_time = None
        
        self.last_obstacle_time = 0
        self.last_obstacle_id = None
        
        self.corridor_width_history = deque(maxlen=20)
        
        atexit.register(self.cleanup)
    
    def _init_board(self):
        self.board = Board(1, 0x10)
        while self.board.begin() != self.board.STA_OK:
            time.sleep(1)
        self.board.set_adc_enable()
    
    def _init_motor(self):
        self.motor = SimpleMotor(MOTOR_IN1, MOTOR_IN2, MOTOR_ENA)
    
    def _init_servo(self):
        self.servo_ctrl = Servo(self.board)
        self.servo_ctrl.begin()
        self.servo_ctrl.move(SERVO_CHANNEL, SERVO_CENTER)
    
    def _init_compass(self):
        try:
            i2c = busio.I2C(board_pins.SCL, board_pins.SDA)
            self.compass = adafruit_bno055.BNO055_I2C(i2c)
            time.sleep(1)
        except:
            self.compass = None
    
    def _init_lidar(self):
        self.lidar = serial.Serial(port=LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=1)
        self.lidar.write(bytes([0xA5, 0x20]))
        time.sleep(0.1)
        response = self.lidar.read(7)
    
    def _init_camera(self):
        self.camera = Picamera2Detector(show_display=SHOW_CAMERA)
    
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
        return max(-MAX_TURN_ANGLE, min(MAX_TURN_ANGLE, correction))
    
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
            pass
    
    def update_lidar_aggressive(self):
        for _ in range(3):
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
            except:
                pass
    
    def _parse_scan_point(self, data):
        if len(data) < 5:
            return None
        
        quality = (data[0] >> 2) & 0x3F
        angle_q6 = (data[1] >> 1) | (data[2] << 7)
        angle = (angle_q6 / 64.0) % 360.0
        distance_q2 = data[3] | (data[4] << 8)
        distance = distance_q2 / 4.0
        
        if quality < LIDAR_MIN_QUALITY:
            return None
        
        if distance < LIDAR_MIN_DISTANCE or distance > LIDAR_MAX_DISTANCE:
            return None
        
        return {
            'angle': angle,
            'distance': distance,
            'quality': quality
        }
    
    def _is_front_angle(self, angle):
        return angle >= FRONT_MIN or angle <= FRONT_MAX
    
    def _is_left(self, angle):
        return LEFT_ZONE_MIN <= angle <= LEFT_ZONE_MAX
    
    def _is_right(self, angle):
        return RIGHT_ZONE_MIN <= angle <= RIGHT_ZONE_MAX
    
    def _filter_outliers(self, data):
        if len(data) < 3:
            return list(data)
        
        median = statistics.median(data)
        filtered = [d for d in data if abs(d - median) < median * LIDAR_OUTLIER_THRESHOLD]
        
        return filtered if filtered else list(data)
    
    def distance_front(self):
        if self.front_data:
            filtered = self._filter_outliers(self.front_data)
            if filtered:
                return round(statistics.median(filtered))
        return None
    
    def distance_left(self):
        if self.left_data:
            filtered = self._filter_outliers(self.left_data)
            if filtered:
                return round(statistics.median(filtered))
        return None
    
    def distance_right(self):
        if self.right_data:
            filtered = self._filter_outliers(self.right_data)
            if filtered:
                return round(statistics.median(filtered))
        return None
    
    def read_light_sensor(self, port):
        try:
            value = self.board.get_adc_value(port)
            return value
        except:
            return None
        
    def get_turn_side(self):
        current_heading = self.heading()
        if current_heading is None:
            return 'both'
        heading_error = angle_difference(self.target_heading, current_heading)
        if heading_error > 35:
            return 'left'
        elif heading_error < -35:
            return 'right'
        else:
            return 'both'

    def get_side_distances(self):
        turn_side = self.get_turn_side()
        if turn_side == 'left':
            return self.distance_left(), None
        elif turn_side == 'right':
            return None, self.distance_right()
        else:
            return self.distance_left(), self.distance_right()
        
    def detect_line_and_direction(self):
        current_time = time.time()
        if current_time - self.last_line_detection_time < LINE_IGNORE_TIME:
            return False, None
        
        red_value = self.read_light_sensor(LIGHT_SENSOR_RED)
        white_value = self.read_light_sensor(LIGHT_SENSOR_WHITE)
        
        if red_value is None or white_value is None:
            return False, None
        
        if self.turn_direction is None:
            if red_value < RED_THRESHOLD or white_value < WHITE_THRESHOLD:
                lowest_red = red_value
                sample_start = time.time()
                while time.time() - sample_start < FIRST_LINE_SAMPLE_TIME:
                    current_red = self.read_light_sensor(LIGHT_SENSOR_RED)
                    if current_red is not None and current_red < lowest_red:
                        lowest_red = current_red
                    time.sleep(0.01)
                
                if lowest_red > RED_THRESHOLD:
                    direction = 'right'
                else:
                    direction = 'left'
                
                self.last_line_detection_time = current_time
                return True, direction
        else:
            if white_value < WHITE_THRESHOLD:
                self.last_line_detection_time = current_time
                return True, self.turn_direction
        
        return False, None
    
    def execute_forward_turn(self, direction):
        old_target = self.target_heading
        
        if direction == 'right':
            self.target_heading = normalize_angle(old_target + 90)
        else:
            self.target_heading = normalize_angle(old_target - 90)
        
        self.just_finished_turn = True
        self.turn_finish_time = time.time()
    
    def get_closest_obstacle(self):
        obstacles = self.camera.detect_obstacles()
        if not obstacles:
            return None
        
        valid_obstacles = [o for o in obstacles if o['center_y'] > OBSTACLE_Y_MIN]
        
        if not valid_obstacles:
            return None
        
        if self.parking_exit_time is not None:
            time_since_exit = time.time() - self.parking_exit_time
            if time_since_exit < 1.0:
                MIDDLE_X_MIN = 240
                MIDDLE_X_MAX = 400
                middle_obstacles = [o for o in valid_obstacles 
                                  if MIDDLE_X_MIN <= o['center_x'] <= MIDDLE_X_MAX]
                
                if middle_obstacles:
                    valid_obstacles = middle_obstacles
                else:
                    return None
        
        closest = min(valid_obstacles, key=lambda o: o['distance'] * 0.7 + (OBSTACLE_Y_MAX - o['center_y']) * 0.3)
        return closest
    
    def calculate_obstacle_avoidance_angle(self, obstacle):
        if obstacle is None:
            self.prev_obstacle_error = 0
            return None
        
        if obstacle['center_y'] < OBSTACLE_PASSED_Y_THRESHOLD:
            self.prev_obstacle_error = 0
            return None
        
        obstacle_id = f"{obstacle['color']}_{obstacle['center_x']//50}_{obstacle['center_y']//50}"
        
        current_time = time.time()
        
        if obstacle_id == self.last_obstacle_id:
            if current_time - self.last_obstacle_time < OBSTACLE_PASSED_TIME:
                return None
        
        if obstacle['color'] == 'red':
            target = RED_TARGET
            kp_multiplier = 1.2
        else:
            target = GREEN_TARGET
            kp_multiplier = 0.85
        
        error = target - obstacle['center_x']
        
        if abs(error) < 30 and obstacle['center_y'] < OBSTACLE_PASSED_Y_THRESHOLD:
            self.last_obstacle_id = obstacle_id
            self.last_obstacle_time = current_time
            self.prev_obstacle_error = 0
            return None
        
        p_term = error * OBSTACLE_KP * kp_multiplier
        d_term = (error - self.prev_obstacle_error) * OBSTACLE_KD
        
        steering_angle = p_term + d_term
        
        urgency_factor = 1.0
        if obstacle['center_y'] > 180:
            urgency_factor = 1.5
        elif obstacle['area'] > 8000:
            urgency_factor = 1.3
        
        steering_angle *= urgency_factor
        
        self.prev_obstacle_error = error
        
        steering_angle = clamp(steering_angle, -MAX_TURN_ANGLE, MAX_TURN_ANGLE)
        
        return steering_angle
    
    def check_emergency_obstacle(self, obstacle):
        if obstacle is not None:
            if obstacle.get('area', 0) > EMERGENCY_AREA_THRESHOLD:
                return True
            
            if obstacle.get('center_y', 0) > EMERGENCY_Y_THRESHOLD:
                return True
        
        return False
    
    def execute_emergency_maneuver(self):
        self.steer_center()
        self.stop()
        time.sleep(0.3)
        
        self.motor.set_speed(-SLOW_SPEED)
        time.sleep(1.0)
        
        self.stop()
        time.sleep(0.2)
        
        self.forward(SLOW_SPEED)
        time.sleep(0.3)
    
    def forward(self, speed=BASE_SPEED):
        self.motor.set_speed(speed)
        self.current_speed = speed
    
    def stop(self):
        self.motor.stop()
        self.current_speed = 0
    
    def steer(self, angle):
        angle = max(-MAX_TURN_ANGLE, min(MAX_TURN_ANGLE, angle))
        servo_pos = SERVO_CENTER - angle
        self.servo_ctrl.move(SERVO_CHANNEL, int(servo_pos))
    
    def steer_smooth(self, target_angle):
        target_angle = max(-MAX_TURN_ANGLE, min(MAX_TURN_ANGLE, target_angle))
        self.steering_history.append(target_angle)
        
        if len(self.steering_history) > 0:
            smooth_angle = sum(self.steering_history) / len(self.steering_history)
            self.steer(smooth_angle)
    
    def steer_center(self):
        self.servo_ctrl.move(SERVO_CHANNEL, SERVO_CENTER)
        self.steering_history.clear()
    
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
        left_dist = self.distance_left()
        right_dist = self.distance_right()
        
        corridor_type = self.estimate_corridor_width()
        
        if corridor_type == "narrow":
            target_distance = TARGET_WALL_NARROW
        else:
            target_distance = TARGET_WALL_WIDE
        
        steering = 0
        heading_correction = self.calculate_heading_correction()
        
        if right_dist and right_dist < 800:
            error = target_distance - right_dist
            derivative = error - self.last_right_error
            self.last_right_error = error
            
            steering = error * KP_WALL + derivative * KD_WALL
            
            if right_dist < MIN_SAFE_DISTANCE:
                self.forward(SLOW_SPEED)
            else:
                self.forward(BASE_SPEED)
        
        elif left_dist and left_dist < 800:
            error = target_distance - left_dist
            derivative = error - self.last_left_error
            self.last_left_error = error
            
            steering = -(error * KP_WALL + derivative * KD_WALL)
            
            if left_dist < MIN_SAFE_DISTANCE:
                self.forward(SLOW_SPEED)
            else:
                self.forward(BASE_SPEED)
        
        else:
            steering = 0
            self.forward(BASE_SPEED)
            self.last_left_error = 0
            self.last_right_error = 0
        
        steering += heading_correction
        
        return max(-MAX_TURN_ANGLE, min(MAX_TURN_ANGLE, steering))
    
    def calculate_outer_wall_steering(self):
        if self.inner_wall_side == 'left':
            wall_dist = self.distance_right()
            multiplier = 1
        else:
            wall_dist = self.distance_left()
            multiplier = -1
        
        if wall_dist is None or wall_dist > 1000:
            return self.calculate_heading_correction()
        
        error = OUTER_WALL_TARGET - wall_dist
        derivative = error - self.last_inner_error
        self.last_inner_error = error
        
        wall_steering = multiplier * (error * KP_PARKING + derivative * KD_PARKING)
        heading_correction = self.calculate_heading_correction()
        
        total_steering = wall_steering + heading_correction
        
        return clamp(total_steering, -MAX_TURN_ANGLE, MAX_TURN_ANGLE)
    
    def exit_parking_lot(self):
        EXIT_THRESHOLD = 750
        CHECK_INTERVAL = 0.1
        EXIT_DRIVE_TIME = 1.1
        
        self.steer_center()
        self.stop()
        
        last_check = time.time()
        
        while True:
            self.update_lidar()
            
            if time.time() - last_check > CHECK_INTERVAL:
                front_dist = self.distance_front() or 0
                left_dist = self.distance_left() or 0
                right_dist = self.distance_right() or 0
                
                if left_dist > EXIT_THRESHOLD:
                    self.steer(MAX_TURN_ANGLE)
                    break
                elif right_dist > EXIT_THRESHOLD:
                    self.steer(-MAX_TURN_ANGLE)
                    break
                
                last_check = time.time()
            
            time.sleep(0.02)
        
        self.forward(43)
        time.sleep(EXIT_DRIVE_TIME)
        
        self.parking_exit_time = time.time()
        self.steer_center()
        time.sleep(0.2)
    
    def run_mission(self):
        BUTTON_PORT = 1
        BUTTON_THRESHOLD = 2000
        
        button_pressed = False
        while not button_pressed:
            button_value = self.read_light_sensor(BUTTON_PORT)
            
            if button_value and button_value < BUTTON_THRESHOLD:
                time.sleep(1)
                button_pressed = True
            
            time.sleep(0.1)
        
        for _ in range(20):
            self.update_lidar()
            time.sleep(0.05)
        self.set_initial_heading()
        
        try:
            self.exit_parking_lot()
            
            self.forward(BASE_SPEED)
            
            while self.turn_count < 12:
                self.update_lidar()
                
                line_detected, detected_direction = self.detect_line_and_direction()
                
                if line_detected:
                    self.turn_count += 1

                    if self.turn_direction is None:
                        self.turn_direction = detected_direction

                    self.execute_forward_turn(self.turn_direction)
                    continue
                
                ignore_obstacles = False
                if self.just_finished_turn:
                    elapsed = time.time() - self.turn_finish_time
                    if elapsed < IGNORE_SIDE_OBSTACLES_TIME:
                        ignore_obstacles = True
                    else:
                        self.just_finished_turn = False
                
                obstacle = self.get_closest_obstacle()
                
                if obstacle and ignore_obstacles:
                    cx = obstacle['center_x']
                    if cx < OBSTACLE_X_MIN or cx > OBSTACLE_X_MAX:
                        obstacle = None
                
                if obstacle and self.check_emergency_obstacle(obstacle):
                    self.execute_emergency_maneuver()
                    continue
                
                left_dist = self.distance_left()
                right_dist = self.distance_right()
                
                wall_too_close = (left_dist and left_dist < MIN_SAFE_DISTANCE) or \
                                 (right_dist and right_dist < MIN_SAFE_DISTANCE)
                
                if obstacle and not wall_too_close:
                    obstacle_steer = self.calculate_obstacle_avoidance_angle(obstacle)
                    if obstacle_steer is not None:
                        self.steer_smooth(obstacle_steer)
                        self.calculate_steering()
                    else:
                        wall_steer = self.calculate_steering()
                        self.steer_smooth(wall_steer)
                else:
                    wall_steer = self.calculate_steering()
                    self.steer_smooth(wall_steer)
                
                time.sleep(0.02)
            
            self.parking_phase = True
            
            if self.turn_direction == 'left':
                self.inner_wall_side = 'left'
            else:
                self.inner_wall_side = 'right'
            
            while True:
                self.update_lidar()
                
                line_detected, _ = self.detect_line_and_direction()
                
                if line_detected:
                    self.turn_count += 1
                    
                    self.steer_center()
                    self.forward(PARKING_SPEED)
                    
                    while True:
                        self.update_lidar()
                        front_dist = self.distance_front()
                        
                        if front_dist and front_dist < 500:
                            break
                        
                        heading_correction = self.calculate_heading_correction()
                        self.steer_smooth(heading_correction)
                        
                        time.sleep(0.02)
                    
                    self.execute_forward_turn(self.turn_direction)
                    time.sleep(0.3)
                    break
                
                obstacle = self.get_closest_obstacle()
                
                if obstacle and self.check_emergency_obstacle(obstacle):
                    self.execute_emergency_maneuver()
                    continue
                
                if obstacle:
                    obstacle_steer = self.calculate_obstacle_avoidance_angle(obstacle)
                    if obstacle_steer is not None:
                        self.steer_smooth(obstacle_steer)
                    else:
                        wall_steer = self.calculate_steering()
                        self.steer_smooth(wall_steer)
                else:
                    wall_steer = self.calculate_steering()
                    self.steer_smooth(wall_steer)
                
                time.sleep(0.02)
            
            if self.inner_wall_side == 'left':
                outer_wall_side = 'right'
            else:
                outer_wall_side = 'left'

            self.forward(PARKING_SPEED)

            while self.turn_count < 16:
                self.update_lidar()
                
                line_detected, _ = self.detect_line_and_direction()
                
                if line_detected:
                    self.turn_count += 1
                    
                    if self.turn_count >= 13 and self.turn_count <= 16:
                        self.steer_center()
                        self.forward(PARKING_SPEED)
                        
                        while True:
                            self.update_lidar()
                            front_dist = self.distance_front()
                            
                            if front_dist and front_dist < 500:
                                break
                            
                            heading_correction = self.calculate_heading_correction()
                            self.steer_smooth(heading_correction)
                            
                            time.sleep(0.02)
                    
                    self.execute_forward_turn(self.turn_direction)
                
                steering = self.calculate_outer_wall_steering()
                self.steer_smooth(steering)
                self.forward(PARKING_SPEED)
                
                time.sleep(0.02)
            
            self.forward(30)
            
            while True:
                self.update_lidar()
                
                current_heading = self.heading()
    
                heading_valid = False
                if current_heading is not None:
                    if current_heading >= 330 or current_heading <= 30:
                        heading_valid = True
                
                if outer_wall_side == 'left':
                    outer_dist = self.distance_left()
                else:
                    outer_dist = self.distance_right()
                
                if heading_valid and outer_dist and outer_dist < 270:
                    if self.last_outer_distance > 250:
                        self.steer_center()
                        self.forward(40)
                        time.sleep(0.85)
                        self.parking_pillars_detected += 1
                        
                        if self.parking_pillars_detected >= 1:
                            if self.turn_direction == 'right':
                                parking_turn = 'right'
                                target_heading_after_turn = normalize_angle(current_heading + 90)
                                final_heading = 0
                                steer_direction = -MAX_TURN_ANGLE
                            else:
                                parking_turn = 'left'
                                target_heading_after_turn = normalize_angle(current_heading - 90)
                                final_heading = 0
                                steer_direction = MAX_TURN_ANGLE
                            
                            self.target_heading = target_heading_after_turn
                            self.steer(steer_direction)
                            self.forward(45)
                            
                            while True:
                                self.update_lidar()
                                current = self.heading()
                                
                                if current is None:
                                    time.sleep(0.02)
                                    continue
                                
                                heading_error = abs(angle_difference(target_heading_after_turn, current))
                                
                                if heading_error <= 2:
                                    break
                                
                                time.sleep(0.02)
                            
                            self.stop()
                            
                            self.steer_center()
                            self.forward(35)
                            
                            while True:
                                self.update_lidar()
                                front_dist = self.distance_front()
                                
                                if front_dist and front_dist < 110:
                                    break
                                
                                heading_correction = self.calculate_heading_correction()
                                self.steer_smooth(heading_correction)
                                
                                time.sleep(0.02)
                            
                            self.stop()
                            
                            self.steer_center()
                            self.motor.set_speed(-35)
                            
                            while True:
                                self.update_lidar()
                                ultra = self.read_light_sensor(ULTRA)
                                
                                if ultra and ultra < 200:
                                    break
                                
                                heading_correction = -1 * self.calculate_heading_correction()
                                self.steer_smooth(heading_correction)
                                
                                time.sleep(0.02)
                            
                            self.stop()
                            
                            if parking_turn == 'right':
                                self.steer(-MAX_TURN_ANGLE)
                                target_min = 340
                                target_max = 343
                            else:
                                self.steer(MAX_TURN_ANGLE)
                                target_min = 17
                                target_max = 20
                            
                            self.motor.set_speed(-45)
                            
                            while True:
                                self.update_lidar()
                                current = self.heading()
                                ultra = self.read_light_sensor(ULTRA)
                                
                                if current is None:
                                    time.sleep(0.02)
                                    continue
                                
                                if (current >= target_min and current <= target_max):
                                    break
                                
                                time.sleep(0.02)

                            self.stop()
                            self.steer_center()
                            
                            break
                
                self.last_outer_distance = outer_dist if outer_dist else 9999
                
                if outer_dist and outer_dist < 1000:
                    if outer_wall_side == 'left':
                        error = 307 - outer_dist
                        steering = -error * KP_PARKING
                    else:
                        error = 195 - outer_dist
                        steering = error * KP_PARKING
                else:
                    steering = 0
                
                heading_correction = self.calculate_heading_correction()
                total_steering = steering + heading_correction
                
                self.steer_smooth(clamp(total_steering, -MAX_TURN_ANGLE, MAX_TURN_ANGLE))
                self.forward(43)
                
                time.sleep(0.02)
        
        except KeyboardInterrupt:
            pass
        
        finally:
            self.stop()
            self.steer_center()
            time.sleep(0.5)
    
    def cleanup(self):
        try:
            self.stop()
            self.steer_center()
            time.sleep(0.2)
            
            if hasattr(self, 'motor'):
                self.motor.cleanup()
            
            if hasattr(self, 'lidar') and self.lidar.is_open:
                self.lidar.write(bytes([0xA5, 0x25]))
                time.sleep(0.1)
                self.lidar.close()
            
            if hasattr(self, 'camera'):
                self.camera.cleanup()
            
            GPIO.cleanup()
        
        except Exception as e:
            pass

if __name__ == "__main__":
    try:
        vehicle = Vehicle()
        time.sleep(1)
        vehicle.run_mission()
    
    except Exception as e:
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            GPIO.cleanup()
        except:
            pass
