import os
os.environ["QT_QPA_PLATFORM"] = "offscreen"
import cv2
import numpy as np
import time
from motor import tankMotor
from ultrasonic import Ultrasonic
from camera import Camera
import threading

class FaceTrackingCar:
    def __init__(self):
        # Initialize components
        self.motor = tankMotor()
        self.ultrasonic = Ultrasonic()
        self.camera = Camera()
        
        # Load face detection cascade #opencvの正面顔検出モデル
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Camera settings
        self.camera_width = 640
        self.camera_height = 480
        self.center_x = self.camera_width // 2
        self.center_y = self.camera_height // 2
        
        # Control parameters
        self.face_tracking = True
        self.target_distance = 30  # Target distance in cm
        self.tolerance_distance = 5  # Distance tolerance
        self.tolerance_angle = 50  # Angle tolerance in pixels
        
        # Motor speeds
        self.forward_speed = 1200
        self.turn_speed = 1000
        self.approach_speed = 800
        
    def detect_faces(self, frame):
        """Detect faces in the frame and return the largest face center coordinates"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        
        if len(faces) > 0:
            # Find the largest face
            largest_face = max(faces, key=lambda face: face[2] * face[3])
            x, y, w, h = largest_face
            
            # Calculate center of the face
            face_center_x = x + w // 2
            face_center_y = y + h // 2
            
            # Draw rectangle around face (for debugging)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 255, 0), -1)
            
            return face_center_x, face_center_y, w, h
        
        return None, None, None, None

    def calculate_movement(self, face_x, face_y, face_width):
        """Calculate required movement based on face position"""
        if face_x is None:
            return "search"
        
        # Calculate horizontal deviation from center
        deviation_x = face_x - self.center_x
        
        # Determine movement direction
        if abs(deviation_x) > self.tolerance_angle:
            if deviation_x > 0:
                return "turn_right"
            else:
                return "turn_left"
        else:
            # Face is centered, check distance
            distance = self.ultrasonic.get_distance()
            
            if distance == 0:  # Invalid reading
                return "stop"
            elif distance > self.target_distance + self.tolerance_distance:
                return "forward"
            elif distance < self.target_distance - self.tolerance_distance:
                return "backward"
            else:
                return "stop"  # Target reached

    def execute_movement(self, action):
        """Execute the calculated movement"""
        print(f"Executing action: {action}")
        
        if action == "forward":
            self.motor.setMotorModel(self.approach_speed, self.approach_speed)
        elif action == "backward":
            self.motor.setMotorModel(-self.approach_speed, -self.approach_speed)
        elif action == "turn_left":
            self.motor.setMotorModel(-self.turn_speed, self.turn_speed)
        elif action == "turn_right":
            self.motor.setMotorModel(self.turn_speed, -self.turn_speed)
        elif action == "search":
            # Rotate slowly to search for faces
            self.motor.setMotorModel(-self.turn_speed//2, self.turn_speed//2)
        else:  # stop
            self.motor.setMotorModel(0, 0)

    def start_tracking(self):
        """Start face tracking"""
        print("Starting face tracking...")
        
        # Start camera
        self.camera.start_image()
        time.sleep(2)  # Give camera time to warm up
        
        try:
            while self.face_tracking:
                # Capture frame
                self.camera.save_image("temp_frame.jpg")
                frame = cv2.imread("temp_frame.jpg")
                
                if frame is None:
                    continue
                
                # Detect faces
                face_x, face_y, face_w, face_h = self.detect_faces(frame)
                
                # Calculate movement
                action = self.calculate_movement(face_x, face_y, face_w)
                
                # Execute movement
                self.execute_movement(action)
                
                # Get distance for display
                distance = self.ultrasonic.get_distance()
                
                # Display information
                if face_x is not None:
                    print(f"Face detected at ({face_x}, {face_y}), Distance: {distance:.1f}cm, Action: {action}")
                else:
                    print(f"No face detected, Distance: {distance:.1f}cm, Action: {action}")
                
                # Save debug image (optional)
                cv2.imwrite("debug_frame.jpg", frame)
                
                # Small delay to prevent overwhelming the system
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nStopping face tracking...")
        finally:
            self.stop_tracking()

    def stop_tracking(self):
        """Stop face tracking and cleanup"""
        self.face_tracking = False
        self.motor.setMotorModel(0, 0)
        self.motor.close()
        self.ultrasonic.close()
        self.camera.close()
        print("Face tracking stopped.")

# Advanced version with video streaming
class AdvancedFaceTrackingCar(FaceTrackingCar):
    def __init__(self):
        super().__init__()
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
    def frame_capture_thread(self):
        """Continuously capture frames in a separate thread"""
        self.camera.start_stream()
        
        while self.face_tracking:
            try:
                frame_data = self.camera.get_frame()
                if frame_data:
                    # Convert JPEG data to OpenCV frame
                    nparr = np.frombuffer(frame_data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    with self.frame_lock:
                        self.current_frame = frame
                        
            except Exception as e:
                print(f"Frame capture error: {e}")
                
        self.camera.stop_stream()
    
    def start_tracking(self):
        """Start advanced face tracking with video streaming"""
        print("Starting advanced face tracking...")
        
        # Start frame capture thread
        capture_thread = threading.Thread(target=self.frame_capture_thread)
        capture_thread.daemon = True
        capture_thread.start()
        
        time.sleep(2)  # Give camera time to start
        
        try:
            while self.face_tracking:
                with self.frame_lock:
                    if self.current_frame is not None:
                        frame = self.current_frame.copy()
                    else:
                        continue
                
                # Detect faces
                face_x, face_y, face_w, face_h = self.detect_faces(frame)
                
                # Calculate movement
                action = self.calculate_movement(face_x, face_y, face_w)
                
                # Execute movement
                self.execute_movement(action)
                
                # Get distance for display
                distance = self.ultrasonic.get_distance()
                
                # Display information
                if face_x is not None:
                    print(f"Face at ({face_x}, {face_y}), Distance: {distance:.1f}cm, Action: {action}")
                else:
                    print(f"No face, Distance: {distance:.1f}cm, Action: {action}")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nStopping face tracking...")
        finally:
            self.stop_tracking()

# Test functions
def test_face_detection():
    """Test face detection without movement"""
    car = FaceTrackingCar()
    car.camera.start_image()
    
    try:
        for i in range(10):
            car.camera.save_image("test_frame.jpg")
            frame = cv2.imread("test_frame.jpg")
            
            if frame is not None:
                face_x, face_y, face_w, face_h = car.detect_faces(frame)
                if face_x is not None:
                    print(f"Face detected at ({face_x}, {face_y})")
                    cv2.imwrite(f"face_detected_{i}.jpg", frame)
                else:
                    print("No face detected")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        pass
    finally:
        car.camera.close()

def test_basic_tracking():
    """Test basic face tracking"""
    car = FaceTrackingCar()
    car.start_tracking()

def test_advanced_tracking():
    """Test advanced face tracking with video streaming"""
    car = AdvancedFaceTrackingCar()
    car.start_tracking()

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python face_tracking_car.py [test_detection|basic|advanced]")
        exit()
    
    if sys.argv[1] == 'test_detection':
        test_face_detection()
    elif sys.argv[1] == 'basic':
        test_basic_tracking()
    elif sys.argv[1] == 'advanced':
        test_advanced_tracking()
    else:
        print("Invalid option. Use: test_detection, basic, or advanced")