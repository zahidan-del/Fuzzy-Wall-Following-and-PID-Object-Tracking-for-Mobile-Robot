import threading
import time
from PIDControl import ObjectDetector
from FuzzyControl import WallFollower
import serial
import os
from datetime import datetime
import numpy as np

class MainController:
    def __init__(self):
        self.serial_port = '/dev/ttyS0'
        self.baud_rate = 115200
        self.resolution = (640, 480)
        self.close_zone = {
            'x_min': 160, 'x_max': 480,
            'y_min': 380, 'y_max': 410
        }
        self.color_lower = np.array([0, 230, 70])
        self.color_upper = np.array([70, 255, 255])

        self.log_filename = f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        self.log_path = os.path.join(os.getcwd(), self.log_filename)

        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
        self.detector = ObjectDetector(self.resolution, log_path=self.log_path)
        self.wall_follower = WallFollower(log_path=self.log_path)

        self.mode = "WALL"
        self.last_state = None
        self.running = True

        self.fuzzy_errors = []
        self.fuzzy_outputs = []
        self.pid_errors = []
        self.pid_outputs = []
        self.fuzzy_times = []
        self.pid_times = []
        self.start_time = time.time()


        threading.Thread(target=self.serial_thread, daemon=True).start()
        threading.Thread(target=self.object_tracking_thread, daemon=True).start()
        

    def serial_thread(self):
        print("[THREAD] Serial started")
        while self.running:
            self.handle_serial()
            time.sleep(1 / 80.0)

    def object_tracking_thread(self):
        print("[THREAD] Object tracking started")
        current_fps = 10  
        last_detection_time = time.time()
        
        while self.running:
            start_time = time.time()
            
            # Eksekusi object tracking
            self.object_tracking()
            
            # Periksa apakah ada deteksi objek
            if self.detector.last_detection_time > last_detection_time:
                current_fps = 30  
                last_detection_time = self.detector.last_detection_time
            elif time.time() - last_detection_time > 2.0:  
                current_fps = 10 
            
            # Hitung waktu eksekusi dan sleep
            execution_time = time.time() - start_time
            sleep_time = max(0, (1.0 / current_fps) - execution_time)
            time.sleep(sleep_time)

    def handle_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            print(f"[SERIAL RAW] {line}")
            if line.startswith("DIST:") and self.mode == "WALL":
                try:
                    distance = int(line.split(':')[1])
                    if 0 < distance < 2000:
                        output, fuzzy_error = self.wall_follower.fuzzy_control(distance, return_error=True)
                        self.fuzzy_errors.append(fuzzy_error)
                        self.fuzzy_outputs.append(output)
                        elapsed_time = time.time() - self.start_time
                        self.fuzzy_times.append(elapsed_time)
                        self.control_motors(output)
                        print(f"[WALL] Distance: {distance}, Output: {output:.2f}")
                except ValueError:
                    print(f"[ERROR] Invalid data: {line}")

    def control_motors(self, correction=0, left_speed=None, right_speed=None):
        if not self.ser or not self.ser.is_open:
            print("[WARNING] Serial port not open. Skipping motor command.")
            return

        base_speed = 50
        max_speed = 130
        if left_speed is None or right_speed is None:
            left_speed = base_speed + correction
            right_speed = base_speed - correction
        left_speed = int(np.clip(left_speed, -max_speed, max_speed))
        right_speed = int(np.clip(right_speed, -max_speed, max_speed))
        command = f"MOTOR:{left_speed},{right_speed}\n"
        self.ser.write(command.encode())
        print(f"[MOTOR] {command.strip()}")

    def object_tracking(self):
        detection = self.detector.detect(self.color_lower, self.color_upper, self.close_zone)
        current_state = detection['state']

        # Jika state berubah, kirim ke gripper
        if current_state != self.last_state:
            self.ser.write(f"{current_state}\n".encode())
            print(f"[GRIPPER] State: {current_state}")
            self.last_state = current_state

        if current_state == "OPEN" and detection['cx'] is not None:
            self.mode = "OBJECT"
            corr, pid_error = self.detector.compute_pid(detection['cx'], return_error=True)
            self.pid_errors.append(pid_error)
            self.pid_outputs.append(corr)
            elapsed_time = time.time() - self.start_time
            self.pid_times.append(elapsed_time)
            left = self.detector.pid_params['base_speed'] - corr
            right = self.detector.pid_params['base_speed'] + corr
            self.control_motors(left_speed=left, right_speed=right)


        elif current_state == "CLOSE":
            self.mode = "OBJECT"
            self.control_motors(left_speed=0, right_speed=0)

        elif current_state == "INITIAL":
            self.mode = "WALL"

    def save_logs(self):
        with open(self.log_path, "w") as f:
            f.write("Fuzzy Error:\n")
            f.write(", ".join(f"{val:.2f}" for val in self.fuzzy_errors) + "\n\n")
            f.write("Fuzzy Output:\n")
            f.write(", ".join(f"{val:.2f}" for val in self.fuzzy_outputs) + "\n\n")
            f.write("Fuzzy Time:\n")
            f.write(", ".join(f"{val:.2f}" for val in self.fuzzy_times) + "\n\n")
            f.write("PID Error:\n")
            f.write(", ".join(f"{val:.2f}" for val in self.pid_errors) + "\n\n")
            f.write("PID Output:\n")
            f.write(", ".join(f"{val:.2f}" for val in self.pid_outputs) + "\n\n")
            f.write("PID Time:\n")
            f.write(", ".join(f"{val:.2f}" for val in self.pid_times) + "\n\n")


    def run(self):
        print("[SYSTEM] Controller started")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            print("[SYSTEM] Shutting down...")
        finally:
            self.cleanup()

    def cleanup(self):
        self.save_logs()
        self.detector.cleanup()
        self.ser.close()

if __name__ == "__main__":
    controller = MainController()
    controller.run()


