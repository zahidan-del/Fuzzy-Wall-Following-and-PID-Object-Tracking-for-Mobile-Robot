import cv2
import numpy as np
from picamera2 import Picamera2
import time

class ObjectDetector:
    def _init_(self, resolution=(640, 480), log_path="pid_log.txt"):
        self.cam = Picamera2()
        config = self.cam.create_preview_configuration(
            main={"format": "RGB888", "size": resolution}
        )
        self.cam.configure(config)
        self.cam.start()
        self.resolution = resolution
        
        self.log_path = log_path
        self.last_detection_time = 0

        # PID Parameters
        self.pid_params = {
            'Kp': 0.1, 'Ki': 0.04, 'Kd': 0.008,
            'dt': 1/30.0, 'base_speed': 45,
            'max_speed': 100, 'center_x': resolution[0] // 2,
            'integral': 0.0, 'last_error': 0.0
        }

    def compute_pid(self, cx, return_error=False):
        error = cx - self.pid_params['center_x']
        self.pid_params['integral'] += error * self.pid_params['dt']
        derivative = (error - self.pid_params['last_error']) / self.pid_params['dt']
        
        corr = (self.pid_params['Kp'] * error +
                self.pid_params['Ki'] * self.pid_params['integral'] +
                self.pid_params['Kd'] * derivative)
        
        corr = np.clip(corr, -self.pid_params['base_speed'], self.pid_params['base_speed'])
        self.pid_params['last_error'] = error

        if return_error:
            return corr, error
        return corr


    def detect(self, color_lower, color_upper, close_zone):
        """Deteksi objek dan return state serta posisi"""
        frame = self.cam.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        # Threshold warna
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, color_lower, color_upper)
        
        # Noise reduction
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # Cari kontur
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Default values
        result = {
            'state': "INITIAL",
            'cx': None,
            'cy': None,
            'frame': frame,
            'mask': mask
        }
        
        # Proses kontur
        for cnt in contours:
            if cv2.contourArea(cnt) < 500 or cv2.boundingRect(cnt)[3] < 100:
                continue
                
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w//2, y + h//2
            
            result.update({
                'state': "OPEN",
                'cx': cx,
                'cy': cy
            })

            #Update waktu deteksi terakhir
            self.last_detection_time = time.time()
            
            # Gambar annotations
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"({cx},{cy})", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Cek zona CLOSE
            if (close_zone['x_min'] <= cx <= close_zone['x_max'] and
                close_zone['y_min'] <= cy <= close_zone['y_max']):
                result['state'] = "CLOSE"
            
            break
        
        return result

    def cleanup(self):
        self.cam.stop()