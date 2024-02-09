import cv2
from detector import detect_objects, load_network

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        adjustment = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return adjustment

def get_yaw_adjustment(pid, error_in_degrees, dt):
    return pid.update(error_in_degrees, dt)

def pixels_to_degrees(pixel_error, width, fov_degrees):
    ppd = width / fov_degrees  # Pixels per degree
    return pixel_error / ppd  # Error in degrees

def draw_text(frame, text, pos, scale=0.5, color=(0, 255, 0), thickness=1):
    cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)

if __name__ == "__main__":
    net, output_layers = load_network()
    cap = cv2.VideoCapture(0)
    fov_degrees = 90  # Example FOV, adjust to your camera's specification

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    pid = PIDController(1.0, 0.1, 0.05)
    dt = 0.05  # Time step for PID controller update

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            height, width = frame.shape[:2]
            width_center = width // 2

            biggest_box_center, _ = detect_objects(frame, net, output_layers)
            if biggest_box_center:
                center_x, center_y = biggest_box_center
                cv2.circle(frame, (center_x, center_y), 10, (0, 255, 0), -1)
                
                pixel_error = center_x - width_center
                error_in_degrees = pixels_to_degrees(pixel_error, width, fov_degrees)
                adjustment = get_yaw_adjustment(pid, error_in_degrees, dt)
                
                draw_text(frame, f"Center: {center_x-width_center}, {center_y-height // 2}", (10, 30))
                draw_text(frame, f"Yaw adjustment: {adjustment:.2f} degrees", (10, 90))

            draw_text(frame, f"Resolution: {width}x{height}", (10, 60))
            cv2.imshow("Frame", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()