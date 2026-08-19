import time
import argparse
import numpy as np
import cv2
import matplotlib.pyplot as plt
from picar import PiCar


parser = argparse.ArgumentParser(description="Objective 4: Drive straight to object using PID + vision")
parser.add_argument('--mock_car', action='store_true', help='Use mock car instead of real hardware')
parser.add_argument('--tim', type=int, default=10, help='Max time to run (s)')
parser.add_argument('--rps', type=float, default=5.0, help='Target RPS')
parser.add_argument('--cameraDelay', type=float, default=0.2, help='Delay between image captures')
parser.add_argument('--delta', type=float, default=0.7, help='Adjustment factor for servo steering')
parser.add_argument('--adSample', type=int, default=5, help='Sampling rate (ms)')
parser.add_argument('--speedCalc', type=int, default=250, help='Speed calc interval (ms)')
parser.add_argument('--adDelay', type=int, default=0, help='Delay after motor start before control')
parser.add_argument('--motorDelay', type=int, default=1, help='Delay before motor starts')
parser.add_argument('--Kp', type=float, default=9.5)
parser.add_argument('--Ki', type=float, default=5.0)
parser.add_argument('--Kd', type=float, default=0.0)
parser.add_argument('--debug', action='store_true')
args = parser.parse_args()

car = PiCar(mock_car=args.mock_car, threaded=True)
car.set_motor(0)
car.set_steer_servo(0)
car.set_swivel_servo(0)

AD_pin = 0
integral = 0
previous_error = 0
last_rps_time = time.time()
transition_count = 0
is_dark = 0
threshold = 15
ad_readings = []
ad_diff = []
ad_ma = []
RPSs = []
time_log = []
dist_log = []
rps_log = []
steering_pwm = 0

start_time = time.time()
last_sample_time = start_time
camera_next = start_time
sample_interval = args.adSample / 1000  # seconds from adSample argument
log_filename = f"car_{args.rps}rps.txt"

print("Objective 4: Driving toward object...")

#blue object function
def findBlue(array, counter=None):
    if array is None:
        print("Warning: Received None image in findBlue")
        return 0
        
#    array = cv2.flip(array, 0)
    height, width = array.shape[:2]
    center_x = width // 2
    center_y = height // 2
    
    # BGR to RGB
    
    array_rgb = array 
    #cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
    
    hsv = cv2.cvtColor(array_rgb, cv2.COLOR_RGB2HSV)
    
    mask = cv2.inRange(hsv, (100, 50, 50), (140, 255, 255))
    
    mask_blur = cv2.GaussianBlur(mask, (5, 5), 0)
    
    thresh = cv2.threshold(mask_blur, 50, 255, cv2.THRESH_BINARY)[1]
    
    if args.debug:
        cv2.imwrite('array_rgb_new.jpg', array_rgb)
        cv2.imwrite('mask.jpg', mask)
        cv2.imwrite('thresh_img.jpg', thresh)
    
    M = cv2.moments(thresh)
    
    if M["m00"] == 0:
        if args.debug:
            print("No blue object detected")
        return 0
        
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    
    x_cord = center_x - cX
    y_cord = center_y - cY
    
    # Draw circle used for debugging
    img2 = array_rgb.copy()
    cv2.circle(img2, (cX, cY), 5, (0, 0, 255), 2)  # red circle
    #debug
    if args.debug or counter is not None:
        filename = f"frame_{counter:03d}.jpg" if counter is not None else 'blue.jpg'
        cv2.imwrite(filename, cv2.cvtColor(img2, cv2.COLOR_RGB2BGR))
        if args.debug:
            print(f"Saved {filename}, centroid at ({cX}, {cY}), offset: ({x_cord}, {y_cord})")
    
    #calc angle
    theta = np.arctan2(x_cord, center_y - cY)
    degrees = np.degrees(theta)
    
    if args.debug:
        print(f'Angle: {degrees} degrees')
    
    changein_PWM = degrees / 9  # Serv
    return changein_PWM

#motor delay
while time.time() - start_time < args.motorDelay:
    time.sleep(0.01)

car.set_motor(0)
if args.adDelay > 0:
    time.sleep(args.adDelay)

# main
while time.time() - start_time < args.tim:
    current_time = time.time()
    elapsed_time = current_time - start_time

    #stop around 2 feet
    dist = car.read_distance()
    if dist is None:
        dist = 62
    if dist <= 61:
        print("Reached object — stopping.")
        break

    #steering
    if current_time >= camera_next:
        frame = car.get_image()
        if frame is not None:
            adjust = findBlue(frame) * args.delta
            steering_pwm += adjust
            steering_pwm = max(-10, min(10, steering_pwm))
            car.set_steer_servo(steering_pwm)
        camera_next = current_time + args.cameraDelay

    #trnasitions AD
    ad_val = car.adc.read_adc(AD_pin)
    ad_readings.append(ad_val)
    if len(ad_readings) > 1:
        ad_diff.append(ad_readings[-1] - ad_readings[-2])
    else:
        ad_diff.append(0)
    ad_ma.append(np.mean(ad_diff[-3:]))

    #threshold
    if len(ad_ma) >= 20:
        recent_changes = ad_ma[-100:] if len(ad_ma) >= 100 else ad_ma
        if max(recent_changes) != min(recent_changes):
            threshold = 0.2 * (max(recent_changes) - min(recent_changes)) / 2
            if args.debug:
                print(f"Updated threshold: {threshold:.3f}")

    # count transitions
    if len(ad_ma) >= 3:
        if ad_ma[-1] > threshold and not is_dark:
            transition_count += 1
            is_dark = 1
        elif ad_ma[-1] < -threshold and is_dark:
            transition_count += 1
            is_dark = 0

    #PID
    if current_time - last_rps_time >= args.speedCalc / 1000:
        elapsed = current_time - last_rps_time
        actual_rps = transition_count / 4 / elapsed if transition_count > 0 else (RPSs[-1] if RPSs else 0)
        RPSs.append(actual_rps)
        last_rps_time = current_time
        transition_count = 0

        error = args.rps - actual_rps
        integral += error * elapsed
        integral = max(min(integral, 100/args.Ki), -100/args.Ki)
        derivative = (error - previous_error) / elapsed if elapsed > 0 else 0
        previous_error = error

        pwm = args.Kp * error + args.Ki * integral + args.Kd * derivative + args.rps * 11.5
        pwm = min(max(pwm, 0), 100)
        car.set_motor(pwm)

        if args.debug:
            print(f"{elapsed_time:.2f}s: RPS = {actual_rps:.2f}, PWM = {pwm:.1f}, Dist = {dist:.1f} cm")

    if current_time - last_sample_time >= sample_interval:
        time_log.append(elapsed_time)
        dist_log.append(dist)
        rps_log.append(RPSs[-1] if RPSs else 0)
        last_sample_time = current_time

    time.sleep(0.01)

car.set_motor(0)
car.stop()

with open(log_filename, 'w') as f:
    f.write("Time(s)\tDistance(cm)\tRPS\n")
    for t, d, r in zip(time_log, dist_log, rps_log):
        f.write(f"{t:.3f}\t{d:.1f}\t{r:.3f}\n")

# RPS plot
plt.figure(figsize=(10, 5))
plt.plot(time_log, rps_log, label="RPS", color='blue')
plt.axhline(y=args.rps, linestyle='--', color='green', label='Target RPS')
plt.xlabel("Time (s)")
plt.ylabel("RPS")
plt.title(f"Objective 4: RPS vs Time at {args.rps} RPS")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(f"car_{args.rps}rps_plot.png")
plt.show()

print(f"Run complete. Data saved to {log_filename}")
car.stop()

