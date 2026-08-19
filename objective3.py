import time
import argparse
from picar import PiCar
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description="Objective 3: Drive at set speed and stop before wall")
parser.add_argument('--mock_car', action='store_true', help='Use mock car instead of real hardware')
parser.add_argument('--tim', type=int, default=10, help='Time to run (s)')
parser.add_argument('--debug', action='store_true', help='Enable debug output')
parser.add_argument('--rps', type=float, default=5.0, help='Target speed in RPS')
parser.add_argument('--Kp', type=float, default=9.5, help='Proportional gain')
parser.add_argument('--Ki', type=float, default=5.0, help='Integral gain')
parser.add_argument('--Kd', type=float, default=0.0, help='Derivative gain')
parser.add_argument('--speedCalc', type=float, default=0.4, help='RPS calculation interval (s)')
args = parser.parse_args()

car = PiCar(mock_car=args.mock_car, threaded=True)
car.set_motor(0)
car.set_swivel_servo(0)
car.set_steer_servo(0)

#
print("Starting drive at fixed speed toward end of hallway")
start_dist = car.read_distance()
print(f"Starting distance from wall: {start_dist:.1f} cm")


AD_pin = 0
integral = 0
previous_error = 0
last_rps_time = time.time()
transition_count = 0
is_dark = 0
threshold = 15  #placeholder
ad_readings = []
ad_diff = []
ad_ma = []
RPSs = []


log_filename = f"manual_car_{args.rps}rps.txt"
time_log = []
dist_log = []
rps_log = []

start_time = time.time()
last_sample_time = start_time
sample_interval = 0.25  
steering_pwm = 0

while True: #keep it going till hit wall
    current_time = time.time()
    elapsed_time = current_time - start_time

    if elapsed_time >= args.tim:
        print("Reached max runtime. Stopping.")
        break #stop the car


    

    dist = car.read_distance()
    if dist is None:
        dist = 92 #prevent error
        
    if dist < 75:  # around 3 ft
        print(f"Wall detected within 3 feet: {dist:.1f} cm — stopping.")
        break
    

   #get keystroke
    key = car.get_keyin()
    if key:
        if key == 'd':
            steering_pwm = max(steering_pwm - 0.5, -10)  # turn right
        elif key == 's':
            steering_pwm = min(steering_pwm + 0.5, 10)   # turn left

        car.set_steer_servo(steering_pwm)

    
    ad_val = car.adc.read_adc(AD_pin)
    ad_readings.append(ad_val)
    if len(ad_readings) > 1:
        ad_diff.append(ad_readings[-1] - ad_readings[-2])
    else:
        ad_diff.append(0)

    if len(ad_diff) < 3:
        ad_ma.append(sum(ad_diff[-3:]) / len(ad_diff))
    else:
        ad_ma.append(sum(ad_diff[-3:]) / 3)

    #threshold calc
    if len(ad_ma) >= 20:
        recent_changes = ad_ma[-100:] if len(ad_ma) >= 100 else ad_ma
        if max(recent_changes) != min(recent_changes):
            threshold = 0.2 * (max(recent_changes) - min(recent_changes)) / 2
            if args.debug:
                print(f"Updated threshold: {threshold:.3f}")

    if len(ad_ma) >= 3:
        if ad_ma[-1] > threshold and not is_dark:
            transition_count += 1
            is_dark = 1
        elif ad_ma[-1] < -threshold and is_dark:
            transition_count += 1
            is_dark = 0

    #pid every speedCalc seconds
    if current_time - last_rps_time >= args.speedCalc:
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
            print(f"{elapsed_time:.2f}s: RPS = {actual_rps:.2f}, PWM = {pwm:.1f}, Distance = {dist:.1f} cm")

    #used for writing the data
    if current_time - last_sample_time >= sample_interval:
        time_log.append(elapsed_time)
        dist_log.append(dist)
        rps_log.append(RPSs[-1] if RPSs else 0)
        last_sample_time = current_time

    time.sleep(0.01)

#write data
car.set_motor(0)
car.stop()
with open(log_filename, 'w') as f:
    f.write("Time(s)\tDistance(cm)\tRPS\n")
    for t, d, r in zip(time_log, dist_log, rps_log):
        f.write(f"{t:.3f}\t{d:.1f}\t{r:.3f}\n")

print(f"Run complete. Data saved to {log_filename}")

#rps v time
plt.figure(figsize=(10, 5))
plt.plot(time_log, rps_log, label="Measured RPS", color='blue')
plt.axhline(y=args.rps, linestyle='--', color='green', label='Target RPS')
plt.xlabel("Time (s)")
plt.ylabel("RPS")
plt.title(f"RPS vs Time at Target {args.rps} RPS")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(f"rps_plot_{args.rps}rps.png")
plt.show()


