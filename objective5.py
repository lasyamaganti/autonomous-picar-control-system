import time
import argparse
from picar import PiCar
import matplotlib.pyplot as plt

#parameters, will require PID control
parser = argparse.ArgumentParser(description="bonus: drive with accel compensation")
parser.add_argument('--mock_car', action='store_true')
parser.add_argument('--tim', type=int, default=30)
parser.add_argument('--rps', type=float, default=5.0)
parser.add_argument('--Kp', type=float, default=9.5)
parser.add_argument('--Ki', type=float, default=5.0)
parser.add_argument('--Kd', type=float, default=0.0)
parser.add_argument('--debug', action='store_true')
args = parser.parse_args()

# setup car
car = PiCar(mock_car=args.mock_car, threaded=True)
car.set_motor(0)
car.set_swivel_servo(0)
car.set_steer_servo(0)

# initialize vars
pwm =0
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
rps_log = []
pwm_log = []
yaccel_log = []

start_time = time.time()
sample_interval = 0.25
last_sample_time = start_time
turnaround_done = False

print('start')

# main loop
while time.time() - start_time < args.tim:
    current_time = time.time()
    elapsed = current_time - last_rps_time 
    total_time = current_time - start_time #will be used for when to turn around

    # read y accel, only really need the y acceleration since x is goign to measure horizontal tilt. 
    yaccel = car.MPU_Read(2)
    yaccel_log.append(yaccel) #add to list

    # read ADC, same as other objectives
    ad_val = car.adc.read_adc(AD_pin)
    ad_readings.append(ad_val)
    if len(ad_readings) > 1:
        ad_diff.append(ad_readings[-1] - ad_readings[-2])
    else:
        ad_diff.append(0)
    ad_ma.append(sum(ad_diff[-3:]) / min(3, len(ad_diff))) #moving average of differences

    # update threshold
    if len(ad_ma) >= 20:
        recent = ad_ma[-100:] if len(ad_ma) >= 100 else ad_ma
        if max(recent) != min(recent):
            threshold = 0.2 * (max(recent) - min(recent)) / 2
            if args.debug:
                print(f"threshold: {threshold:.3f}")

    # go reverse after  seconds
    if not turnaround_done and total_time >= 5: #turnaround_done just for whether or not we have turned around, didn't know how else to do it
        print("reverse")
        args.rps = -args.rps #rps is now negative, this also makes it easy cause we don't have to adjust all our other coefficeints. 
        turnaround_done = True # turned around
        integral=0
        previous_error=0
        last_rps_time=time.time()
        ad_ma.clear()
        time.sleep(0.5)
        car.set_motor(20, forward=False)
    # count transitions
    if len(ad_ma) >= 3:
        if ad_ma[-1] > threshold and not is_dark:
            transition_count += 1
            is_dark = 1
        elif ad_ma[-1] < -threshold and is_dark:
            transition_count += 1
            is_dark = 0

    # update PID
    if elapsed >= 0.4:
        actual_rps = transition_count / 4 / elapsed if transition_count > 0 else (RPSs[-1] if RPSs else 0) #use the last value if we don't have transitions to detect
        RPSs.append(actual_rps)
        last_rps_time = current_time
        transition_count = 0

        incline_adjustment = yaccel * 3.0 #when yacell is <0 and going uphil, this will cause a larger erro and require more RPS. WHen >0 (downhill) this requires less error control. 
      #3 is an estimate, we need to test and see which values work
      
      #pid, same as other objecties
        error = (args.rps + incline_adjustment) - actual_rps
        integral += error * elapsed
        integral = max(min(integral, 100/args.Ki), -100/args.Ki)
      
        derivative = (error - previous_error) / elapsed if elapsed > 0 else 0
        previous_error = error

        pwm = args.Kp * error + args.Ki * integral + args.Kd * derivative + args.rps * 11.5
        pwm = min(max(abs(pwm), 0), 100)
        car.set_motor(abs(pwm), forward = (args.rps>0))

        if args.debug:
            print(f"{total_time:.2f}s: rps={actual_rps:.2f}, yaccel={yaccel:.3f}, pwm={pwm:.1f}")

    # log the data for graphs
    if current_time - last_sample_time >= sample_interval:
        time_log.append(total_time)
        rps_log.append(RPSs[-1] if RPSs else 0)
        pwm_log.append(pwm)
        last_sample_time = current_time

    time.sleep(0.01)

car.set_motor(0)
car.stop()

# save to file
with open(f"bonus_car_{args.rps}rps.txt", 'w') as f:
    f.write("Time(s)\tRPS\tPWM\tYaccel(g)\n")
    for i in range(len(time_log)):
        f.write(f"{time_log[i]:.3f}\t{rps_log[i]:.2f}\t{pwm_log[i]:.1f}\t{yaccel_log[i]:.3f}\n")

# plot results
plt.figure(figsize=(10,5))
plt.plot(time_log, rps_log, label='rps')
plt.plot(time_log, pwm_log, label='pwm')
plt.plot(time_log, yaccel_log, label='y-accel (g)')
plt.xlabel("time (s)")
plt.title("bonus: rps + pwm vs time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(f"bonus_plot_{args.rps}rps.png")
plt.show()
