import numpy as np
import time
from time import sleep
import argparse
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from picar import PiCar
import matplotlib.pyplot as plt


#parameters
parser = argparse.ArgumentParser(description="Compute RPS in real-time with PiCar")
parser.add_argument('--mock_car', action='store_true', help='Use mock car instead of real hardware')
parser.add_argument('--rps', type=float, default=5.0, help='RPS to run')
parser.add_argument('--tim', type=int, default=10, help='Time to run (s)')
parser.add_argument('--adSample', type=int, default=5, help='Sampling Rate (ms)')
parser.add_argument('--speedCalc', type=int, default=400, help='Speed calc interval (ms)')
parser.add_argument('--adDelay', type=int, default=0, help='Delay before data collection (s)')
parser.add_argument('--motorDelay', type=int, default=1, help='Delay to start motor (s)')
parser.add_argument('--Kp', type=float, default=9.2, help='Proportional Value to multiply by the error')
parser.add_argument('--Ki', type=float, default=5.5, help='Value to multiply by the integral of the error')
parser.add_argument('--Kd', type=float, default=0.0, help='Value to multiply by the derivative of the error')
parser.add_argument('--debug', action='store_true', help='Enable debug mode')
parser.add_argument('--pwm_base', type=float, default=12, help='PWMSlope')

args = parser.parse_args()

if args.debug:
    print(f'Arguments: {vars(args)}')

#initialize
car = PiCar(mock_car=args.mock_car)

#variables
times = []  
ad_readings = []  
ad_diff = []  
ad_ma = []  #moving average
transitions = []
RPSs = [] 

is_dark = 0  
transition_count = 0  
threshold = 0 
motor_started = False
start_time = time.time()
last_rps_time = start_time
transitions_start_time = 0
previous_error = 0
integral = 0
AD_pin = 0 

# Main loop
while time.time() - start_time < args.tim:
    current_time = time.time()
    next_sample_time = start_time + len(times) * args.adSample / 1000  #calculates when to take next same

    #make sure its time to start motor and that motor has not already been started
    if current_time >= start_time + args.motorDelay and not motor_started:
        print(f"Starting motor at {current_time - start_time:.3f}s")
        car.set_motor(0)  # Initialize
        motor_started = True
        sleep(args.adDelay)  #delay

    #start sampling
    if current_time >= next_sample_time:
        #adc
        ad_val = car.adc.read_adc(AD_pin)
        ad_readings.append(ad_val)
        times.append(current_time - start_time)

        #diff
        if len(ad_readings) == 1:
            diff = 0  #used for first reading
        else:
            diff = ad_readings[-1] - ad_readings[-2]
        ad_diff.append(diff)
  
        #moving average, accounted for first few readings
        if len(ad_diff) == 1:
            ad_ma.append(ad_diff[-1])
        elif len(ad_diff) == 2:
            ad_ma.append((ad_diff[-1] + ad_diff[-2]) / 2)
        else:
            ad_ma.append((ad_diff[-1] + ad_diff[-2] + ad_diff[-3]) / 3)

        #calculate threshold
        if motor_started and len(ad_ma) >= 20:
            if len(ad_ma) == 20 or (len(ad_ma) % 100 == 0):
                # use recent readings
                recent_changes = ad_ma[-100:] if len(ad_ma) >= 100 else ad_ma
                if max(recent_changes) != min(recent_changes):  #no division by 0
                    threshold = 0.2 * (max(recent_changes) - min(recent_changes)) / 2
                    if args.debug:
                        print(f"New threshold: {threshold}")

        #transitions
        if motor_started and len(ad_ma) >= 20:
            if transitions_start_time == 0:
                transitions_start_time = current_time
                
            if ad_ma[-1] > threshold and is_dark == 0:  # Light to dark
                transitions.append(1)
                is_dark = 1
                transition_count += 1
            elif ad_ma[-1] < -threshold and is_dark == 1:  # Dark to light
                transitions.append(-1)
                is_dark = 0
                transition_count += 1
            else:
                transitions.append(0)  # No transition
        else:
            transitions.append(0)  #no tenough data

        # calcRPS every speed calc and make sure we have enough entries
        if motor_started and current_time - last_rps_time >= args.speedCalc / 1000 and len(ad_ma) >= 20:
            #reset time elapsed
            time_elapsed = current_time - last_rps_time
            
            # calc speed
            if transition_count > 0 and time_elapsed > 0.05:
                actual_speed = transition_count / 4 / time_elapsed  #4 transitions in 1 rotation
                transition_count = 0  
            else:
                actual_speed = 0 if not RPSs else RPSs[-1]  # if no measurement, use the last one
                
            RPSs.append(actual_speed)
            last_rps_time = current_time
            
            if args.debug:
                print(f"Time: {current_time - start_time:.3f}s, RPS: {actual_speed:.2f}")

            #PID
            error = args.rps - actual_speed
            integral += error * (args.speedCalc / 1000)  #integrate
            
            # make sure integral is onot dividing by 0
            integral = max(min(integral, 100/args.Ki), -100/args.Ki) if args.Ki != 0 else 0
            
            derivative = (error - previous_error) / (args.speedCalc / 1000)
            
            #new pwmm
            pwm_value = args.Kp * error + args.Ki * integral + args.Kd * derivative
            
            #base_pwm is our estimate
            pwm_value += args.rps * args.pwm_base
            
            #keep in bounds
            pwm_value = min(max(pwm_value, 0), 100)
            
            car.set_motor(pwm_value)
            
            previous_error = error
        else:
            #fill in blank rps values
            RPSs.append(RPSs[-1] if RPSs else 0)


car.stop()

with open('car_noload_5rps.txt', 'w') as file:
    file.write(f"{args.adSample / 1000:.4f}\n")  # First line = sampling interval
    for i in range(len(times)):
        file.write(f"{times[i]:.4f}\t{ad_readings[i]:.0f}\t{RPSs[i]:.3f}\n")

#Overshoot
#First find when motor actually starts spinning
start_index = 0
for i in range(len(times)):
    if times[i] >= args.motorDelay:
        start_index = i
        break

max_rps = max(RPSs[start_index:])
overshoot = max_rps - args.rps


#Steady State Error
samples_per_second = int(1.0 / (args.adSample / 1000)) #second worth of data
final_rps_entries = RPSs[-samples_per_second:] #last second of data
final_rps = sum(final_rps_entries) / len(final_rps_entries) #take the mean
steady_state_error = final_rps - args.rps

#Response time RPS reaches 90% of final value, finds time from when 90% value is met and when motor starts spinning


ninety_percent_value = 0.9 * final_rps
response_time = 0
for i in range(start_index, len(RPSs)):
    if RPSs[i] >= ninety_percent_value:
        response_time = times[i] - times[start_index]
        break
        

print(f"Target RPS             : {args.rps}")
print(f"Final Steady RPS       : {final_rps:.3f}")
if response_time is not None:
    print(f"Response Time (s)      : {response_time:.3f}")
else:
    print("Response Time (s)      : Not reached")
print(f"Overshoot (RPS)        : {overshoot:.3f}")
print(f"Steady-State Error (RPS): {steady_state_error:.3f}")
car.stop()
#Velocity/time graph

plt.figure(figsize=(10, 5))
plt.plot(times, RPSs, label="Velocity (RPS)", color='blue')
plt.xlabel("Time (s)")
plt.ylabel("Velocity (RPS)")
plt.title(f"Velocity vs Time of 5 rps Step Response")
plt.legend()
plt.grid(True)
#plt.axvline(x=response_time, color='red', linestyle='--', label='Response Time')
#plt.axhline(y=max_rps, color='orange', linestyle='--', label='Max RPS')
plt.savefig("plot_9c_mock.png")
plt.show()

#FFT
# FFT
ad_array = np.array(ad_readings)
n = len(ad_array)
T = args.adSample / 1000  # Sampling interval in seconds

# Check if there's enough data
if n < 2:
    print("Error: Not enough data for FFT analysis.")
else:
    # Use second half of data (if intentional) or full array
    ad_array_half = ad_array[n//2:]  # Or use ad_array for full data
    n_half = len(ad_array_half)

    if n_half < 2:
        print("Error: Not enough data in second half for FFT analysis.")
    else:
        # Compute FFT
        yf = fft(ad_array_half)
        xf = fftfreq(n_half, T)[:n_half//2]  # Positive frequencies only

        # Compute magnitude (normalize by length of ad_array_half)
        fft_magnitude = 2.0 / n_half * np.abs(yf[:n_half//2])

        # Plot
        plt.figure(figsize=(10, 5))
        plt.plot(xf, fft_magnitude, color='purple')
        plt.title("FFT of ADC Readings")
        plt.xlabel("Frequency (Hz)")
        plt.ylabel("Magnitude")
        plt.xlim(0, 20)
        plt.ylim(0, 100)
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("fft_9c_mock.png")
        plt.show()


