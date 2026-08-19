import time
import argparse
import numpy as np
import cv2
from picar import PiCar

parser = argparse.ArgumentParser(description='Drive toward a blue bin with visual targeting.')
parser.add_argument('--mock_car', action='store_true', help='Use mock car instead of real hardware')
parser.add_argument('--tim', type=int, default=10, help='Max time to run (s)')
parser.add_argument('--delay', type=float, default=0.2, help='Time between image captures')
parser.add_argument('--delta', type=float, default=0.7, help='Adjustment factor for servo')
parser.add_argument('--debug', action='store_true', help='Enable debug mode')
args = parser.parse_args()

car = PiCar(mock_car=False, threaded=True)
car.set_motor(0)
car.set_swivel_servo(0)
car.set_steer_servo(0)
car.set_nod_servo(-5)
print("Car initialized.")
time.sleep(1)
start_dist = car.read_distance()
print(f"Starting distance: {start_dist:.1f} cm")

'''if start_dist > 914:  # Make sure we are sarting <30 ft
    print("Too far")
    car.stop()
    exit()
'''

def findBlue(array, counter=None):
    if array is None:
        print("Warning: Received None image in findBlue")
        return 0
        
#    array = cv2.flip(array, 0)
    height, width = array.shape[:2]
    center_x = width // 2
    center_y = height // 2
    

    
    array_rgb = array 
    #cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
    
    #RGB to HSV
    hsv = cv2.cvtColor(array_rgb, cv2.COLOR_RGB2HSV)
    
    
    mask = cv2.inRange(hsv, (100, 50, 50), (140, 255, 255))
    
    mask_blur = cv2.GaussianBlur(mask, (5, 5), 0)
    
    thresh = cv2.threshold(mask_blur, 50, 255, cv2.THRESH_BINARY)[1]
    
    if args.debug:
        cv2.imwrite('array_rgb.jpg', array_rgb)
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
    
#draw circle 
    img2 = array_rgb.copy()
    cv2.circle(img2, (cX, cY), 5, (0, 0, 255), 2)  # red circle
    
    #used for debugging
    if args.debug or counter is not None:
        filename = f"frame_{counter:03d}.jpg" if counter is not None else 'blue.jpg'
        cv2.imwrite(filename, cv2.cvtColor(img2, cv2.COLOR_RGB2BGR))
        if args.debug:
            print(f"Saved {filename}, centroid at ({cX}, {cY}), offset: ({x_cord}, {y_cord})")
    
    #angle
    theta = np.arctan2(x_cord, center_y - cY)
    degrees = np.degrees(theta)
    
    if args.debug:
        print(f'Angle: {degrees} degrees')
    
    changein_PWM = degrees / 9  #adjust servo
    return changein_PWM


start_time = time.time()
counter = 0
current_position = 0

print("Starting to drive")

while time.time() - start_time < args.tim:
    current_time = time.time()
    if current_time - start_time >= counter * args.delay:
      
        img = car.get_image()
        if img is None: #make sure we have image
            print("Image capture failed")
            continue

        
        adjustment = findBlue(img, counter) * args.delta
        current_position += adjustment
        current_position = max(-10, min(10, current_position))  # limit swivel
        car.set_steer_servo(current_position)

        # get distance
        
        dist = car.read_distance()
        if dist is None:
            dist = 305
        if args.debug:
            print(f"Distance to bin: {dist:.1f} cm")

        # reach bin
        if dist < 25:
            print("Reached bin — stopping.")
            car.set_motor(0)
            break
        else:
            if dist >304.8:
                speed = 80
                car.set_motor(speed)
            else:
                #speed = min(100, max(30, 80 * (dist / 304.8)))
                if dist >=100 and dist<200:
                   speed=70

                elif dist <=304.8 and dist>=200:
                   speed = 90
                elif dist >=25 and dist<35:
                   speed= 10
                elif dist>=35 and dist< 50:
                   speed = 25
                elif dist>=50 and dist <100:
                   speed = 45
                
                else:
                   norm = min(dist, 304.8) / 304.8
                   speed = 30 + 70 * (norm ** 2)

                car.set_motor(speed)

        counter += 1

    time.sleep(0.01)


#car.stop()
car.set_motor(0)
car.stop()
total_time = time.time() - start_time
print(f"Finished. Total time: {total_time:.2f} seconds.")
